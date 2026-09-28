"""MuJoCo のバックエンド（胴体固定）。

実機の arm_sdk では、内蔵コントローラの出力と、こちらの指令が weight でブレンドされる。
MuJoCo（公式モデルを直接読む）には内蔵コントローラが無いので、次のように近似する:
- 「内蔵コントローラ」= 開始時の姿勢を hold_kp / hold_kd で保持する PD
- 腕の目標 = weight × 指令 + (1 − weight) × 開始時の姿勢（kp / kd も同様にブレンド）
lowcmd（uses_weight=False）の近似では、ブレンドせず指令をそのまま使う。

重力補償（gravity_compensation）について: PD 制御だけだと、腕の重さの分だけ目標より下がる
（Kp=60 で数十 mrad）。実機の arm_sdk で内蔵コントローラが重力を補償するかは未確認、
lowcmd（デバッグモード）では補償されない。設定で切り替えて差を確認できるようにしてある。

数値計算の安定性について（2026-09-28 実測）: 公式モデルは関節のアーマチュア（モータの回転子の
見かけの慣性）も減衰も 0 で、手首ロールの慣性は約 0.00037 kg·m² しかない。PD の Kd 項を
陽的に（今の速度だけで）計算すると kd·dt/I ≈ 8 となり、安定条件（< 2）を超えて発散した。
そこで Kd 項は MuJoCo の関節減衰（dof_damping）に入れ、積分器を implicitfast（減衰を陰的に解く）
にして計算させる。物理モデルの値は変えない。Kp 項と重力補償はトルクとして与える
（トルクの上限は公式モデルの actuatorfrcrange で効く。減衰の分はこの上限の外になる点だけが実機と違う）。
"""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from ..robot_model import JOINT_NAMES, NUM_MOTORS, load_model
from .backend import ArmBackend
from .types import JointCommand, JointState


class SimBackend(ArmBackend):
    def __init__(
        self,
        robot_cfg: dict[str, Any],
        sim_cfg: dict[str, Any],
        hold_kp: np.ndarray,
        hold_kd: np.ndarray,
        uses_weight: bool = True,
        dry_run: bool = False,
    ) -> None:
        self.name = "sim(arm_sdk 近似)" if uses_weight else "sim(lowcmd 近似)"
        self.uses_weight = uses_weight
        self.dry_run = dry_run
        self._robot_cfg = robot_cfg
        self._sim_cfg = sim_cfg
        self._hold_kp = np.asarray(hold_kp, dtype=float)
        self._hold_kd = np.asarray(hold_kd, dtype=float)
        self._cmd: JointCommand | None = None
        self.model: Any = None
        self.data: Any = None

    def open(self) -> None:
        import mujoco

        self._mj = mujoco
        self.model = load_model(self._robot_cfg, fixed_base=True)
        # Kd 項を陰的に解くため（上の説明を参照）
        self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.data = mujoco.MjData(self.model)
        # 関節と qpos / qvel / actuator の対応（胴体固定なので素直に 0..28 のはずだが、名前で引く）
        self._qadr = np.array([self.model.joint(n).qposadr[0] for n in JOINT_NAMES])
        self._vadr = np.array([self.model.joint(n).dofadr[0] for n in JOINT_NAMES])
        self._act = np.array([self.model.actuator(n).id for n in JOINT_NAMES])
        q0 = np.asarray(self._sim_cfg.get("initial_q", [0.0] * NUM_MOTORS), dtype=float)
        self.data.qpos[self._qadr] = q0
        mujoco.mj_forward(self.model, self.data)
        self._hold_q = q0.copy()
        self._gravity_comp = bool(self._sim_cfg.get("gravity_compensation", True))
        self._realtime = bool(self._sim_cfg.get("realtime", False))
        print(f"[sim] MuJoCo 開始（重力補償={'あり' if self._gravity_comp else 'なし'}）")

    def read_state(self) -> JointState:
        d = self.data
        return JointState(
            q=d.qpos[self._qadr].copy(),
            dq=d.qvel[self._vadr].copy(),
            motor_mode=np.ones(NUM_MOTORS, dtype=int),
            mode_machine=int(self._sim_cfg.get("mode_machine", 5)),
            stamp=time.monotonic(),
        )

    def send(self, cmd: JointCommand) -> None:
        if not self.dry_run:
            self._cmd = cmd.copy()

    def _effective_targets(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """今の指令から、各関節の (q, kp, kd, tau) を決める。"""
        q, kp, kd = self._hold_q.copy(), self._hold_kp.copy(), self._hold_kd.copy()
        tau = np.zeros(NUM_MOTORS)
        cmd = self._cmd
        if cmd is None:
            return q, kp, kd, tau
        active = (cmd.kp > 0) | (cmd.kd > 0)
        if self.uses_weight:
            w = float(np.clip(cmd.weight, 0.0, 1.0))
            q[active] = w * cmd.q[active] + (1 - w) * q[active]
            kp[active] = w * cmd.kp[active] + (1 - w) * kp[active]
            kd[active] = w * cmd.kd[active] + (1 - w) * kd[active]
            tau[active] = w * cmd.tau[active]
        else:
            # lowcmd は全関節に指令が要る。指令の無い関節は脱力（kp=kd=0）
            q, kp, kd, tau = cmd.q.copy(), cmd.kp.copy(), cmd.kd.copy(), cmd.tau.copy()
        return q, kp, kd, tau

    def tick(self, dt: float) -> None:
        mj, m, d = self._mj, self.model, self.data
        n_sub = max(1, int(round(dt / m.opt.timestep)))
        t_start = time.monotonic()
        q_des, kp, kd, tau_ff = self._effective_targets()
        m.dof_damping[self._vadr] = kd
        for _ in range(n_sub):
            q = d.qpos[self._qadr]
            tau = kp * (q_des - q) + tau_ff
            if self._gravity_comp:
                # qfrc_bias = 重力 + コリオリ。関節ごとの重力トルクを打ち消す
                tau = tau + d.qfrc_bias[self._vadr]
            d.ctrl[self._act] = tau
            mj.mj_step(m, d)
        if self._realtime:
            rest = dt - (time.monotonic() - t_start)
            if rest > 0:
                time.sleep(rest)

    def close(self) -> None:
        pass
