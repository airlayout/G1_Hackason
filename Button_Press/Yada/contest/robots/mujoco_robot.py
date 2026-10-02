"""MuJoCo のロボット（G1 + エレベーター乗り場）。腰（pelvis）は固定。

上半身は、実機の arm_sdk と同じく関節の目標の角度を PD で追う（強さは configs/contest.yaml の gains）。
PD の D の項は MuJoCo の関節の減衰（dof_damping）に入れ、積分器を implicitfast にする。
陽的に計算すると、軽い手首のリンクで発散するため（J1-gen の common/arm/backend_sim.py と同じ対策）。
下半身の速度の指令は、腰を固定しているので無視する（base_enabled = False）。

評価セット realistic（realism を渡したとき）は、common/realism.py の乱しを入れる:
体の揺れ（腰の関節をばねで揺らす）、立ち位置のずれ、頭カメラの取り付けの誤差、関節の armature と摩擦、
PD の強さのばらつき、指令の遅れ、深度・カラー・関節のノイズ、画像の遅れとコマ数。
"""

from __future__ import annotations

import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

from common.config import FEATURE_DIR
from common.j1gen_bridge import j1gen_config
from common.realism import ImageDelay, Realism, SensorModel, perturbed_robot_cfg, sway_offset
from common.scene_spec import build_hall_scene

from ..interface import JOINT_NAMES, NUM_JOINTS, UPPER_BODY_IDX, Observation
from .base import Robot, StepInfo, joint_gains

sys.path.insert(0, str(FEATURE_DIR / "sim" / "mujoco"))

from mujoco_hall import BASE_JOINTS, BASE_KP_LIN, HallMujoco, build_hall_model  # noqa: E402

GRAVITY = 9.81


class MujocoRobot(Robot):
    name = "mujoco"
    base_enabled = False

    def __init__(self, scene_cfg: dict[str, Any], contest_cfg: dict[str, Any], viewer: bool = False,
                 realism: Realism | None = None):
        import mujoco

        self._mj = mujoco
        self.realism = realism
        self.scene = build_hall_scene(scene_cfg)
        # エージェントが知っている機体の値（robot.yaml）。シミュレーションのカメラだけ、取り付けの誤差でずらす
        self.robot_cfg = j1gen_config("robot.yaml")
        sim_robot_cfg = perturbed_robot_cfg(self.robot_cfg, realism) if realism is not None else self.robot_cfg
        m = build_hall_model(self.scene, sim_robot_cfg, scene_cfg.get("fingertip"), realism=realism)
        m.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.model, self.data = m, mujoco.MjData(m)

        self._qadr = np.zeros(NUM_JOINTS, dtype=int)
        self._vadr = np.zeros(NUM_JOINTS, dtype=int)
        self._act = np.zeros(NUM_JOINTS, dtype=int)
        self._lo, self._hi = np.zeros(NUM_JOINTS), np.zeros(NUM_JOINTS)
        for i, n in enumerate(JOINT_NAMES):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            self._qadr[i], self._vadr[i] = m.jnt_qposadr[jid], m.jnt_dofadr[jid]
            self._act[i] = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            self._lo[i], self._hi[i] = m.jnt_range[jid]
        # 公称の PD の強さ（エージェントに渡す値）と、実際に効く強さ（realistic では関節ごとにばらつく）
        self._kp, self._kd = joint_gains(contest_cfg["gains"])
        self._kp_eff = self._kp * (realism.kp_scale if realism is not None else 1.0)
        # 上半身だけ重力を補償する（内蔵コントローラの近似。configs/contest.yaml の arm_sdk_gravity_compensation）
        self._gc_upper = bool(contest_cfg.get("arm_sdk_gravity_compensation", False))
        self._upper = np.zeros(NUM_JOINTS, dtype=bool)
        self._upper[list(UPPER_BODY_IDX)] = True
        m.dof_damping[self._vadr] = self._kd

        q0 = np.zeros(NUM_JOINTS)
        for idx, deg in scene_cfg.get("initial_pose_deg", {}).items():
            q0[int(idx)] = np.radians(float(deg))
        self.data.qpos[self._qadr] = q0
        self._pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self._base_act = None
        if realism is not None:
            self._base_act = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in BASE_JOINTS])
            base_q = np.array([m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in BASE_JOINTS])
            # 上下の関節は体の重さで下がるので、重さ ÷ ばねの強さだけ目標を上げる。体は、重さとつり合う位置
            # （= 揺れの位置。上乗せは含めない）に置く（上乗せを含めて置くと、始めた直後に 1.7 cm 落ちた）
            self._z_ff = float(m.body_subtreemass[self._pelvis]) * GRAVITY / BASE_KP_LIN
            self.data.qpos[base_q] = sway_offset(realism, 0.0)
        mujoco.mj_forward(m, self.data)
        self.q_des = q0.copy()
        self._cmd_queue: deque[tuple[float, np.ndarray]] = deque()
        self.hall = HallMujoco(m, self.data, self.scene)

        cam = self.robot_cfg["head_camera"]
        self._cam = cam["name"]
        w, h = int(cam["width"]), int(cam["height"])
        fy = (h / 2.0) / np.tan(np.radians(float(cam["fovy_deg"])) / 2.0)
        self.K = np.array([[fy, 0.0, w / 2.0], [0.0, fy, h / 2.0], [0.0, 0.0, 1.0]])
        self._rgb = mujoco.Renderer(m, height=h, width=w)
        self._depth = mujoco.Renderer(m, height=h, width=w)
        self._depth.enable_depth_rendering()
        self._sensor = SensorModel(realism) if realism is not None else None
        self._images = ImageDelay(realism.image_latency_s, realism.image_fps) if realism is not None else None

        self._viewer = None
        if viewer:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(m, self.data)

    # ---- Robot ----------------------------------------------------------------

    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._lo.copy(), self._hi.copy()

    def upper_gains(self) -> tuple[np.ndarray, np.ndarray]:
        idx = list(UPPER_BODY_IDX)
        return self._kp[idx].copy(), self._kd[idx].copy()

    def observe(self, t: float) -> Observation:
        d = self.data
        q, dq = d.qpos[self._qadr].copy(), d.qvel[self._vadr].copy()
        image_t = float(t)
        if self._images is None:
            rgb, depth = self._render()
        else:
            if self._images.due(t):
                rgb, depth = self._render()
                self._images.push(t, (self._sensor.color(rgb), self._sensor.depth(depth)))
            image_t, (rgb, depth) = self._images.get(t)
            q, dq = self._sensor.joints(q, dq)
        return Observation(rgb=rgb, depth=depth, K=self.K.copy(), q=q, dq=dq, t=float(t),
                           imu_quat=d.xquat[self._pelvis].copy(), image_t=image_t)

    def command(self, q_upper: np.ndarray, base_cmd: np.ndarray) -> None:
        q = self.q_des.copy() if not self._cmd_queue else self._cmd_queue[-1][1].copy()
        q[list(UPPER_BODY_IDX)] = q_upper
        if self.realism is None or self.realism.command_latency_s <= 0:
            self.q_des = q
        else:
            # 指令の遅れ: 送った指令は、遅れの時間がたってから効く
            self._cmd_queue.append((self.data.time + self.realism.command_latency_s, q))

    def advance(self, dt: float) -> StepInfo:
        m, d = self.model, self.data
        n = max(1, int(round(dt / m.opt.timestep)))
        max_f = 0.0
        t0 = time.time()
        diag = {"tip_dist": {}, "depth": {}, "touched": {}}
        for _ in range(n):
            while self._cmd_queue and self._cmd_queue[0][0] <= d.time:
                self.q_des = self._cmd_queue.popleft()[1]
            tau = self._kp_eff * (self.q_des - d.qpos[self._qadr])
            if self._gc_upper:
                tau = tau + np.where(self._upper, d.qfrc_bias[self._vadr], 0.0)
            d.ctrl[self._act] = tau
            if self._base_act is not None:
                d.ctrl[self._base_act] = self._base_targets(d.time)
            self._mj.mj_step(m, d)
            self.hall.update()
            max_f = max(max_f, self.hall.max_robot_contact_force())
            # 周期の中の最小の距離・最大の沈み・触れたか（物理の 1 ステップごとに見る。押した瞬間を逃さないように）
            dg = self.hall.diagnostics()
            for k, v in dg["tip_dist"].items():
                diag["tip_dist"][k] = min(v, diag["tip_dist"].get(k, np.inf))
            for k, v in dg["depth"].items():
                diag["depth"][k] = max(v, diag["depth"].get(k, 0.0))
            for k, v in dg["touched"].items():
                diag["touched"][k] = v or diag["touched"].get(k, False)
        if self._viewer is not None:
            if not self._viewer.is_running():
                raise KeyboardInterrupt("画面が閉じられた")
            self._viewer.sync()
            time.sleep(max(0.0, dt - (time.time() - t0)))
        return StepInfo(lit={b.name: self.hall.lit(b.name) for b in self.scene.buttons}, max_contact_force=max_f,
                        diag=diag)

    def close(self) -> None:
        self._rgb.close()
        self._depth.close()
        if self._viewer is not None:
            self._viewer.close()

    # ---- 内部 -----------------------------------------------------------------

    def _render(self) -> tuple[np.ndarray, np.ndarray]:
        self._rgb.update_scene(self.data, camera=self._cam)
        self._depth.update_scene(self.data, camera=self._cam)
        return self._rgb.render().copy(), self._depth.render().astype(np.float32)

    def _base_targets(self, t: float) -> np.ndarray:
        """体の揺れの関節の目標（揺れ + 上下の重さの打ち消し）。"""
        target = sway_offset(self.realism, t)
        target[2] += self._z_ff
        return target
