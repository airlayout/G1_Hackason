"""腕の指令部分（send_arm_cmd 相当）。HANDOFF 5章の安全要件 1〜5 をここに実装する。

    with ArmCommander(backend, arm_cfg, ...) as arm:   # 開始: 現在の姿勢のまま weight を 0→1
        arm.move_to(q_target)                           # 関節空間で補間して移動、動いたかを確認
    # ブロックを抜けると（正常終了・例外・Ctrl+C・SIGTERM のどれでも）安全に終了する

安全要件との対応:
1. weight を急に切り替えない → start() / safe_stop() で weight_ramp_s 秒かけて上げ下げする。
   上げる間は、開始時に読んだ関節角を目標にして姿勢を保つ
2. SIGINT / SIGTERM / 例外のすべてで安全に終了 → __exit__ と シグナルハンドラ → safe_stop()。
   プランA: 現在の姿勢を保ったまま weight を 1→0。プランB: 現在の姿勢を保持してから送信をやめる
3. 関節リミット、1ステップの最大移動量、作業空間の箱 → safety.py。外れる目標は送らずに拒否する
4. （IK が届かない場合の拒否は、タスク2の IK 側で行う）
5. dry-run（backend.dry_run）と確認モード（confirm=True で各段階で Enter を待つ）
6. 腰は動かさない → 腰は開始時の角度を保持する指令を送り、実測の腰の角度が開始時から
   waist_max_deviation_rad 以上ずれたら中止する（WaistDeviationError）
さらに、送信後に lowstate で実際に動いたかを確認する（check_motion）。
プランB（実機の lowcmd）では、開始前に必ず「座った状態、または吊り下げた状態か」を人に確認する。
重力補償（gravity_compensation.scale > 0）のときは、腕の重力トルク × 倍率を tau として送る。
"""

from __future__ import annotations

import signal
import time
from collections.abc import Callable, Sequence
from types import FrameType
from typing import Any

import numpy as np

from ..robot_model import JOINT_NAMES, LEFT_ARM_IDX, NUM_MOTORS, RIGHT_ARM_IDX, WAIST_IDX
from .backend import ArmBackend
from .gravity import GravityModel
from .safety import (
    UnsafeTargetError,
    WorkspaceBox,
    check_finite,
    check_joint_limits,
    interpolate_joint,
    rate_limit,
)
from .types import JointCommand, JointState

WRIST_IDX: frozenset[int] = frozenset({19, 20, 21, 26, 27, 28})


class StopRequested(Exception):
    """Ctrl+C / SIGTERM / 確認モードでの中止。safe_stop() で安全に終了する。"""


class NoMotionError(RuntimeError):
    """指令は送ったのに、lowstate の関節角が変わらなかった（「エラーなしで動かない」失敗）。"""


class StateTimeoutError(RuntimeError):
    """lowstate が途切れた。"""


class WaistDeviationError(RuntimeError):
    """腰の角度が開始時からずれた（腕の動きで上体が引っ張られている、など）。"""


class ArmCommander:
    def __init__(
        self,
        backend: ArmBackend,
        arm_cfg: dict[str, Any],
        joint_lower: np.ndarray,
        joint_upper: np.ndarray,
        fk: Callable[[np.ndarray], np.ndarray] | None = None,
        workspace: WorkspaceBox | None = None,
        confirm: bool = False,
        input_fn: Callable[[str], str] = input,
        gravity: GravityModel | None = None,
    ) -> None:
        """
        joint_lower / joint_upper: 29関節の可動範囲（motor 番号順、モデルから読む）
        fk: 29関節の角度 → 手先の位置（pelvis 座標）。与えれば作業空間の箱で目標を確認する
        gravity: 重力補償に使うモデル。設定の倍率が 0 より大きいときは必須
        """
        self.backend = backend
        self.cfg = arm_cfg
        self.dt = 1.0 / float(arm_cfg["control_hz"])
        side = arm_cfg["arm"]
        if side not in ("left", "right"):
            raise ValueError(f"arm は left か right: {side}")
        self.arm_idx = np.array(RIGHT_ARM_IDX if side == "right" else LEFT_ARM_IDX)
        self.lower = np.asarray(joint_lower, dtype=float)
        self.upper = np.asarray(joint_upper, dtype=float)
        self.fk = fk
        self.workspace = workspace
        self.confirm = confirm
        self._input = input_fn

        saf = arm_cfg["safety"]
        self.max_step = float(saf["max_joint_speed_rad_s"]) * self.dt
        self.margin = float(saf["limit_margin_rad"])
        self.waist_max_dev = float(saf["waist_max_deviation_rad"])
        self._waist_start: np.ndarray | None = None

        gc = arm_cfg["gravity_compensation"]
        self.gc_scale = float(gc["scale"])
        self.gc_tau_max = float(gc["tau_max_nm"])
        if not (0.0 <= self.gc_scale <= 1.0):
            raise ValueError(f"gravity_compensation.scale は 0.0〜1.0: {self.gc_scale}")
        if self.gc_tau_max <= 0.0:
            raise ValueError(f"gravity_compensation.tau_max_nm は正の値: {self.gc_tau_max}")
        self.gc_waist_scale = float(gc.get("waist_scale", 0.0))
        self.gc_waist_tau_max = float(gc.get("waist_tau_max_nm", 15.0))
        if not (0.0 <= self.gc_waist_scale <= 1.0):
            raise ValueError(f"gravity_compensation.waist_scale は 0.0〜1.0: {self.gc_waist_scale}")
        if self.gc_waist_tau_max <= 0.0:
            raise ValueError(f"gravity_compensation.waist_tau_max_nm は正の値: {self.gc_waist_tau_max}")
        if (self.gc_scale > 0.0 or self.gc_waist_scale > 0.0) and gravity is None:
            raise ValueError("重力補償の倍率が 0 より大きいのに、GravityModel が渡されていない")
        self.gravity = gravity
        self._arms_idx = np.array(list(LEFT_ARM_IDX) + list(RIGHT_ARM_IDX))

        self._joints, self._kp, self._kd = self._build_gains()
        self._cmd = JointCommand()
        self._cmd.kp[:] = self._kp
        self._cmd.kd[:] = self._kd
        self._started = False
        self._stopping = False
        self._stopped = False
        #: Ctrl+C / SIGTERM / 確認モードで中止したときの理由（中止していなければ None）
        self.stop_reason: str | None = None
        self._old_handlers: dict[int, Any] = {}

    # ---- 準備 -----------------------------------------------------------------

    def _build_gains(self) -> tuple[list[int], np.ndarray, np.ndarray]:
        """指令する関節と、関節ごとの kp / kd を決める。"""
        g = self.cfg["gains"]
        kp = np.zeros(NUM_MOTORS)
        kd = np.zeros(NUM_MOTORS)
        arms = list(LEFT_ARM_IDX) + list(RIGHT_ARM_IDX)
        for i in arms:
            src = g["wrist"] if i in WRIST_IDX else g["arm"]
            kp[i], kd[i] = src["kp"], src["kd"]
        if self.backend.uses_weight:
            joints = list(arms)
            if self.cfg["arm_sdk"]["include_waist_hold"]:
                for i in WAIST_IDX:
                    kp[i] = self.cfg["arm_sdk"]["waist_hold"]["kp"]
                    kd[i] = self.cfg["arm_sdk"]["waist_hold"]["kd"]
                joints += list(WAIST_IDX)
        else:
            # lowcmd は全関節に指令が要る（送らない関節は脱力する）。腕以外は開始時の姿勢を保持
            hold_kp = np.asarray(self.cfg["lowcmd"]["hold_kp"], dtype=float)
            hold_kd = np.asarray(self.cfg["lowcmd"]["hold_kd"], dtype=float)
            others = [i for i in range(NUM_MOTORS) if i not in arms]
            kp[others] = hold_kp[others]
            kd[others] = hold_kd[others]
            joints = list(range(NUM_MOTORS))
        return sorted(joints), kp, kd

    @property
    def commanded_arm_q(self) -> np.ndarray:
        """いま指令している片腕 7 関節の角度（開始直後は開始時の実測値）。"""
        return self._cmd.q[self.arm_idx].copy()

    def measured_q(self) -> np.ndarray:
        """lowstate の最新の関節角（29、motor 番号順）。途切れていれば StateTimeoutError。"""
        return self._fresh_state().q.copy()

    @property
    def joints(self) -> list[int]:
        """指令を書き込む関節の motor 番号。"""
        return self._joints

    # ---- with 文とシグナル --------------------------------------------------------

    def __enter__(self) -> "ArmCommander":
        for sig in (signal.SIGINT, signal.SIGTERM):
            self._old_handlers[sig] = signal.signal(sig, self._on_signal)
        try:
            self.start()
        except BaseException as e:
            if isinstance(e, StopRequested):
                self.stop_reason = str(e) or "中止"
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, exc_type: type | None, exc: BaseException | None, tb: Any) -> bool:
        if isinstance(exc, StopRequested):
            self.stop_reason = str(exc) or "中止"
        elif exc is not None:
            print(f"[arm] 例外で終了する: {type(exc).__name__}: {exc}")
        try:
            self.safe_stop()
        finally:
            for sig, h in self._old_handlers.items():
                signal.signal(sig, h)
            self._old_handlers.clear()
        # StopRequested（Ctrl+C など）は安全に止めたので飲み込む。それ以外の例外は外へ伝える
        return isinstance(exc, StopRequested)

    def _on_signal(self, signum: int, frame: FrameType | None) -> None:
        name = signal.Signals(signum).name
        if self._stopping:
            print(f"[arm] {name} を受信したが、安全終了の途中なので続ける")
            return
        print(f"[arm] {name} を受信。安全に終了する")
        raise StopRequested(name)

    def _confirm(self, label: str) -> None:
        if not self.confirm:
            return
        ans = self._input(f"[confirm] {label} → Enter で実行 / q で中止: ")
        if ans.strip().lower() == "q":
            raise StopRequested("確認モードで中止")

    # ---- 送信の 1 周期 ----------------------------------------------------------

    def _fresh_state(self) -> JointState:
        st = self.backend.read_state()
        if st is None:
            raise StateTimeoutError("lowstate を受信していない")
        age = time.monotonic() - st.stamp
        if age > float(self.cfg["state_timeout_s"]):
            raise StateTimeoutError(f"lowstate が {age:.2f} 秒途切れている")
        return st

    def _check_waist(self, st: JointState) -> None:
        if self._waist_start is None:
            return
        dev = np.abs(st.q[list(WAIST_IDX)] - self._waist_start)
        if float(np.max(dev)) > self.waist_max_dev:
            raise WaistDeviationError(
                f"腰の角度が開始時から {np.degrees(dev).round(2)}° ずれた"
                f"（上限 {np.degrees(self.waist_max_dev):.1f}°）。中止する"
            )

    @property
    def _waist_gc_active(self) -> bool:
        # 腰の重力補償はプランB（lowcmd）のときだけ。arm_sdk では内蔵コントローラと二重になるため送らない
        return self.gc_waist_scale > 0.0 and not self.backend.uses_weight

    def _update_gravity_tau(self, st: JointState | None) -> None:
        """両腕（とプランB で有効なら腰）の tau に、重力トルク × 倍率（上限付き）を入れる。倍率 0 なら 0。"""
        self._cmd.tau[:] = 0.0
        if self.gravity is None or (self.gc_scale <= 0.0 and not self._waist_gc_active):
            return
        imu = st.imu_quat if st is not None else None
        # 腕は目標の角度で計算する（実測値を使うと、センサの揺れがそのままトルクに乗るため）。
        # 腰は実測値を使う（腰が倒れると腕にかかる重力の向きが変わるため）
        q = self._cmd.q.copy()
        if st is not None:
            q[list(WAIST_IDX)] = st.q[list(WAIST_IDX)]
        g = self.gravity.torques(q, imu)
        self._cmd.tau[self._arms_idx] = np.clip(self.gc_scale * g[self._arms_idx], -self.gc_tau_max, self.gc_tau_max)
        if self._waist_gc_active:
            w = list(WAIST_IDX)
            self._cmd.tau[w] = np.clip(self.gc_waist_scale * g[w], -self.gc_waist_tau_max, self.gc_waist_tau_max)

    def _send_tick(self) -> None:
        st = None
        if not self._stopping:
            st = self._fresh_state()
            self._check_waist(st)
        else:
            st = self.backend.read_state()
        self._update_gravity_tau(st)
        self.backend.send(self._cmd)
        self.backend.tick(self.dt)

    # ---- 開始 -----------------------------------------------------------------

    def _wait_state(self) -> JointState:
        t_end = time.monotonic() + float(self.cfg["start_state_wait_s"])
        while time.monotonic() < t_end:
            st = self.backend.read_state()
            if st is not None:
                return st
            time.sleep(0.05)
        raise StateTimeoutError(
            f"{self.cfg['start_state_wait_s']} 秒待っても lowstate が届かない。"
            "有線接続・NIC 名（network_interface）・G1 の電源を確認すること"
        )

    def start(self) -> None:
        st = self._wait_state()
        self.backend.verify_peer(st)
        print(f"[arm] 経路 {self.backend.name}、腕 {self.cfg['arm']}、mode_machine={st.mode_machine}")
        expected = int(self.cfg["expected_mode_machine"])
        if st.mode_machine != expected:
            raise RuntimeError(
                f"mode_machine が {st.mode_machine}（期待値 {expected}）。機体構成が違うので中止する"
            )
        disabled = [JOINT_NAMES[i] for i in self._joints if st.motor_mode[i] != 1]
        if disabled:
            raise RuntimeError(
                "モータが無効（mode=0、ゼロトルク）: " + ", ".join(disabled) + "。"
                "この状態では送信しても動かない。リモコンでダンピング（FSM 1）に入れて有効化すること"
            )
        check_finite(st.q, "lowstate の関節角")
        if self.backend.needs_support_check:
            ans = self._input(
                "[confirm] プランB（デバッグモード）はバランス制御が止まる。"
                "ロボットは座った状態、または吊り下げた状態か？ → Enter で続行 / q で中止: "
            )
            if ans.strip().lower() == "q":
                raise StopRequested("支持の確認で中止")
        if self.gc_scale > 0.0:
            print(f"[arm] 重力補償（腕）: 倍率 {self.gc_scale}、上限 {self.gc_tau_max} Nm")
        if self._waist_gc_active:
            print(f"[arm] 重力補償（腰）: 倍率 {self.gc_waist_scale}、上限 {self.gc_waist_tau_max} Nm")
        self._waist_start = st.q[list(WAIST_IDX)].copy()

        # 目標 = 今の姿勢。これで weight を上げても腕は動かない
        self._cmd.q[:] = st.q
        self._cmd.weight = 0.0
        self._confirm("開始（現在の姿勢を保持して制御を始める）")
        self._started = True
        if self.backend.uses_weight:
            ramp_s = float(self.cfg["safety"]["weight_ramp_s"])
            n = max(1, int(np.ceil(ramp_s / self.dt)))
            print(f"[arm] weight 0→1（{ramp_s} 秒）")
            for k in range(1, n + 1):
                self._cmd.weight = k / n
                self._send_tick()
        else:
            print("[arm] lowcmd: 全関節を現在の姿勢で保持して開始")
            for _ in range(max(1, int(0.5 / self.dt))):
                self._send_tick()

    # ---- 目標の確認と移動 ----------------------------------------------------------

    def validate_arm_target(self, q_arm: np.ndarray, what: str = "目標", check_workspace: bool = True) -> np.ndarray:
        """片腕 7 関節の目標を確認し、29 関節の指令ベクトルにして返す。通らなければ UnsafeTargetError。

        check_workspace=False は、開始時にいた姿勢へ戻るときだけに使う（開始姿勢は作業空間の箱の外にあってよい）。
        """
        q_arm = np.asarray(q_arm, dtype=float)
        if q_arm.shape != (len(self.arm_idx),):
            raise UnsafeTargetError(f"{what}の要素数が {q_arm.shape}（{len(self.arm_idx)} のはず）")
        check_finite(q_arm, what)
        check_joint_limits(
            q_arm,
            self.lower[self.arm_idx],
            self.upper[self.arm_idx],
            self.margin,
            [JOINT_NAMES[i] for i in self.arm_idx],
        )
        q_full = self._cmd.q.copy()
        q_full[self.arm_idx] = q_arm
        if check_workspace and self.fk is not None and self.workspace is not None:
            self.workspace.check(self.fk(q_full), f"{what}の手先")
        return q_full

    @property
    def started(self) -> bool:
        """start() が済み、指令を送り始めたか（エラーの表示を分けるため）。"""
        return self._started

    def move_to(self, q_arm: np.ndarray, duration: float | None = None, label: str = "移動",
                check_workspace: bool = True) -> None:
        """片腕 7 関節を、関節空間で smoothstep 補間して目標へ動かし、動いたかを確認する。"""
        if not self._started or self._stopping:
            raise RuntimeError("start() の前、または終了処理中に move_to() が呼ばれた")
        self.validate_arm_target(q_arm, check_workspace=check_workspace)
        q_arm = np.asarray(q_arm, dtype=float)
        q0 = self._cmd.q[self.arm_idx].copy()
        dist = float(np.max(np.abs(q_arm - q0)))
        if duration is None:
            # smoothstep の最大速度は平均の 1.5 倍。それが上限速度の半分になるよう余裕を持たせる
            v = float(self.cfg["safety"]["max_joint_speed_rad_s"])
            duration = max(float(self.cfg["safety"]["min_move_s"]), 3.0 * dist / v)
        self._confirm(f"{label}（最大 {np.degrees(dist):.1f}°、{duration:.1f} 秒）")
        print(f"[arm] {label}: 最大 {np.degrees(dist):.1f}° を {duration:.1f} 秒で")
        q_before = self._fresh_state().q[self.arm_idx].copy()
        q_cmd_start = q0.copy()
        for wp in interpolate_joint(q0, q_arm, duration, self.dt):
            self._cmd.q[self.arm_idx] = rate_limit(self._cmd.q[self.arm_idx], wp, self.max_step)
            self._send_tick()
        # 上限速度で削られた分が残っていれば、追いつくまで送る
        while np.max(np.abs(self._cmd.q[self.arm_idx] - q_arm)) > 1e-9:
            self._cmd.q[self.arm_idx] = rate_limit(self._cmd.q[self.arm_idx], q_arm, self.max_step)
            self._send_tick()
        for _ in range(int(float(self.cfg["safety"]["settle_s"]) / self.dt)):
            self._send_tick()
        self.check_motion(q_before, q_cmd_start, q_arm)

    def follow(self, waypoints_arm: Sequence[np.ndarray], label: str = "軌道") -> None:
        """片腕 7 関節の点列を 1 周期に 1 点ずつ送る（タスク2の直線押し込み用）。

        送る前に全点を確認する。隣り合う点の差が 1 ステップの上限を超える軌道は拒否する。
        """
        if not self._started or self._stopping:
            raise RuntimeError("start() の前、または終了処理中に follow() が呼ばれた")
        wps = [np.asarray(w, dtype=float) for w in waypoints_arm]
        if not wps:
            return
        prev = self._cmd.q[self.arm_idx].copy()
        for k, w in enumerate(wps):
            self.validate_arm_target(w, f"{label}の {k} 点目")
            step = float(np.max(np.abs(w - prev)))
            if step > self.max_step + 1e-9:
                raise UnsafeTargetError(
                    f"{label}の {k} 点目で 1 ステップ {step:.4f} rad（上限 {self.max_step:.4f}）"
                )
            prev = w
        self._confirm(f"{label}（{len(wps)} 点、{len(wps) * self.dt:.1f} 秒）")
        q_before = self._fresh_state().q[self.arm_idx].copy()
        q_cmd_start = self._cmd.q[self.arm_idx].copy()
        for w in wps:
            self._cmd.q[self.arm_idx] = w
            self._send_tick()
        for _ in range(int(float(self.cfg["safety"]["settle_s"]) / self.dt)):
            self._send_tick()
        self.check_motion(q_before, q_cmd_start, wps[-1])

    # ---- 動いたかの確認 ----------------------------------------------------------

    def check_motion(
        self, q_before: np.ndarray, q_cmd_start: np.ndarray, q_target: np.ndarray
    ) -> dict[str, float]:
        """lowstate の関節角が指令どおりに変わったかを確認する。

        q_before: 移動前の実測値、q_cmd_start: 移動前の指令値、q_target: 移動後の指令値。
        指令の変化（q_target − q_cmd_start）が min_commanded_rad より大きい関節で、実測の変化
        （q_after − q_before）がその min_ratio 倍未満なら NoMotionError（送信は成功したのに動かない）。
        指令と実測を別々に差を取るのは、重力で下がっている分（一定のずれ）を判定に混ぜないため。
        追従誤差が大きいときは警告だけ出す。
        """
        mc = self.cfg["safety"]["motion_check"]
        if self.backend.dry_run:
            print("[arm] dry-run なので、動いたかの確認はしない")
            return {}
        q_after = self._fresh_state().q[self.arm_idx]
        commanded = q_target - q_cmd_start
        measured = q_after - q_before
        err = float(np.max(np.abs(q_after - q_target)))
        big = np.abs(commanded) > float(mc["min_commanded_rad"])
        if np.any(big):
            # 腕全体として、指令した変化の向きにどれだけ進んだか（射影の割合）。関節ごとの最小では見ない
            # （押し当てたときに、1 つの関節だけ押し戻されることがあるため。模擬ロボットで手首ピッチが −0.17）
            c, m = commanded[big], measured[big]
            progress = float(np.dot(m, c) / np.dot(c, c))
            if progress < float(mc["min_ratio"]):
                per_joint = ", ".join(f"{JOINT_NAMES[i]}={r:+.2f}" for i, r in zip(self.arm_idx[big], m / c))
                raise NoMotionError(
                    f"指令どおりに動いていない（腕全体で指令の {progress:.2f} 倍しか動いていない。関節ごと: "
                    f"{per_joint}）。motor の mode・経路（arm_sdk / lowcmd）・weight を確認すること"
                )
        tol = float(self.cfg["safety"]["tracking_tolerance_rad"])
        mark = "OK" if err <= tol else "⚠️ 追従誤差が大きい"
        print(f"[arm] 確認: 最大の追従誤差 {np.degrees(err):.2f}°（許容 {np.degrees(tol):.1f}°）{mark}")
        return {"max_error_rad": err}

    # ---- 安全な終了 ----------------------------------------------------------------

    def safe_stop(self) -> None:
        """安全に終了する。何度呼んでもよい。"""
        if self._stopped or self._stopping:
            return
        self._stopping = True
        try:
            if not self._started:
                return
            st = self.backend.read_state()
            if st is not None and np.all(np.isfinite(st.q)):
                # 今いる姿勢を目標にする（古い目標へ戻ろうとして動かないように）
                self._cmd.q[self.arm_idx] = st.q[self.arm_idx]
            if self.backend.uses_weight:
                ramp_s = float(self.cfg["safety"]["weight_ramp_s"])
                n = max(1, int(np.ceil(ramp_s / self.dt)))
                w0 = self._cmd.weight
                print(f"[arm] 安全終了: 現在の姿勢のまま weight {w0:.2f}→0（{ramp_s} 秒）")
                for k in range(1, n + 1):
                    self._cmd.weight = w0 * (1 - k / n)
                    self._send_tick()
            else:
                hold_s = float(self.cfg["lowcmd"]["stop_hold_s"])
                print(f"[arm] 安全終了: 現在の姿勢を {hold_s} 秒保持してから送信をやめる")
                for _ in range(int(hold_s / self.dt)):
                    self._send_tick()
                print(
                    "[arm] lowcmd の送信を止めた。デバッグモードのままなので、"
                    "リモコンのダンピング（L2+B）などで機体を安全な状態にすること"
                )
        except Exception as e:  # noqa: BLE001  終了処理の失敗は表示して続ける（元の例外を隠さない）
            print(f"[arm] ⚠️ 安全終了の途中で失敗: {type(e).__name__}: {e}。リモコンで L2+B を押すこと")
        finally:
            # _stopping は True のまま残す（終了後に届いたシグナルで例外を投げないため）
            self._stopped = True
            print("[arm] 終了")
