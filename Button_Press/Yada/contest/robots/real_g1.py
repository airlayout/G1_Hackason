"""実機の G1（と、同じ口を持つ模擬 G1）。ランナーとエージェントを、実機に持っていくときに使う。

- 上半身: rt/arm_sdk（DDS）。J1-gen の DdsBackend で送る。weight は開始時に 0 → 1、終了時に 1 → 0 へ少しずつ変える
- 下半身: LocoClient.Move（RPC の "sport" サービス）。速度の指令が変わったときだけ送る
- 関節の状態: rt/lowstate（J1-gen の DdsBackend が受け取る）
- 頭カメラ: PC2 の RGB-D サーバ（ZMQ）。J1-gen の RgbdZmqSource で受け取る

つなぎ先:
- 実機: network_interface に G1 につないでいる有線 NIC の名前（domain 0）、カメラは PC2 の IP
- 模擬 G1（sim/mujoco/g1_sim_server.py）: network_interface = "lo"（domain 1 になる）、カメラは 127.0.0.1

ボタンの点灯は実機では読めないので、StepInfo.lit は空（判定は人が行う。模擬 G1 ならサーバが判定する）。
advance() は実際の時刻で次の周期まで待つ（シミュレーションと違い、act() が遅いとその分だけ前の指令のまま）。

⚠️ 実機で動かす前に: J1-gen の real/REAL_DAY_PROCEDURE.md の手順（ダンピング状態の確認、吊り下げ、非常停止の用意）
   に従うこと。このクラスには、J1-gen の ArmCommander の安全の手順のうち、weight の上げ下げ、lowstate の途切れの
   検出、Ctrl+C での安全な終了だけを入れている（作業空間の箱、動いたかの確認、腰のずれの確認はまだ無い）。
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from common.j1gen_bridge import j1gen

from ..interface import NUM_JOINTS, UPPER_BODY_IDX, Observation
from .base import Robot, StepInfo, joint_gains

# lowstate がこの秒数より古ければ止める（J1-gen の configs/arm.yaml の state_timeout_s と同じ）
STATE_TIMEOUT_S = 0.5
# 下半身の速度の指令を送る最短の間隔 [秒]（RPC なので毎周期は送らない）
LOCO_MIN_INTERVAL_S = 0.1


class StateTimeoutError(RuntimeError):
    pass


class RealG1Robot(Robot):
    base_enabled = True

    def __init__(self, contest_cfg: dict[str, Any], network_interface: str, camera_host: str,
                 rgbd_port: int = 5556, dry_run: bool = False, weight_ramp_s: float = 2.0,
                 use_loco: bool = True) -> None:
        self.name = "sim_dds" if network_interface == "lo" else "real"
        self._dds = j1gen("dds")
        backend_mod = j1gen("arm.backend_dds")
        self._JointCommand = j1gen("arm.types").JointCommand
        self.backend = backend_mod.DdsBackend("arm_sdk", network_interface, domain_id=0, dry_run=dry_run)
        self.backend.open()
        st = self._wait_state(10.0)
        self.backend.verify_peer(st)

        # 画像は別のスレッドで受け取り続け、最新の 1 枚だけを持つ（制御の周期を、画像の到着で待たせないため。
        # 同じスレッドで受け取ると、50 Hz のつもりが画像の 15 fps に引きずられて約 20 Hz になった）
        self.camera = j1gen("camera_rgbd").RgbdZmqSource(camera_host, rgbd_port, timeout_ms=200)
        self.camera.open()
        self._frame = None
        self._frame_lock = threading.Lock()
        self._cam_stop = False
        self._cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._cam_thread.start()
        t_end = time.monotonic() + 10.0
        while self._frame is None and time.monotonic() < t_end:
            time.sleep(0.05)

        self._loco = None
        if use_loco:
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

            self._loco = LocoClient()
            self._loco.SetTimeout(3.0)
            self._loco.Init()
        self._last_base = np.zeros(3)
        self._last_base_t = 0.0

        kin = j1gen("kinematics")
        import pinocchio as pin

        m = pin.buildModelFromUrdf(str(j1gen("config").resolve_repo_path(
            j1gen("config").load_config("robot.yaml")["model"]["urdf_path"])))
        names = [m.names[i] for i in range(1, m.njoints)]
        assert names == list(kin.JOINT_NAMES), "URDF の関節の並びが motor 番号順と違う"
        self._lo, self._hi = m.lowerPositionLimit.copy(), m.upperPositionLimit.copy()
        self._kp, self._kd = joint_gains(contest_cfg["gains"])
        self._ramp_s = float(weight_ramp_s)
        self._t_start: float | None = None
        self._q_start = st.q.copy()
        self._q_cmd = st.q.copy()
        self._weight = 0.0

    # ---- Robot ----------------------------------------------------------------

    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._lo.copy(), self._hi.copy()

    def upper_gains(self) -> tuple[np.ndarray, np.ndarray]:
        idx = list(UPPER_BODY_IDX)
        return self._kp[idx].copy(), self._kd[idx].copy()

    def observe(self, t: float) -> Observation:
        st = self._fresh_state()
        with self._frame_lock:
            fr = self._frame
        if fr is None:
            raise RuntimeError("頭カメラの画像が届かない（RGB-D サーバの起動とアドレスを確認）")
        ins = fr.intrinsics
        K = np.array([[ins.fx, 0.0, ins.cx], [0.0, ins.fy, ins.cy], [0.0, 0.0, 1.0]])
        return Observation(rgb=fr.color_bgr[:, :, ::-1].copy(), depth=fr.depth_m(), K=K, q=st.q.copy(),
                           dq=st.dq.copy(), t=float(t))

    def command(self, q_upper: np.ndarray, base_cmd: np.ndarray) -> None:
        now = time.monotonic()
        if self._t_start is None:
            self._t_start = now
        # weight を 0 → 1 に少しずつ上げる（その間に急に指令の姿勢へ切り替わらないように）
        self._weight = min(1.0, (now - self._t_start) / self._ramp_s) if self._ramp_s > 0 else 1.0
        self._q_cmd[list(UPPER_BODY_IDX)] = q_upper
        self._send(self._q_cmd, self._weight)
        self._send_base(np.asarray(base_cmd, dtype=float), now)

    def advance(self, dt: float) -> StepInfo:
        self.backend.tick(dt)
        return StepInfo(lit={}, max_contact_force=None)

    def close(self) -> None:
        """今の姿勢を保ったまま weight を 1 → 0 に下げ、下半身を止める（J1-gen の ArmCommander.safe_stop と同じ考え）。"""
        try:
            if self._loco is not None and np.any(self._last_base != 0.0):
                self._loco.StopMove()
            st = self.backend.read_state()
            q_hold = st.q.copy() if st is not None else self._q_cmd.copy()
            w0 = self._weight
            n = max(1, int(self._ramp_s / 0.02))
            for k in range(n):
                self._send(q_hold, w0 * (1.0 - (k + 1) / n))
                self.backend.tick(0.02)
        finally:
            self._cam_stop = True
            self._cam_thread.join(timeout=2.0)
            self.camera.close()
            self.backend.close()

    # ---- 内部 -----------------------------------------------------------------

    def _camera_loop(self) -> None:
        while not self._cam_stop:
            f = self.camera.read_rgbd()
            if f is not None:
                with self._frame_lock:
                    self._frame = f

    def _send(self, q29: np.ndarray, weight: float) -> None:
        cmd = self._JointCommand()
        idx = list(UPPER_BODY_IDX)
        cmd.q[idx] = q29[idx]
        cmd.kp[idx] = self._kp[idx]
        cmd.kd[idx] = self._kd[idx]
        cmd.weight = float(weight)
        self.backend.send(cmd)

    def _send_base(self, base: np.ndarray, now: float) -> None:
        if self._loco is None:
            return
        changed = not np.allclose(base, self._last_base)
        if changed and now - self._last_base_t >= LOCO_MIN_INTERVAL_S:
            if np.any(base != 0.0):
                self._loco.Move(float(base[0]), float(base[1]), float(base[2]), continous_move=True)
            else:
                self._loco.StopMove()
            self._last_base, self._last_base_t = base.copy(), now

    def _wait_state(self, timeout_s: float) -> Any:
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            st = self.backend.read_state()
            if st is not None:
                return st
            time.sleep(0.05)
        raise TimeoutError(f"{timeout_s} 秒待っても lowstate が届かない（接続・NIC 名・模擬 G1 の起動を確認）")

    def _fresh_state(self) -> Any:
        st = self.backend.read_state()
        if st is None or time.monotonic() - st.stamp > STATE_TIMEOUT_S:
            raise StateTimeoutError(f"lowstate が {STATE_TIMEOUT_S} 秒以上届いていない。止める")
        if len(st.q) != NUM_JOINTS:
            raise RuntimeError("lowstate の関節の数が違う")
        return st
