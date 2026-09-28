"""ループバックの模擬ロボット: MuJoCo の G1 を、実機と同じ DDS のトピックと ZMQ のカメラで見せる（タスク6のリハーサル用）。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/sim_robot_server.py
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/sim_robot_server.py --fault ignore_arm_sdk
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/sim_robot_server.py --fault lowstate_dropout --fault-after 8

実機用のスクリプトを、当日と同じコマンドのまま（--network-interface lo と --camera-config camera_sim.yaml を付けて）
このマシンの中だけで試せる:
- DDS: rt/lowstate を 100 Hz で送る。rt/arm_sdk（weight でブレンド。内蔵コントローラの重力補償を近似）と
  rt/lowcmd（デバッグモード。重力補償なし）を受け取って MuJoCo に反映する
- カメラ: 頭カメラの RGB + 深度を、深度付きの形式（5556）と RGB 互換の形式（5555）で配信する
- シーン: 机とボトル（configs/sim_scene.yaml）。胴体は固定。ハンドには衝突判定の箱がある（ボトルに当たって止まる）

**実機と混ざらないように、通信はループバックだけに固定している（変える引数は無い）:**
- DDS はネットワークの口 lo と domain 1（G1 は domain 0）。lowstate の reserve[0] に目印を入れる
- ZMQ のカメラは 127.0.0.1 だけで待ち受ける

わざと起こせる故障（--fault。複数可。--fault-after 秒たってから起こす。既定は最初から）:
- ignore_arm_sdk: rt/arm_sdk を無視する（arm_sdk が効かない機体。腕が動かない）
- motor_mode0: 全モータの mode を 0 にする（ゼロトルク）
- lowstate_dropout: lowstate を送るのをやめる（通信が途切れる）
- mode_machine: mode_machine を 2 にする（機体構成が違う）
- waist_sag: 腰ピッチを前に倒す力をかける（腰が倒れていく）
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path
from types import FrameType
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.arm.backend_sim import SimBackend  # noqa: E402
from common.arm.types import JointCommand  # noqa: E402
from common.config import load_config  # noqa: E402
from common.dds import SIM_DOMAIN_ID, SIM_INTERFACE, SIM_MARKER, TOPIC_LOWSTATE  # noqa: E402
from common.rgbd_protocol import encode_legacy_rgb, encode_rgbd  # noqa: E402
from common.robot_model import ARM_SDK_WEIGHT_IDX, NUM_MOTORS  # noqa: E402
from common.sim_camera import SimHeadCamera  # noqa: E402
from common.sim_scene import detection_pose  # noqa: E402

FAULTS = ("ignore_arm_sdk", "motor_mode0", "lowstate_dropout", "mode_machine", "waist_sag")
# 指令がこの秒数届かなければ、内蔵コントローラが元の姿勢を保持する状態に戻す（プログラムが終わったあと）
COMMAND_TIMEOUT_S = 0.5
CAMERA_HOST = "127.0.0.1"  # ループバックだけで待ち受ける
WAIST_SAG_TORQUE = 25.0  # [Nm] 腰ピッチを前に倒す力（Kp=300 の保持で約 5° 倒れる）


def msg_to_command(msg: Any, use_weight: bool) -> JointCommand:
    """LowCmd_ → JointCommand（mode=1 の関節だけ。weight は motor_cmd[29].q）。"""
    c = JointCommand()
    for i in range(NUM_MOTORS):
        m = msg.motor_cmd[i]
        if m.mode == 1:
            c.q[i], c.kp[i], c.kd[i], c.tau[i] = m.q, m.kp, m.kd, m.tau
    c.weight = float(msg.motor_cmd[ARM_SDK_WEIGHT_IDX].q) if use_weight else 1.0
    return c


class SimRobotServer:
    def __init__(self, faults: list[str], fault_after: float, rgbd_port: int, rgb_port: int,
                 camera_fps: float, control_hz: float = 100.0) -> None:
        for f in faults:
            if f not in FAULTS:
                raise ValueError(f"--fault は {FAULTS} のどれか: {f}")
        self.faults = set(faults)
        self.fault_after = fault_after
        self.robot_cfg = load_config("robot.yaml")
        self.scene_cfg = load_config("sim_scene.yaml")
        arm_cfg = load_config("arm.yaml")
        sim_cfg = dict(arm_cfg["sim"])
        sim_cfg.update(scene=self.scene_cfg, realtime=True, emulate="arm_sdk", gravity_compensation=True,
                       initial_q=list(detection_pose(self.scene_cfg, np.zeros(NUM_MOTORS))))
        self.backend = SimBackend(self.robot_cfg, sim_cfg, np.asarray(arm_cfg["lowcmd"]["hold_kp"], float),
                                  np.asarray(arm_cfg["lowcmd"]["hold_kd"], float), uses_weight=True)
        self.dt = 1.0 / control_hz
        self.camera_period = 1.0 / camera_fps
        self.rgbd_port, self.rgb_port = rgbd_port, rgb_port
        self._lock = threading.Lock()
        self._pending: tuple[str, JointCommand] | None = None
        self._stop = False
        self.counts = {"arm_sdk": 0, "lowcmd": 0, "arm_sdk_ignored": 0, "lowstate": 0, "frames": 0}

    # ---- DDS ----------------------------------------------------------------

    def _on_cmd(self, topic: str, msg: Any) -> None:
        if topic == "arm_sdk" and self._fault_active("ignore_arm_sdk"):
            self.counts["arm_sdk_ignored"] += 1
            return
        cmd = msg_to_command(msg, use_weight=topic == "arm_sdk")
        with self._lock:
            self._pending = (topic, cmd)
            self.counts[topic] += 1

    def _fault_active(self, name: str) -> bool:
        return name in self.faults and time.monotonic() - self._t0 >= self.fault_after

    def _open_dds(self) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        # 実機と混ざらないよう、口と domain は固定（引数で変えられない）
        ChannelFactoryInitialize(SIM_DOMAIN_ID, SIM_INTERFACE)
        self._pub = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self._pub.Init()
        self._sub_arm = ChannelSubscriber("rt/arm_sdk", LowCmd_)
        self._sub_arm.Init(lambda m: self._on_cmd("arm_sdk", m), 10)
        self._sub_low = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self._sub_low.Init(lambda m: self._on_cmd("lowcmd", m), 10)
        self._msg = unitree_hg_msg_dds__LowState_()
        self._crc = CRC()

    def _publish_state(self, tick: int) -> None:
        if self._fault_active("lowstate_dropout"):
            return
        st = self.backend.read_state()
        m = self._msg
        mode = 0 if self._fault_active("motor_mode0") else 1
        for i in range(NUM_MOTORS):
            m.motor_state[i].q = float(st.q[i])
            m.motor_state[i].dq = float(st.dq[i])
            m.motor_state[i].mode = mode
        m.mode_machine = 2 if self._fault_active("mode_machine") else 5
        m.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
        m.reserve = [SIM_MARKER, 0, 0, 0]
        m.tick = tick
        m.crc = self._crc.Crc(m)
        self._pub.Write(m)
        self.counts["lowstate"] += 1

    # ---- カメラ ----------------------------------------------------------------

    def _open_camera(self) -> None:
        import mujoco
        import zmq

        # 描画は別のスレッドで、描画用の MjData に関節角を写して行う（描画は 1 枚約 0.1 秒かかり、
        # 物理と同じループで描くと lowstate も物理も遅れるため）
        self._render_data = mujoco.MjData(self.backend.model)
        self._render_qpos = self.backend.data.qpos.copy()
        self._zctx = zmq.Context()
        self._zrgbd = self._zctx.socket(zmq.PUB)
        self._zrgb = self._zctx.socket(zmq.PUB)
        for s, port in ((self._zrgbd, self.rgbd_port), (self._zrgb, self.rgb_port)):
            s.setsockopt(zmq.SNDHWM, 2)
            s.setsockopt(zmq.LINGER, 0)
            s.bind(f"tcp://{CAMERA_HOST}:{port}")

    def _camera_loop(self) -> None:
        import mujoco

        # OpenGL の都合で、描画の部品は描画するスレッドの中で作る
        camera = SimHeadCamera(self.backend.model, self._render_data, self.robot_cfg)
        try:
            while not self._stop:
                t = time.monotonic()
                with self._lock:
                    self._render_data.qpos[:] = self._render_qpos
                mujoco.mj_forward(self.backend.model, self._render_data)
                self._publish_frame(camera)
                rest = self.camera_period - (time.monotonic() - t)
                if rest > 0:
                    time.sleep(rest)
        finally:
            camera.close()

    def _publish_frame(self, camera: SimHeadCamera) -> None:
        import zmq

        f = camera.render()
        try:
            self._zrgbd.send(encode_rgbd(f), zmq.NOBLOCK)
            self._zrgb.send_string(encode_legacy_rgb(f.color_bgr[:, :, ::-1].copy(), f.camera, f.timestamp),
                                   zmq.NOBLOCK)
        except zmq.Again:
            pass
        self.counts["frames"] += 1

    # ---- 本体 -----------------------------------------------------------------

    def stop(self, signum: int = 0, frame: FrameType | None = None) -> None:
        self._stop = True

    def run(self, duration_s: float | None = None) -> None:
        self.backend.open()
        self._t0 = time.monotonic()
        self._open_dds()
        self._open_camera()
        be = self.backend
        waist_pitch = be.model.joint("waist_pitch_joint").dofadr[0]
        print(f"[sim_robot] 開始: DDS は {SIM_INTERFACE} / domain {SIM_DOMAIN_ID}（lowstate {1 / self.dt:.0f} Hz）、"
              f"カメラは {CAMERA_HOST}:{self.rgbd_port}（深度付き）/ {self.rgb_port}（RGB 互換）")
        if self.faults:
            print(f"[sim_robot] 故障: {sorted(self.faults)}（{self.fault_after:.0f} 秒後から）")
        tick = 0
        last_cmd_t: float | None = None
        next_log = time.monotonic() + 5.0
        cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
        cam_thread.start()
        try:
            while not self._stop and (duration_s is None or time.monotonic() - self._t0 < duration_s):
                with self._lock:
                    pending, self._pending = self._pending, None
                if pending is not None:
                    last_cmd_t = time.monotonic()
                elif last_cmd_t is not None and time.monotonic() - last_cmd_t > COMMAND_TIMEOUT_S:
                    # 指令が途絶えた: 内蔵コントローラが開始時の姿勢を保持する状態へ戻す
                    be.uses_weight = True
                    be._gravity_comp = True
                    be.send(JointCommand())
                    last_cmd_t = None
                if pending is not None:
                    topic, cmd = pending
                    be.uses_weight = topic == "arm_sdk"
                    # arm_sdk は内蔵コントローラが動いている（重力を補償する）、lowcmd はデバッグモード（補償なし）
                    be._gravity_comp = topic == "arm_sdk"
                    be.send(cmd)
                be.data.qfrc_applied[:] = 0.0
                if self._fault_active("waist_sag"):
                    be.data.qfrc_applied[waist_pitch] = WAIST_SAG_TORQUE
                be.tick(self.dt)
                with self._lock:
                    self._render_qpos[:] = be.data.qpos
                tick += 1
                self._publish_state(tick)
                now = time.monotonic()
                if now >= next_log:
                    print(f"[sim_robot] {self.counts}")
                    next_log = now + 5.0
        finally:
            self._stop = True
            cam_thread.join(timeout=5.0)
            self._zrgbd.close()
            self._zrgb.close()
            self._zctx.term()
            print(f"[sim_robot] 終了: {self.counts}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--fault", action="append", default=[], choices=FAULTS, help="わざと起こす故障（複数可）")
    p.add_argument("--fault-after", type=float, default=0.0, help="故障を起こすまでの秒数")
    p.add_argument("--rgbd-port", type=int, default=5556)
    p.add_argument("--rgb-port", type=int, default=5555)
    p.add_argument("--camera-fps", type=float, default=10.0)
    p.add_argument("--duration-s", type=float, help="この秒数で止める")
    args = p.parse_args()
    server = SimRobotServer(args.fault, args.fault_after, args.rgbd_port, args.rgb_port, args.camera_fps)
    signal.signal(signal.SIGINT, server.stop)
    signal.signal(signal.SIGTERM, server.stop)
    server.run(args.duration_s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
