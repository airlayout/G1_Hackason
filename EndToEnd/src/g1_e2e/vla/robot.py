"""Orin NX側のカメラとarm_sdk。PyTorch・Isaac・VLAの依存は持ち込まない。"""

from __future__ import annotations

import argparse
import base64
import os
import signal
import threading
import time

import numpy as np

from .config import DEFAULT_CONFIG, load_config
from .control import ArmControl
from .http import serve


class RobotBridge:
    def __init__(self, config: dict, *, execute: bool = False):
        self.config = config
        self.control = ArmControl(config["arm"], execute=execute)
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.jpeg = None
        self.camera_error = ""
        self.publish = None
        self.hardware = None
        self.mode_machine = 5
        self.threads = []

    def handle(self, method: str, path: str, body: dict | None):
        with self.lock:
            now = time.monotonic()
            if method == "GET" and path == "/api/status":
                return "application/json", self.status(now)
            if method == "GET" and path == "/api/observation":
                identifier = self.control.observe(now=now)
                return "application/json", {
                    **self.status(now),
                    "observation_id": identifier,
                    "jpeg": base64.b64encode(self.jpeg).decode(),
                }
            if method == "POST" and path == "/api/arm/start":
                session = self.control.start(now=now, max_task_s=self.config["max_task_s"])
                return "application/json", {"session": session, "execute": self.control.execute}
            if method == "POST" and path == "/api/arm/chunk":
                try:
                    self.control.accept(
                        body["session"], body["observation_id"], body["positions"], body["action_hz"], now=now
                    )
                except (ValueError, KeyError, TypeError):
                    # 古いsessionからの通信では、新しいsessionを停止させない。
                    if body.get("session") == self.control.session:
                        self.control.stop("無効な指令を受信しました")
                    raise
                return "application/json", self.status(now)
            if method == "POST" and path == "/api/arm/stop":
                self.control.stop()
                return "application/json", self.status(now)
        raise FileNotFoundError(path)

    def status(self, now: float) -> dict:
        return {
            **self.control.status(now=now),
            "camera_error": self.camera_error,
            "mode_machine": self.mode_machine,
            "hand_control": False,
        }

    def start_thread(self, target, name: str) -> None:
        thread = threading.Thread(target=target, name=name, daemon=True)
        thread.start()
        self.threads.append(thread)

    def run_control(self) -> None:
        interval = 1 / self.config["arm"]["control_hz"]
        previous = time.monotonic()
        was_active = False
        while not self.done.is_set():
            begin = time.monotonic()
            try:
                with self.lock:
                    q, weight = self.control.tick(now=begin, dt=begin - previous)
                    active = self.control.phase != "idle" or weight > 0
                    # 起動時・待機中はweight=0の指令も送らず、制御権を取得しない。
                    if self.publish is not None and (active or was_active):
                        self.publish(q, weight)
                    was_active = active
            except Exception as error:
                with self.lock:
                    self.control.stop(f"送信失敗: {error}")
            previous = begin
            self.done.wait(max(0, interval - (time.monotonic() - begin)))

    def close(self) -> None:
        with self.lock:
            self.control.stop("終了")
        # 送信スレッドを止める前に、arm_sdkのweightをゼロへ戻す。
        deadline = time.monotonic() + self.config["arm"]["ramp_s"] * 2 + 0.5
        while time.monotonic() < deadline:
            with self.lock:
                if self.control.weight == 0:
                    break
            time.sleep(0.02)
        self.done.set()
        for thread in self.threads:
            thread.join(timeout=1)
        if self.hardware:
            self.hardware.close()


class UnitreeArm:
    def __init__(self, bridge: RobotBridge, *, interface: str, domain: int):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        # native locomotionを維持する。ReleaseMode・lowcmdは使わない。
        ChannelFactoryInitialize(domain, interface)
        self.bridge = bridge
        self.crc = CRC()
        self.last_tick = None
        self.new_command = unitree_hg_msg_dds__LowCmd_
        self.publisher = None
        if bridge.control.execute:
            self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self.publisher.Init()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.receive, 1)

    def receive(self, message) -> None:
        if self.crc.Crc(message) != message.crc:
            return
        with self.bridge.lock:
            if message.tick == self.last_tick:
                return
            if message.mode_pr != 0 or message.mode_machine != 5:
                self.bridge.control.stop("29DoF・PR座標の状態ではありません")
                return
            if len(message.motor_state) < 29 or any(m.mode != 1 for m in message.motor_state[15:29]):
                self.bridge.control.stop("腕のモーター状態が不正です")
                return
            if any(m.motorstate != 0 for m in message.motor_state[15:29]):
                self.bridge.control.stop("腕のモーターが異常を報告しました")
                return
            try:
                self.bridge.control.update_state(
                    [m.q for m in message.motor_state[:29]],
                    [m.dq for m in message.motor_state[:29]],
                    now=time.monotonic(),
                )
                self.last_tick = message.tick
            except ValueError as error:
                self.bridge.control.stop(str(error))

    def publish(self, q: np.ndarray, weight: float) -> None:
        if self.publisher is None:
            return
        message = self.new_command()
        message.mode_pr, message.mode_machine = 0, 5
        for i, joint in enumerate(range(15, 29)):
            motor = message.motor_cmd[joint]
            motor.mode, motor.q = 1, float(q[i])
            motor.dq, motor.tau = 0.0, 0.0
            motor.kp = self.bridge.config["arm"]["kp"][i]
            motor.kd = self.bridge.config["arm"]["kd"][i]
        message.motor_cmd[29].q = float(weight)
        message.crc = self.crc.Crc(message)
        if not self.publisher.Write(message):
            raise RuntimeError("arm_sdkの送信に失敗しました")

    def close(self) -> None:
        if self.publisher:
            self.publisher.Close()
        self.subscriber.Close()


def camera_loop(bridge: RobotBridge, device: str) -> None:
    import cv2

    capture = cv2.VideoCapture(int(device) if device.isdigit() else device)
    try:
        if not capture.isOpened():
            raise RuntimeError(f"カメラを開けません: {device}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(cv2.CAP_PROP_FPS, 30)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        while not bridge.done.is_set():
            ok, bgr = capture.read()
            captured_at = time.monotonic()
            if not ok:
                raise RuntimeError("カメラを読み取れません")
            ok, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise RuntimeError("画像をJPEGに変換できません")
            with bridge.lock:
                bridge.jpeg = jpeg.tobytes()
                bridge.control.camera_at = captured_at
                bridge.camera_error = ""
    except Exception as error:
        with bridge.lock:
            bridge.camera_error = str(error)
            bridge.control.stop(str(error))
    finally:
        capture.release()


def main() -> None:
    parser = argparse.ArgumentParser(description="G1側のVLA用アーム・カメラブリッジ（既定は送信なし）")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--camera", required=True, help="/dev/v4l/by-id/...のRGBカメラ")
    parser.add_argument("--execute", action="store_true", help="rt/arm_sdkへの実送信を有効にする")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    token = os.environ.get("G1_ARM_TOKEN", "")
    if args.bind not in {"127.0.0.1", "localhost", "::1"} and len(token) < 24:
        parser.error("外部公開には24文字以上のG1_ARM_TOKENが必要です。SSH転送なら不要です")
    bridge = RobotBridge(load_config(args.config), execute=args.execute)
    server = serve(bridge, args.bind, args.port, token=token)
    try:
        bridge.hardware = UnitreeArm(bridge, interface=args.interface, domain=args.domain)
        bridge.publish = bridge.hardware.publish
        bridge.start_thread(bridge.run_control, "arm-50hz")
        bridge.start_thread(lambda: camera_loop(bridge, args.camera), "camera")

        def stop(_signum, _frame) -> None:
            raise KeyboardInterrupt

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        print(
            f"[腕] http://{args.bind}:{args.port} / 実送信={'有効' if args.execute else '無効'}", flush=True
        )
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        server.server_close()


if __name__ == "__main__":
    main()
