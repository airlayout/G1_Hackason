"""同じSDK入出力でhold / wrist wave / 既存ONNX歩行ポリシーを実行する。"""

import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np

from g1_e2e.contracts import JointCommand, Observation
from g1_e2e.data.geometry import rotation_xyzw
from g1_e2e.dds.codec import LowLevelCodec
from g1_e2e.robot import RobotSpec


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=42)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--controller", choices=["hold", "wave", "unitree"], default="hold")
    parser.add_argument("--velocity", type=float, nargs=3, default=[0.3, 0, 0])
    parser.add_argument("--seconds", type=float, default=10, help="simulation tick基準の秒数")
    parser.add_argument("--wall-timeout", type=float, default=180)
    parser.add_argument(
        "--exercise-watchdog", action="store_true", help="試験用: 不正CRC/AB送信と250msの送信停止"
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.seconds <= 0 or args.wall_timeout <= 0:
        parser.error("seconds/wall-timeoutは正数です")
    spec = RobotSpec.load(root / "configs/robot/g1_29dof.yaml")
    codec, lock = LowLevelCodec(), threading.Lock()
    sdk = codec.sdk
    sdk.initialize(args.domain, args.interface)
    latest, received, bad_crc = None, 0, 0

    def receive(message):
        nonlocal latest, received, bad_crc
        with lock:
            if sdk.crc.Crc(message) != message.crc:
                bad_crc += 1
                return
            latest = message
            received += 1

    subscriber = sdk.Subscriber("rt/lowstate", sdk.LowState)
    subscriber.Init(receive)
    publisher = sdk.Publisher("rt/lowcmd", sdk.LowCmd)
    publisher.Init()
    controller = None
    if args.controller == "unitree":
        from g1_e2e.inference.unitree_onnx import create_controller

        controller = create_controller(
            robot=spec, control_dt=0.02, options={"root": str(root), "velocity": args.velocity}
        )
        controller.reset()
    sent, previous_tick, policy_tick, exercised = 0, -1, -20, False
    command = JointCommand(spec.default_position.copy())
    started = time.monotonic()
    reached = False
    try:
        while time.monotonic() - started < args.wall_timeout:
            with lock:
                state = latest
            if state is None:
                time.sleep(0.005)
                continue
            if state.tick >= args.seconds * 1000:
                reached = True
                break
            codec.mode_machine = state.mode_machine
            if state.mode_pr != 0:
                raise ValueError("PR LowStateのみ対応しています")
            if state.tick < previous_tick:
                raise ValueError("tickが逆行しました。再起動/wrap時はcontrollerを再初期化してください")
            if state.tick != previous_tick:
                previous_tick = state.tick
                q = np.asarray([m.q for m in state.motor_state[:29]], dtype=np.float32)
                dq = np.asarray([m.dq for m in state.motor_state[:29]], dtype=np.float32)
                orientation = np.asarray(state.imu_state.quaternion, dtype=np.float32)[[1, 2, 3, 0]]
                observation = Observation(
                    0,
                    state.tick // 20,
                    state.tick / 1000,
                    q,
                    dq,
                    np.asarray([m.tau_est for m in state.motor_state[:29]], dtype=np.float32),
                    orientation,
                    np.asarray(state.imu_state.gyroscope, dtype=np.float32),
                    rotation_xyzw(orientation).T @ np.array([0, 0, -1], dtype=np.float32),
                )
                if args.controller == "unitree" and state.tick - policy_tick >= 20:
                    command = controller.act(observation)
                    policy_tick = state.tick
                elif args.controller == "wave":
                    target = spec.default_position.copy()
                    target[19] += 0.25 * np.sin(2 * np.pi * 0.7 * state.tick / 1000)
                    command = JointCommand(target)
            message = codec.encode_command(command, spec.stiffness, spec.damping)
            if args.exercise_watchdog and state.tick >= 1000 and not exercised:
                message.crc ^= 1
                publisher.Write(message)
                message.mode_pr = 1
                message.crc = sdk.crc.Crc(message)
                publisher.Write(message)
                time.sleep(0.25)
                exercised = True
                continue
            if not publisher.Write(message):
                raise RuntimeError("LowCmdのDDS送信に失敗しました")
            sent += 1
            time.sleep(0.002)
        summary = {
            "controller": args.controller,
            "domain": args.domain,
            "interface": args.interface,
            "states_received": received,
            "bad_state_crc": bad_crc,
            "commands_sent": sent,
            "last_tick_ms": state.tick if state else None,
            "reached_duration": reached,
            "watchdog_exercised": exercised,
            "sdk_commit": sdk.commit,
        }
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x") as file:
                file.write(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary), flush=True)
        if not reached:
            raise TimeoutError("指定したsimulation tickまでLowStateを受信しませんでした")
    finally:
        subscriber.Close()
        publisher.Close()


if __name__ == "__main__":
    main()
