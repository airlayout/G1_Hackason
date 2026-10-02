#!/usr/bin/env python3
"""停止した29DoF G1の右腕だけで、調整済みのボタン押下経路を実行する。

既定は計画表示のみ。実機指令には --execute --calibrated が両方必要。
歩行モードは事前に有効であること。ここではモード変更、Damp、ReleaseMode を行わない。
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trajectory import (ALL_ARM_INDICES, ARM_INDICES, ARM_JOINTS, PHASES,
                        interpolate, load_plan, validate_arm_q)


CONTROL_HZ = 50.0
STATE_TIMEOUT = 0.4
TRACKING_LIMIT = 0.5  # [rad] 継続して外れたら押下を中止
MAX_JOINT_SPEED = 0.8  # [rad/s] smoothstep の最大速度で評価


def check_speed(start: tuple[float, ...], end: tuple[float, ...],
                duration: float) -> None:
    peak = max(abs(a - b) for a, b in zip(start, end)) * 1.5 / duration
    if peak > MAX_JOINT_SPEED:
        raise ValueError(f"関節の計画速度 {peak:.2f} rad/s が上限を超えます")


class ArmSdk:
    """歩行モードを維持して rt/arm_sdk へ腕関節だけを送る。"""

    def __init__(self, interface: str) -> None:
        # dry-run は SDK を import しないため、開発PC上でも経路を確認できる。
        from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                                  ChannelPublisher, ChannelSubscriber)
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        ChannelFactoryInitialize(0, interface)
        self._lock = threading.Lock()
        self._state = None
        self._state_at = 0.0
        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._on_state, 10)
        self._publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._publisher.Init()
        self._loco = LocoClient()
        self._loco.SetTimeout(5.0)
        self._loco.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._crc = CRC()
        self._hold = None
        self._last_right = None
        self._last_weight = 0.0

    def _on_state(self, message) -> None:
        with self._lock:
            self._state = message
            self._state_at = time.monotonic()

    def current_arm(self, timeout: float = 5.0) -> tuple[float, ...]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                state, age = self._state, time.monotonic() - self._state_at
            if state is not None and age <= STATE_TIMEOUT:
                if len(state.motor_state) <= max(ALL_ARM_INDICES):
                    raise RuntimeError("LowState の腕関節数が不足しています")
                values = tuple(float(state.motor_state[i].q) for i in ALL_ARM_INDICES)
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError("LowState に不正な関節角があります")
                return values
            time.sleep(0.02)
        raise RuntimeError("新しい rt/lowstate を取得できません")

    def stop_walking(self) -> None:
        # Python SDK の StopMove は SetVelocity(0,0,0) の戻り値を捨てるため、
        # 同じAPIを直接呼んで応答コードも検査する。歩行モードは維持する。
        code = self._loco.SetVelocity(0.0, 0.0, 0.0)
        if code != 0:
            raise RuntimeError(f"StopMove が失敗しました (code={code})")
        time.sleep(1.0)

    def current_configuration(self):
        """運動学と校正姿勢の監視用。脚や腰への指令は送らない。"""
        self.current_arm(timeout=STATE_TIMEOUT)
        with self._lock:
            state, received_at = self._state, self._state_at
            joints = tuple(float(state.motor_state[i].q) for i in range(29))
            rpy = tuple(float(v) for v in state.imu_state.rpy)
            leg_speed = max(abs(float(state.motor_state[i].dq)) for i in range(12))
            gyro = max(abs(float(v)) for v in state.imu_state.gyroscope)
        if len(rpy) != 3 or not all(math.isfinite(v) for v in (*joints, *rpy, leg_speed, gyro)):
            raise RuntimeError("姿勢監視用の実機状態が不正です")
        if time.monotonic() - received_at > STATE_TIMEOUT:
            raise RuntimeError("実機状態が古すぎます")
        return joints, rpy, received_at, leg_speed, gyro

    def set_hold(self, current: tuple[float, ...]) -> None:
        self._hold = list(current)
        self._last_right = tuple(current[7:])

    def publish(self, right: tuple[float, ...], weight: float,
                require_fresh_state: bool = True) -> None:
        if self._hold is None:
            raise RuntimeError("現在姿勢が未取得です")
        validate_arm_q(right)
        if require_fresh_state:
            self.current_arm(timeout=STATE_TIMEOUT)
        targets = self._hold.copy()
        targets[7:] = right
        for index, target in zip(ALL_ARM_INDICES, targets):
            motor = self._cmd.motor_cmd[index]
            motor.q = target
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = 15.0 if index in (19, 20, 21, 26, 27, 28) else 40.0
            motor.kd = 1.5
        self._cmd.motor_cmd[29].q = max(0.0, min(1.0, weight))
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._publisher.Write(self._cmd)
        self._last_right = right
        self._last_weight = weight

    def release(self) -> None:
        """最後の目標姿勢を維持しながら腕の重みを徐々に解除する。"""
        if self._last_right is None or self._last_weight <= 0:
            return
        start = self._last_weight
        for step in range(101):
            weight = start * (1.0 - step / 100.0)
            try:
                self.publish(self._last_right, weight, require_fresh_state=False)
            except Exception as error:  # 通信障害でも解除を試みた事実をログに残す
                print(f"[release] 指令送信に失敗: {error}", file=sys.stderr, flush=True)
                break
            time.sleep(0.02)


def execute(plan: dict, interface: str) -> None:
    arm = ArmSdk(interface)
    arm.stop_walking()
    current = arm.current_arm()
    start = validate_arm_q(current[7:])
    arm.set_hold(current)
    poses = {phase: validate_arm_q(plan["poses"][phase]) for phase in PHASES}
    # 戻り姿勢はシミュレーションの立位姿勢でなく、実機の開始時姿勢にする。
    poses["home"] = start
    previous = start
    for phase in PHASES:
        check_speed(previous, poses[phase], float(plan["durations"][phase]))
        previous = poses[phase]
    print("歩行停止を要求しました。実際に静止し、支持者と非常停止手段があることを目視確認してください。")
    print("ボタンと手先の位置・向きがこの経路に合わせて実機で調整済みであることが必要です。")
    if input("腕の制御を開始する場合だけ YES と入力: ").strip() != "YES":
        print("中止しました。歩行モードは維持されています。")
        return

    try:
        # 公式 arm_sdk 例のように、現在姿勢を保ったまま制御重みを上げる。
        for step in range(101):
            arm.publish(start, step / 100.0)
            time.sleep(0.02)
        previous = start
        bad_tracking_since = None
        for phase in PHASES:
            duration = float(plan["durations"][phase])
            print(f"[{phase}] 開始", flush=True)
            next_tick = time.monotonic()
            for target in interpolate(previous, poses[phase], duration, CONTROL_HZ):
                observed = arm.current_arm(timeout=STATE_TIMEOUT)[7:]
                error = max(abs(a - b) for a, b in zip(observed, arm._last_right))
                if error > TRACKING_LIMIT:
                    bad_tracking_since = bad_tracking_since or time.monotonic()
                    if time.monotonic() - bad_tracking_since > 0.5:
                        raise RuntimeError(f"腕の追従誤差が {error:.2f} rad。押下を中止します")
                else:
                    bad_tracking_since = None
                arm.publish(target, 1.0)
                next_tick += 1.0 / CONTROL_HZ
                time.sleep(max(0.0, next_tick - time.monotonic()))
            previous = poses[phase]
    finally:
        arm.release()
    print("腕を戻して制御重みを解除しました。ボタンの点灯や反応を人が確認してください。")


def main() -> int:
    if "--align" in sys.argv[1:]:
        from align_button import main as align_main
        return align_main([arg for arg in sys.argv[1:] if arg != "--align"])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, help="MuJoCo で成功した経路 JSON")
    parser.add_argument("--network-interface", default="enp3s0")
    parser.add_argument("--execute", action="store_true", help="実機へ指令を送る")
    parser.add_argument("--calibrated", action="store_true",
                        help="手先・ボタン位置を実機で合わせたことを明示")
    args = parser.parse_args()
    try:
        plan = load_plan(args.plan)
        for phase in PHASES:
            print(f"{phase:8s} {plan['durations'][phase]:4.1f}s  "
                  + " ".join(f"{q:+.2f}" for q in plan["poses"][phase]))
        if "ik_validation" in plan:
            for phase in ("approach", "contact", "press"):
                result = plan["ik_validation"][phase]
                print(f"IK {phase}: 位置誤差={float(result['position_error_m'])*1000:.2f} mm, "
                      f"方向誤差={float(result['axis_error_deg']):.2f} 度, "
                      f"関節余裕={float(result['min_joint_margin_rad']):.3f} rad")
        print("シミュレーションの押下量: "
              f"{plan['sim_result']['max_stroke_m']*1000:.1f} mm "
              f"({'固定台' if plan['sim_result'].get('base_fixed') else '自由立位'})")
        if not args.execute:
            print("計画表示のみ。DDS 指令は送信していません。")
            return 0
        if not args.calibrated:
            raise ValueError("実機で位置合わせをした後に --calibrated を指定してください")
        execute(plan, args.network_interface)
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("中断しました。腕の制御重みを解除します。", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
