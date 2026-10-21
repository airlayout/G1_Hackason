#!/usr/bin/env python3
"""右肩を少し上げて戻し、実測の動きで接続を確認する。既定は受信・計画表示のみ。"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trajectory import validate_arm_q
from push_button import ArmSdk


def validate_settings(angle_deg, duration):
    if (not math.isfinite(angle_deg) or not 2 <= angle_deg <= 10
            or not math.isfinite(duration) or not 2 <= duration <= 5):
        raise ValueError("上げる角度は2～10度、片道の移動時間は2～5秒が必要です")


def check_state(configuration, now, baseline=None):
    joints, rpy, stamp, leg_speed, gyro = configuration
    if (len(joints) != 29 or len(rpy) != 3
            or not all(math.isfinite(v) for v in (*joints, *rpy, stamp, leg_speed, gyro))):
        raise ValueError("29軸の関節角・IMU・時刻の状態データが不正です")
    if not 0 <= now-stamp <= 0.25:
        raise RuntimeError(f"実機状態が古いか時刻が不正です (age={now-stamp:.3f}s)")
    validate_arm_q(list(joints[22:29]))
    if leg_speed > 0.15 or gyro > 0.10 or max(abs(v) for v in rpy[:2]) > 0.15:
        raise RuntimeError(f"脚または骨盤が静止・直立していません "
                           f"(脚速度={leg_speed:.3f}rad/s, 角速度={gyro:.3f}rad/s, "
                           f"roll={rpy[0]:.3f}rad, pitch={rpy[1]:.3f}rad)")
    if baseline is not None:
        q0, rpy0 = baseline[0], baseline[1]
        if max(abs(a-b) for a, b in zip(joints[12:15], q0[12:15])) > 0.025:
            raise RuntimeError("確認動作中に腰姿勢が変化しました")
        if max(abs(math.atan2(math.sin(a-b), math.cos(a-b)))
               for a, b in zip(rpy[:2], rpy0[:2])) > math.radians(2):
            raise RuntimeError("確認動作中に骨盤の傾きが変化しました")
        if max(abs(joints[i]-q0[i]) for i in range(15, 29) if i != 22) > math.radians(4):
            raise RuntimeError("右肩以外の腕関節が4度を超えて動きました")
    return tuple(joints)


def make_plan(configuration, angle_deg, duration, now):
    validate_settings(angle_deg, duration)
    joints = check_state(configuration, now)
    # 肩が下がった姿勢から前方へ上げる小動作に限定する。
    if not -0.5 <= joints[22] <= 0.8:
        raise ValueError("右肩が確認動作の開始範囲外です。腕を下ろした姿勢で使ってください")
    start = validate_arm_q(list(joints[22:29]))
    goal = validate_arm_q([start[0]-math.radians(angle_deg), *start[1:]])
    return {"joint": 22, "joint_name": "right_shoulder_pitch_joint", "angle_deg": angle_deg,
            "raise_duration_s": duration, "hold_s": 0.5, "return_duration_s": duration,
            "start_right_arm_rad": list(start), "raised_right_arm_rad": list(goal),
            "peak_command_speed_rad_s": 1.5*math.radians(angle_deg)/duration,
            "peak_command_acceleration_rad_s2": 6*math.radians(angle_deg)/duration**2}


def validate_connection(info):
    if info["mode_machine"] != 5 or info["motor_count"] < 29:
        raise ValueError("29DoF G1 (mode_machine=5) の状態が必要です")
    if info["fsm_id"] != 501:
        raise RuntimeError(f"安定立位FSM 501が必要です (現在: {info['fsm_id']})")


class ArmConnectionCheck:
    def __init__(self, arm, angle_deg=5., duration=2., *, now=time.monotonic, wait=time.sleep):
        validate_settings(angle_deg, duration)
        self.arm, self.angle_deg, self.duration = arm, angle_deg, duration
        self.now, self.wait = now, wait
        self.target = None
        self.weight = 0.
        self.baseline = None
        self.report = {"schema": "g1-arm-connection-check-v1", "completed": False,
                       "movement_confirmed": False, "return_confirmed": False,
                       "release_sent": False, "phases": []}

    def state(self):
        configuration = self.arm.current_configuration()
        self.report["last_observation"] = {"motor_q_rad": list(configuration[0]),
                                           "imu_rpy_rad": list(configuration[1]),
                                           "received_at_s": configuration[2],
                                           "leg_speed_rad_s": configuration[3], "gyro_rad_s": configuration[4]}
        joints = check_state(configuration, self.now(), self.baseline)
        if self.target is not None and max(abs(a-b) for a, b in zip(joints[22:29], self.target)) > 0.12:
            raise RuntimeError("腕の追従誤差が0.12radを超えました")
        if self.baseline is not None and self.baseline[0][22]-joints[22] > math.radians(self.angle_deg+2):
            raise RuntimeError("右肩の実測移動量が指令角度＋2度を超えました")
        return configuration, joints

    def phase(self, name, duration, start, goal, w0=1., w1=1., *, monitor=True):
        self.report["phase"] = name
        self.report["phases"].append({"phase": name, "time_s": self.now()})
        started = self.now()
        steps = round(duration*50)
        for index in range(steps+1):
            deadline = started+index/50
            self.wait(max(0., deadline-self.now()))
            if monitor:
                if self.now()-deadline > 0.10:
                    raise RuntimeError("腕指令の更新が100msを超えて遅れました")
                self.state()
            alpha = index/steps
            blend = alpha*alpha*(3-2*alpha)
            target = validate_arm_q([a+(b-a)*blend for a, b in zip(start, goal)])
            weight = w0+(w1-w0)*blend
            self.arm.publish(target, weight, require_fresh_state=monitor)
            self.target, self.weight = target, weight

    def execute(self):
        started = self.now()
        try:
            info = self.arm.connection_info()
            self.report["connection"] = info
            validate_connection(info)
            self.arm.stop_walking()
            baseline, joints = self.state()
            make_plan(baseline, self.angle_deg, self.duration, self.now())
            # 腕が既に動いていれば有効化しない。保持姿勢は最後の新しい状態から取る。
            for _ in range(10):
                self.wait(.02)
                state, current = self.state()
                check_state(state, self.now(), baseline)
                if max(abs(a-b) for a, b in zip(joints[15:29], current[15:29])) > 0.02:
                    raise RuntimeError("腕が静止していません")
            self.baseline = state
            plan = make_plan(state, self.angle_deg, self.duration, self.now())
            self.report["plan"] = plan
            start, goal = plan["start_right_arm_rad"], plan["raised_right_arm_rad"]
            self.target = tuple(start)
            self.arm.set_hold(tuple(current[15:29]))
            self.phase("enable", 2., start, start, 0., 1.)
            _, before = self.state()
            self.phase("raise", self.duration, start, goal)
            self.phase("hold", .5, goal, goal)
            _, raised = self.state()
            observed = math.degrees(before[22]-raised[22])
            self.report["observed_raise_deg"] = observed
            self.report["movement_confirmed"] = max(.5, self.angle_deg*.4) <= observed <= self.angle_deg+2
            self.phase("return", self.duration, goal, start)
            self.phase("settle", .5, start, start)
            _, returned = self.state()
            error = max(abs(a-b) for a, b in zip(returned[15:29], current[15:29]))
            self.report["return_max_error_deg"] = math.degrees(error)
            self.report["return_confirmed"] = error <= math.radians(2)
            if not self.report["movement_confirmed"]:
                self.report["error"] = "右肩の実測から上昇を確認できません。自動では再試行しません"
            elif not self.report["return_confirmed"]:
                self.report["error"] = "開始時の腕姿勢への復帰誤差が2度を超えました"
        except KeyboardInterrupt:
            self.report["interrupted"] = True
            self.report["error"] = "操作者による中断"
        except (ValueError, RuntimeError, OSError, KeyError, TypeError) as exc:
            self.report["error"] = str(exc)
        finally:
            if self.weight > 0:
                try:
                    # 姿勢や受信が異常でも、最後の目標を保って重み解除だけを試みる。
                    self.phase("release", 2., self.target, self.target, self.weight, 0., monitor=False)
                    self.report["release_sent"] = True
                except (ValueError, RuntimeError, OSError) as exc:
                    self.report["release_error"] = str(exc)
                    try:
                        self.arm.release()
                    except (ValueError, RuntimeError, OSError) as fallback:
                        self.report["release_fallback_error"] = str(fallback)
            self.report["completed"] = bool(self.report["movement_confirmed"] and self.report["return_confirmed"]
                                             and self.report["release_sent"] and "error" not in self.report
                                             and "release_error" not in self.report)
            self.report["elapsed_time_s"] = self.now()-started
        return self.report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network-interface", default="enp3s0")
    parser.add_argument("--arm-socket", help="SDKを別のPython環境で動かす場合の既存ブリッジ")
    parser.add_argument("--state-json", help="保存状態でのオフライン計画表示。実行には使えない")
    parser.add_argument("--angle-deg", type=float, default=5.)
    parser.add_argument("--duration", type=float, default=2., help="上げる／戻す各片道の時間[秒]")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", default="_local/button_press/arm_connection_check.json")
    args = parser.parse_args(argv)
    report = {"schema": "g1-arm-connection-check-v1", "completed": False, "connection_verified": False}
    arm = None
    code = 2
    old_sigterm = None
    try:
        validate_settings(args.angle_deg, args.duration)
        if args.execute and args.state_json:
            raise ValueError("--state-jsonはオフライン計画専用です。実行時は実機の現在状態を使います")
        if args.state_json:
            payload = json.loads(Path(args.state_json).read_text())
            configuration = (payload["motor_q_rad"], payload["imu_rpy_rad"], time.monotonic(), 0., 0.)
        else:
            if args.arm_socket:
                from arm_bridge import ArmProxy
                arm = ArmProxy(args.arm_socket)
            else:
                arm = ArmSdk(args.network_interface)
            info = arm.connection_info()
            validate_connection(info)
            configuration = arm.current_configuration()
            report["connection"] = info
            report["connection_verified"] = True
        report["plan"] = make_plan(configuration, args.angle_deg, args.duration, time.monotonic())
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        if not args.execute:
            print("接続・計画の確認のみ。腕への指令は送っていません。" if arm else "保存状態での計画表示のみ。実機には接続していません。")
            code = 0
        elif input("停止・直立と腕の周囲の空きを確認し、動かす場合だけ YES と入力: ").strip() != "YES":
            report["cancelled"] = True
            code = 0
        else:
            def interrupt(_signal, _frame):
                raise KeyboardInterrupt

            old_sigterm = signal.signal(signal.SIGTERM, interrupt)
            checker = ArmConnectionCheck(arm, args.angle_deg, args.duration)
            report = checker.execute()
            report["connection_verified"] = "connection" in report
            code = 130 if report.get("interrupted") else (0 if report["completed"] else 1)
    except KeyboardInterrupt:
        report["error"] = "操作者による中断"
        code = 130
    except (ValueError, RuntimeError, OSError, KeyError, TypeError, ImportError) as exc:
        report["error"] = str(exc)
    finally:
        if old_sigterm is not None:
            signal.signal(signal.SIGTERM, old_sigterm)
        if arm is not None:
            try:
                arm.close()
            except (ValueError, RuntimeError, OSError) as exc:
                report["close_error"] = str(exc)
                report["completed"] = False
                code = 1
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
