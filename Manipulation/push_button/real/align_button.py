#!/usr/bin/env python3
"""停止したG1でRGB-D再計測と右腕の位置補正を行ってから押す。既定は計画表示。"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from alignment import (AlignmentController, CorrectionCadence, CorrectionTiming,
                       ContinuousNormalController, JointCommandFilter, NormalMotionController,
                       SingleCorrectionTiming, add_correction_arguments,
                       align_fixed_target, normal_completion_tolerance, press_compensation_allowance,
                       press_overtravel_allowance, run_continuous_normal)
from kinematics import ArmKinematics, load_robot
from trajectory import interpolate, validate_arm_q
from localize_button import RgbdLocalizer
from push_button import ArmSdk, CONTROL_HZ, STATE_TIMEOUT, TRACKING_LIMIT, check_speed


def read_calibration(path):
    calibration = json.loads(Path(path).read_text())
    if calibration.get("schema") != "g1-button-calibration-v1":
        raise ValueError("g1-button-calibration-v1 の実測校正ファイルが必要です")
    if calibration["camera_body"] not in ("pelvis", "waist_yaw_link", "waist_roll_link", "torso_link"):
        raise ValueError("骨盤または胴体に固定したカメラのbody名が必要です")
    transform = np.asarray(calibration["T_base_optical"], dtype=float)
    if (transform.shape != (4, 4) or not np.all(np.isfinite(transform))
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=0.002)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=0.002)
            or not np.allclose(transform[3], [0, 0, 0, 1])):
        raise ValueError("カメラ校正行列が不正です")
    for key in ("reference_waist_q_rad", "reference_imu_rpy_rad"):
        vector = np.asarray(calibration[key], dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"校正時の姿勢 {key} が必要です")
    tip_radius = float(calibration["tip_radius_m"])
    button_radius = float(calibration["button_radius_m"])
    uncertainty = float(calibration["position_uncertainty_m"])
    if (not np.all(np.isfinite((tip_radius, button_radius, uncertainty)))
            or not 0 < tip_radius < 0.04 or not 0 < button_radius <= 0.06
            or not 0 <= uncertainty <= 0.02
            or button_radius - tip_radius < uncertainty + 0.003):
        raise ValueError("ボタンと手先の寸法に対して位置の不確かさが大きすぎます")
    return calibration, transform


def validate_stationary_pose(configuration, calibration):
    joints, rpy, received_at, leg_speed, gyro = configuration
    if time.monotonic() - received_at > STATE_TIMEOUT:
        raise RuntimeError("実機状態が古すぎます")
    if leg_speed > 0.15 or gyro > 0.10:
        raise RuntimeError("脚または骨盤が動いています。押下を中止します")
    waist = np.array(joints[12:15])
    if np.max(np.abs(waist - calibration["reference_waist_q_rad"])) > 0.025:
        raise RuntimeError("腰姿勢が校正時から変化しました。再校正が必要です")
    angle_error = np.arctan2(np.sin(np.array(rpy[:2]) - calibration["reference_imu_rpy_rad"][:2]),
                             np.cos(np.array(rpy[:2]) - calibration["reference_imu_rpy_rad"][:2]))
    if np.max(np.abs(angle_error)) > np.deg2rad(2):
        raise RuntimeError("骨盤の傾きが校正時から変化しました")


class ArmMotion:
    """画像推論に依存せず50Hzで保持・補間指令を送り、状態を監視する。"""

    def __init__(self, arm, calibration):
        self.arm, self.calibration = arm, calibration
        current = arm.current_arm()
        arm.set_hold(current)
        self.home = tuple(current[7:])
        self.target = self.home
        self.weight = 0.0
        self.error = None
        self.release_error = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        bad_since = None
        next_tick = time.monotonic()
        try:
            while not self.stop.is_set():
                configuration = self.arm.current_configuration()
                validate_stationary_pose(configuration, self.calibration)
                with self.lock:
                    target, weight = self.target, self.weight
                observed = configuration[0][22:29]
                tracking = max(abs(a-b) for a, b in zip(observed, target))
                if tracking > TRACKING_LIMIT:
                    bad_since = bad_since or time.monotonic()
                    if time.monotonic() - bad_since > 0.5:
                        raise RuntimeError("腕の追従誤差が継続しています")
                else:
                    bad_since = None
                self.arm.publish(target, weight)
                next_tick += 1 / CONTROL_HZ
                self.stop.wait(max(0, next_tick - time.monotonic()))
        except Exception as exc:
            self.error = exc
            self.stop.set()
        finally:
            try:
                self.arm.release()
            except Exception as exc:
                self.release_error = str(exc)

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"腕の状態監視で中止: {self.error}") from self.error

    def wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check()
            time.sleep(min(0.02, max(0, deadline - time.monotonic())))

    def enable(self):
        self.thread.start()
        for step in range(101):
            self.check()
            with self.lock:
                self.weight = step / 100
            self.wait(0.02)

    def move(self, goal, duration, settle_s=0.1):
        goal = validate_arm_q(np.asarray(goal).tolist())
        with self.lock:
            start = self.target
        duration = max(duration, 1.5 * max(abs(a-b) for a, b in zip(start, goal)) / 0.8)
        check_speed(start, goal, duration)
        for target in interpolate(start, goal, duration, CONTROL_HZ):
            self.check()
            with self.lock:
                self.target = target
            self.wait(1 / CONTROL_HZ)
        self.wait(settle_s)

    def current_target(self):
        with self.lock:
            return np.array(self.target)

    def stream(self, goal):
        """検査済みの50Hz指令を更新する。区間補間や静止待ちは入れない。"""
        self.check()
        goal = validate_arm_q(np.asarray(goal).tolist())
        with self.lock:
            self.target = goal

    def close(self):
        self.stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=6.5)
        try:
            self.arm.release()
        except Exception as exc:
            self.release_error = str(exc)


def execute_alignment(arm, camera, kin, calibration, args):
    execution_started = time.monotonic()
    timing = CorrectionTiming.from_args(args)
    single_timing = SingleCorrectionTiming.from_args(args)
    single = getattr(args, "alignment_mode", "single") == "single"
    smooth = single and getattr(args, "normal_motion", "smooth") == "smooth"
    motion = ArmMotion(arm, calibration)
    report = {"schema": "g1-live-button-alignment-v1", "physical_press_success": None,
              "alignment_mode": "single" if single else "iterative",
              "normal_motion": "smooth" if smooth else "stepped",
              "single_correction_timing": vars(single_timing), "measurement_count": 0,
              "correction_timing": vars(timing), "alignment_steps": [],
              "press_completion_tolerance_m": normal_completion_tolerance(args.stroke, single),
              "stroke_m": args.stroke,
              "press_limits": {"overtravel_allowance_m": press_overtravel_allowance(args.stroke),
                               "compensation_allowance_m": press_compensation_allowance(args.stroke)},
              "normal_steps": [], "completed": False}
    controller = AlignmentController(tip_radius=calibration["tip_radius_m"],
                                     clearance=args.clearance)
    last_target = None

    def refresh():
        motion.check()
        configuration = arm.current_configuration()
        validate_stationary_pose(configuration, calibration)
        kin.update_fixed_base(np.array(configuration[0]), configuration[1])

    def move_xyz(xyz, target, duration, allow_contact=False, settle_s=0.1, cadence=None):
        refresh()
        solution = kin.solve(xyz, np.array(target.press_direction))
        kin.validate_path(solution.joint_angles, np.array(target.face_xyz),
                          np.array(target.press_direction), allow_contact)
        started_at = cadence.start() if cadence is not None else time.monotonic()
        motion.move(solution.joint_angles, duration, settle_s)
        return started_at

    def move_normal(face, normal, depth, bias, target, phase, allow_contact):
        radius = calibration["tip_radius_m"]
        refresh()
        start_depth = float((kin.tip_position()-face) @ normal + radius)
        command = face + (start_depth-radius) * normal + bias
        servo = NormalMotionController(face, normal, radius, depth, command, single=single)
        cadence = None if single else CorrectionCadence(timing, time.monotonic, motion.wait)
        phase_started = time.monotonic()
        while True:
            if cadence is not None:
                cadence.wait_for_next()
            elif time.monotonic() - phase_started >= single_timing.phase_timeout_s:
                raise ValueError("法線方向の追従が制限時間内に収束しません")
            refresh()
            goal = servo.next_goal(kin.tip_position())
            if goal is None:
                break
            started_at = move_xyz(goal, target,
                                  single_timing.feedback_move_s if single else timing.move_s,
                                  allow_contact,
                                  single_timing.feedback_settle_s if single else timing.settle_s, cadence)
            report["normal_steps"].append({"phase": phase, "motion_started_s": started_at,
                                           "tip_goal_xyz_m": goal.tolist()})

    def move_smooth(face, normal, bias):
        refresh()
        radius = calibration["tip_radius_m"]
        start_depth = float((kin.tip_position() - face) @ normal + radius)
        servo = ContinuousNormalController(face, normal, radius, args.stroke,
                                            face + (start_depth-radius)*normal + bias)
        joint_filter = JointCommandFilter(motion.current_target(), servo.settings)
        report["smooth_normal_settings"] = vars(servo.settings)

        def read_tip():
            refresh()
            return kin.tip_position()

        def send_goal(xyz, dt, phase):
            solution = kin.solve(xyz, normal, position_tolerance_m=0.00002)
            goal = joint_filter.next_goal(solution.joint_angles, dt)
            kin.validate_path(goal, face, normal, phase == "press")
            motion.stream(goal)
            return goal

        report["continuous_normal_result"] = run_continuous_normal(
            servo, time.monotonic, motion.wait, read_tip, send_goal,
            single_timing.phase_timeout_s, report["normal_steps"].append)

    try:
        motion.enable()
        observation = camera.observe()
        report["measurement_count"] += 1
        # 最初の目標も鮮度を検査する。大きな移動の前に古い画像を使わない。
        if not 0 <= time.monotonic() - observation.captured_at <= controller.max_age_s:
            raise ValueError("初回のボタン計測が古すぎます")
        last_target = observation.target
        face = np.array(last_target.face_xyz)
        normal = np.array(last_target.press_direction)
        wait_point = face - (calibration["tip_radius_m"] + args.clearance) * normal
        if single:
            move_xyz(face - (calibration["tip_radius_m"] + 0.09)*normal, last_target, 4.0)
        move_xyz(wait_point, last_target, 1.0 if single else 4.0,
                 settle_s=single_timing.capture_settle_s if single else timing.settle_s)
        if single:
            observation = camera.observe()
            report["measurement_count"] += 1
            last_target = observation.target

            def read_tip():
                refresh()
                return kin.tip_position()

            controller, result = align_fixed_target(
                observation, calibration["tip_radius_m"], args.clearance,
                single_timing, time.monotonic, motion.wait, read_tip,
                lambda goal, duration, settle: move_xyz(goal, last_target, duration, settle_s=settle))
            report["alignment_steps"].append({"sequence": observation.sequence,
                                              "captured_at_s": observation.captured_at,
                                              "target_xyz_m": list(last_target.face_xyz), **result})
            report["final_alignment_error_m"] = result["final_error_m"]
        cadence = CorrectionCadence(timing, time.monotonic, motion.wait) if not single else None
        while not single:
            # 間隔待ちは撮影より先。2秒前の計測を次の指令に使わない。
            cadence.wait_for_next()
            observation = camera.observe()
            report["measurement_count"] += 1
            refresh()
            step = controller.next_step(observation, kin.tip_position(), time.monotonic())
            last_target = observation.target
            entry = {"time_s": time.monotonic(), "captured_at_s": observation.captured_at,
                     "error_m": step.error_m, "target_xyz_m": list(last_target.face_xyz),
                     "sequence": observation.sequence, "motion_started_s": None}
            report["alignment_steps"].append(entry)
            print(f"[align] error={step.error_m*1000:.1f} mm lateral={step.lateral_error_m*1000:.1f} mm", flush=True)
            if step.converged:
                cadence.start()
                report["final_alignment_error_m"] = step.error_m
                break
            if step.error_m > controller.tolerance_m:
                entry["motion_started_s"] = move_xyz(
                    step.tip_goal, last_target, timing.move_s,
                    settle_s=timing.settle_s, cadence=cadence)
            else:
                cadence.start()
        # 接触後は再検出による横方向の修正を行わない。押込量を明示的に制限する。
        face = np.array(last_target.face_xyz)
        normal = np.array(last_target.press_direction)
        radius = calibration["tip_radius_m"]
        bias = controller.command_xyz - (face - (radius + args.clearance) * normal)
        bias -= (bias @ normal) * normal
        if smooth:
            move_smooth(face, normal, bias)
        else:
            move_normal(face, normal, -0.003, bias, last_target, "contact", False)
            move_normal(face, normal, args.stroke, bias, last_target, "press", True)
        move_xyz(face - (radius + args.clearance) * normal + bias, last_target, 2.0, True)
        refresh()
        kin.validate_path(np.array(motion.home), face, normal)
        motion.move(motion.home, 4.0)
        report["completed"] = True
        print("位置合わせと押下動作を完了しました。点灯や反応で押下成否を確認してください。")
    except Exception as exc:
        report["error"] = str(exc)
        # 状態が正常で、無接触と確認できる経路だけで後退する。
        if last_target is not None:
            try:
                retreat = np.array(last_target.face_xyz) - (calibration["tip_radius_m"] + args.clearance) * np.array(last_target.press_direction)
                move_xyz(retreat, last_target, 2.0, True)
            except Exception as retreat_error:
                report["retreat_error"] = str(retreat_error)
        raise
    finally:
        motion.close()
        report["elapsed_time_s"] = time.monotonic() - execution_started
        report["remeasurement_count"] = max(0, report["measurement_count"] - 1)
        if motion.release_error is not None:
            report["release_error"] = motion.release_error
            report["completed"] = False
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + "\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Menagerie の29DoF g1.xml")
    parser.add_argument("--weights", required=True)
    parser.add_argument("--base-transform", required=True, help="カメラ・手先・停止姿勢を実測した校正JSON")
    parser.add_argument("--target-pixel", type=int, nargs=2)
    parser.add_argument("--stroke", required=True, type=float, help="実物に合わせた押込量 [m]")
    parser.add_argument("--clearance", type=float, default=0.05)
    parser.add_argument("--network-interface", default="enp3s0")
    parser.add_argument("--arm-socket", help="SDKを別のPython環境で実行する場合のUnixソケット")
    parser.add_argument("--state-json", help="計画表示用。motor_q_rad (29個) を持つ状態JSON")
    parser.add_argument("--output", required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--calibrated", action="store_true")
    add_correction_arguments(parser)
    args = parser.parse_args(argv)
    camera = None
    try:
        timing = CorrectionTiming.from_args(args)
        single_timing = SingleCorrectionTiming.from_args(args)
        if (not np.isfinite(args.stroke) or not 0.0015 <= args.stroke <= 0.015
                or not np.isfinite(args.clearance) or not 0.02 <= args.clearance <= 0.05):
            raise ValueError("strokeは1.5～15mm、clearanceは2～5cmが必要です")
        calibration, transform = read_calibration(args.base_transform)
        model, rest = load_robot(Path(args.model).resolve(), tuple(calibration["tip_offset_m"]),
                              float(calibration["pelvis_height_m"]), float(calibration["tip_radius_m"]))
        kin = ArmKinematics(model)
        reference_q = rest.copy()
        reference_q[12:15] = calibration["reference_waist_q_rad"]
        kin.update_fixed_base(reference_q, calibration["reference_imu_rpy_rad"])
        camera_body = calibration["camera_body"]
        T_body_optical = np.linalg.inv(kin.body_transform(camera_body)) @ transform
        if args.execute and not args.calibrated:
            raise ValueError("実測校正後に --execute --calibrated を指定してください")
        if not args.execute and not args.state_json:
            raise ValueError("計画表示には現在関節角の --state-json が必要です")
        arm = None
        if args.execute:
            if args.arm_socket:
                from arm_bridge import ArmProxy
                arm = ArmProxy(args.arm_socket)
            else:
                arm = ArmSdk(args.network_interface)
            arm.stop_walking()
            for _ in range(6):
                configuration = arm.current_configuration()
                validate_stationary_pose(configuration, calibration)
                time.sleep(0.1)
            print("Navigationの移動指令を止め、機体の静止と手先校正を確認してください。")
            if input("実機の右腕を動かす場合だけ YES と入力: ").strip() != "YES":
                return 0
        if args.execute:
            def current_transform():
                configuration = arm.current_configuration()
                validate_stationary_pose(configuration, calibration)
                if time.monotonic()-configuration[2] > 0.10:
                    raise RuntimeError("カメラ計測に対応する実機状態が古すぎます")
                kin.update_fixed_base(np.array(configuration[0]), configuration[1])
                return kin.body_transform(camera_body) @ T_body_optical
        else:
            state = json.loads(Path(args.state_json).read_text())
            validate_stationary_pose((state["motor_q_rad"], state["imu_rpy_rad"],
                                      time.monotonic(), 0, 0), calibration)
            kin.update_fixed_base(np.array(state["motor_q_rad"]), state["imu_rpy_rad"])
            current_transform = kin.body_transform(camera_body) @ T_body_optical
        camera = RgbdLocalizer(args.weights, current_transform,
                               tuple(args.target_pixel) if args.target_pixel else None)
        if args.execute:
            report = execute_alignment(arm, camera, kin, calibration, args)
            if not report["completed"]:
                raise RuntimeError(f"制御重みの解除に失敗: {report.get('release_error')}")
        else:
            observation = camera.observe()
            if time.monotonic() - observation.captured_at > 0.5:
                raise ValueError("ボタン計測が古すぎます")
            target = observation.target
            face, normal = np.array(target.face_xyz), np.array(target.press_direction)
            radius = calibration["tip_radius_m"]
            poses = {}
            phases = [("transit", -0.09)] if args.alignment_mode == "single" else []
            phases += [("approach", -args.clearance), ("contact", -0.003), ("press", args.stroke)]
            for phase, distance in phases:
                goal = kin.solve(face + (distance-radius)*normal, normal).joint_angles
                kin.validate_path(goal, face, normal, phase == "press")
                poses[phase] = goal.tolist()
                qpos = kin.data.qpos.copy()
                qpos[kin.qadr] = goal
                kin.update(qpos)
            report = {"schema": "g1-live-button-preview-v1", "poses": poses,
                      "target_xyz_m": list(target.face_xyz), "physical_press_success": None,
                      "alignment_mode": args.alignment_mode,
                      "normal_motion": args.normal_motion if args.alignment_mode == "single" else "stepped",
                      "requires_mid_approach_remeasurement": args.alignment_mode == "single",
                      "single_correction_timing": vars(single_timing),
                      "correction_timing": vars(timing)}
            output = Path(args.output)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps(report, indent=2))
            print("計画表示のみ。DDS指令は送信していません。")
        return 0
    except (ValueError, RuntimeError, OSError, KeyError, TypeError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        if camera is not None:
            camera.close()
        if args.execute and args.arm_socket and "arm" in locals() and arm is not None:
            try:
                arm.close()
            except (OSError, RuntimeError) as exc:
                print(f"[bridge close] {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
