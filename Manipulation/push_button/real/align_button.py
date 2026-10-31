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
from alignment import (AlignmentController, AlignmentNotSettled, CorrectionCadence, CorrectionTiming,
                       ContinuousNormalController, JointCommandFilter, NormalMotionController,
                       SingleCorrectionTiming, add_correction_arguments,
                       align_fixed_target, normal_completion_tolerance, press_compensation_allowance,
                       press_overtravel_allowance, run_continuous_normal)
from kinematics import ArmKinematics, load_robot
from trajectory import interpolate, validate_arm_q
from button_feedback import LampCheck, lamp_level
from return_path import ApproachPath, validate_return_path
from localize_button import RgbdLocalizer
from j1gen_camera import CalibratedHeadCamera, J1CameraRuntime, add_camera_arguments
from push_button import ArmSdk, CONTROL_HZ, STATE_TIMEOUT, TRACKING_LIMIT, check_speed
from button_control import STAGED_CLEARANCES_M, preview_button_path, solve_checked, validate_fk_margin
from trial_policy import ButtonTrialPolicy, TrialPause, ObservationUnavailable, add_trial_arguments, resolve_trial_arguments


def read_calibration(path, *, require_button=True):
    calibration = json.loads(Path(path).read_text())
    schema = calibration.get("schema")
    if schema not in ("g1-button-calibration-v1", "g1-button-calibration-v2"):
        raise ValueError("g1-button-calibration-v1/v2 の実測校正ファイルが必要です")
    if schema == "g1-button-calibration-v2":
        calibration.setdefault("camera_body", "torso_link")
    if calibration["camera_body"] not in ("pelvis", "waist_yaw_link", "waist_roll_link", "torso_link"):
        raise ValueError("骨盤または胴体に固定したカメラのbody名が必要です")
    transform = None
    if schema == "g1-button-calibration-v1":
        transform = np.asarray(calibration["T_base_optical"], dtype=float)
        if (transform.shape != (4, 4) or not np.all(np.isfinite(transform))
                or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=0.002)
                or not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=0.002)
                or not np.allclose(transform[3], [0, 0, 0, 1])):
            raise ValueError("カメラ校正行列が不正です")
    else:
        for key in ("camera_offset_m", "camera_rpy_offset_rad"):
            vector = np.asarray(calibration[key], dtype=float)
            if vector.shape != (3,) or not np.all(np.isfinite(vector)):
                raise ValueError(f"カメラ取付補正 {key} が不正です")
    for key in ("reference_waist_q_rad", "reference_imu_rpy_rad"):
        vector = np.asarray(calibration[key], dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"校正時の姿勢 {key} が必要です")
    tip_radius = float(calibration["tip_radius_m"])
    button_radius = float(calibration["button_radius_m"]) if require_button else 0.06
    uncertainty = float(calibration["position_uncertainty_m"])
    if (not np.all(np.isfinite((tip_radius, button_radius, uncertainty)))
            or not 0 < tip_radius < 0.04 or not 0 < button_radius <= 0.06
            or not 0 <= uncertainty <= 0.02
            or (require_button and button_radius - tip_radius < uncertainty + 0.003)):
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

    def __init__(self, arm, calibration, *, gravity=None, state_sink=None):
        self.arm, self.calibration = arm, calibration
        self.gravity = gravity
        self.state_sink = state_sink
        current = arm.current_arm()
        arm.set_hold(current)
        self.home = tuple(current[7:])
        self.target = self.home
        self.target_velocity = np.zeros(7)
        self.weight = 0.0
        self.error = None
        self.release_error = None
        self.approach = None
        self.record_approach = False
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        bad_since = None
        next_tick = time.monotonic()
        try:
            while not self.stop.is_set():
                configuration = self.arm.current_configuration()
                if self.state_sink is not None:
                    self.state_sink(configuration)
                validate_stationary_pose(configuration, self.calibration)
                with self.lock:
                    target, weight = self.target, self.weight
                    velocity = self.target_velocity.copy()
                observed = configuration[0][22:29]
                tracking = max(abs(a-b) for a, b in zip(observed, target))
                if tracking > TRACKING_LIMIT:
                    bad_since = bad_since or time.monotonic()
                    if time.monotonic() - bad_since > 0.5:
                        raise RuntimeError("腕の追従誤差が継続しています")
                else:
                    bad_since = None
                self.publish(configuration, target, weight, velocity=velocity)
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

    def publish(self, configuration, target, weight, *, velocity=None):
        if self.gravity is None:
            self.arm.publish(target, weight)
        else:
            if velocity is None:
                with self.lock:
                    velocity=self.target_velocity.copy()
            tau = tuple(float(value)*weight for value in self.gravity(configuration,velocity=velocity))
            self.arm.publish(target, weight, right_tau=tau)

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
        previous=np.array(start)
        for target in interpolate(start, goal, duration, CONTROL_HZ):
            self.check()
            with self.lock:
                self.target = target
                self.target_velocity=(np.array(target)-previous)*CONTROL_HZ
            previous=np.array(target)
            if self.record_approach:
                self.approach.append(target, 1/CONTROL_HZ)
            self.wait(1 / CONTROL_HZ)
        with self.lock:
            self.target_velocity=np.zeros(7)
        self.wait(settle_s)

    def start_approach(self):
        self.approach = ApproachPath(self.current_target())
        self.record_approach = True

    def freeze_approach(self):
        self.approach.freeze()
        self.record_approach = False

    def replay(self, targets, validate):
        """記録した隣接区間だけをたどる。遅れを取り戻す高速送信はしない。"""
        last_tick = time.monotonic()
        for target in targets:
            self.check()
            validate(target)
            now = time.monotonic()
            if now-last_tick > 0.1:
                raise RuntimeError("腕の逆再生の更新が100ms以上途絶えました")
            self.stream(target)
            last_tick = now
            self.wait(1/CONTROL_HZ)

    def current_target(self):
        with self.lock:
            return np.array(self.target)

    def stream(self, goal, *, velocity=None):
        """検査済みの50Hz指令を更新する。区間補間や静止待ちは入れない。"""
        self.check()
        goal = validate_arm_q(np.asarray(goal).tolist())
        with self.lock:
            self.target_velocity=(np.array(goal)-np.array(self.target))*CONTROL_HZ if velocity is None else np.array(velocity)
            self.target = goal
        if self.record_approach:
            self.approach.append(goal, 1/CONTROL_HZ)

    def close(self):
        self.stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=6.5)
        try:
            self.arm.release()
        except Exception as exc:
            self.release_error = str(exc)


def execute_alignment(arm, camera, kin, calibration, args, *, motion=None, policy=None):
    execution_started = time.monotonic()
    timing = CorrectionTiming.from_args(args)
    single_timing = SingleCorrectionTiming.from_args(args)
    single = getattr(args, "alignment_mode", "single") == "single"
    smooth = single and getattr(args, "normal_motion", "smooth") == "smooth"
    policy = policy or ButtonTrialPolicy()
    motion = motion or ArmMotion(arm, calibration)
    staged = policy.approach_mode == 'staged'
    diagnostic = getattr(args, 'stop_before_contact', False)
    margin = (validate_fk_margin(calibration, policy) if 'button_radius_m' in calibration else float('inf'))
    report = {"schema": "g1-live-button-alignment-v1", "physical_press_success": None,
              "alignment_mode": "single" if single else "iterative",
              "normal_motion": "smooth" if smooth else "stepped",
              "single_correction_timing": vars(single_timing), "measurement_count": 0,
              "correction_timing": vars(timing), "alignment_steps": [],
              "press_completion_tolerance_m": normal_completion_tolerance(args.stroke, single),
              "stroke_m": args.stroke,
              "press_limits": {"overtravel_allowance_m": press_overtravel_allowance(args.stroke),
                               "compensation_allowance_m": press_compensation_allowance(args.stroke)},
              "normal_steps": [], "completed": False,
              "lamp_confirmed": None, "retry_count": 0, "arm_returned": False,
              "lamp_checks": [], "lamp_settings": {"min_rise": getattr(args, "lamp_min_rise", 30.0),
                  "ellipse_scale": 1.15, "percentile": 90, "frames": 3, "timeout_s": 2.0}}
    report.update(task='button', profile=policy.profile, motion_completed=False, home_completed=False,
                  release_completed=False, diagnostic_completed=False, stop_before_contact=diagnostic,
                  approach_mode=policy.approach_mode, approach_clearances_m=list(STAGED_CLEARANCES_M) if staged else [args.clearance],
                  approach_runs=[], approach_steps=[], approach_remeasurement_count=0, approach_extra_retry_count=0,
                  fk_tolerance_m=policy.fk_tolerance_m, fk_abort_m=policy.fk_abort_m, ik_searches=[])
    controller = AlignmentController(tip_radius=calibration["tip_radius_m"],
                                     clearance=args.clearance, tolerance_m=policy.fk_tolerance_m)
    last_target = None
    return_started = False
    enabled = pushing = False

    def snapshot(frame, name, row):
        # 画像の保存失敗は、腕の復帰や確認結果を妨げない。
        try:
            import cv2

            folder = Path(args.output).with_suffix("").with_name(Path(args.output).stem+"_lamp")
            folder.mkdir(parents=True, exist_ok=True)
            raw = folder / f"{name}.png"
            annotated = folder / f"{name}_annotated.jpg"
            image = frame.image.copy()
            x1, y1, x2, y2 = (int(v) for v in frame.bbox)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, f"level={row['level']:.1f} rise={row.get('rise', 0):.1f}",
                        (max(0, x1), max(15, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if not cv2.imwrite(str(raw), frame.image) or not cv2.imwrite(str(annotated), image):
                raise OSError("点灯確認画像を書き込めません")
            row.update(image_path=str(raw), annotated_path=str(annotated))
        except Exception as exc:
            report.setdefault("snapshot_errors", []).append(str(exc))

    def check_lamp(baseline, attempt):
        motion.wait(0.3)
        after = time.monotonic()
        check = LampCheck(baseline, after, report["lamp_settings"]["min_rise"])
        result = {"attempt": attempt, "frames": check.frames, "lamp_confirmed": None}
        report["lamp_checks"].append(result)
        try:
            for frame in camera.lamp_frames(after, timeout_s=2.0):
                motion.check()
                if check.observe(frame, time.monotonic()):
                    snapshot(frame, f"attempt_{attempt}_{len(check.frames)}", check.frames[-1])
                if check.finished:
                    break
        except (ValueError, RuntimeError, OSError) as exc:
            if isinstance(exc, (TrialPause, ObservationUnavailable)):
                raise
            result["reason"] = str(exc)
        motion.check()  # 通信や姿勢の異常は画像の判定不能に丸めない。
        result["lamp_confirmed"] = check.result
        result.setdefault("reason", "点灯" if check.result is True else
                          "増光なし" if check.result is False else "画像不足または3フレームの結果が不一致")
        return check.result

    def refresh():
        motion.check()
        configuration = arm.current_configuration()
        if policy.field:
            motion.monitor.update(configuration)
            motion.check()
        else:
            validate_stationary_pose(configuration, calibration)
        if not 0 <= time.monotonic()-configuration[2] <= .1:
            raise RuntimeError('押下制御の状態が100msより古い、または時刻が不正です')
        kin.update_fixed_base(np.array(configuration[0]), configuration[1])

    def move_xyz(xyz, target, duration, allow_contact=False, settle_s=0.1, cadence=None):
        refresh()
        if policy.field or staged:
            solution = solve_checked(kin, xyz, np.array(target.face_xyz), np.array(target.press_direction),
                                     contact=allow_contact, searches=report['ik_searches'])
        else:
            solution = kin.solve(xyz, np.array(target.press_direction))
            kin.validate_path(solution.joint_angles, np.array(target.face_xyz),
                              np.array(target.press_direction), allow_contact)
        if policy.field or staged:
            delta = float(np.max(np.abs(solution.joint_angles-motion.current_target())))
            duration = max(duration, np.sqrt(6*delta/2.))
        started_at = cadence.start() if cadence is not None else time.monotonic()
        motion.move(solution.joint_angles, duration, settle_s)
        return started_at

    def read_press_tip(face, normal):
        refresh()
        tip = kin.tip_position()
        lateral = tip-face-((tip-face)@normal)*normal
        if np.linalg.norm(lateral) > margin:
            raise ValueError('手先の横ずれがボタンと手先・校正の余裕を超えました')
        return tip

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
            goal = servo.next_goal(read_press_tip(face, normal))
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
            return read_press_tip(face, normal)

        def send_goal(xyz, dt, phase):
            solution = kin.solve(xyz, normal, position_tolerance_m=0.00002)
            goal = joint_filter.next_goal(solution.joint_angles, dt)
            kin.validate_path(goal, face, normal, phase == "press")
            motion.stream(goal)
            return goal

        report["continuous_normal_result"] = run_continuous_normal(
            servo, time.monotonic, motion.wait, read_tip, send_goal,
            single_timing.phase_timeout_s, report["normal_steps"].append)

    def measure():
        observation = camera.observe()
        report["measurement_count"] += 1
        if not 0 <= time.monotonic() - observation.captured_at <= .5:
            raise ValueError('ボタン計測が古すぎます')
        return observation

    def staged_approach(observation):
        nonlocal last_target
        run = {'run': len(report['approach_runs'])+1, 'extra_retry_count': 0,
               'started_at_s': time.monotonic(), 'completed': False}
        report['approach_runs'].append(run)
        try:
            for index, clearance in enumerate(STAGED_CLEARANCES_M):
                extra, reasons = False, []
                while True:
                    row = {'run': run['run'], 'stage': index+1, 'clearance_m': clearance,
                           'is_extra': extra, 'retry_reason': reasons, 'accepted': False,
                           'started_at_s': time.monotonic()}
                    report['approach_steps'].append(row)
                    try:
                        if extra:
                            motion.wait(.6)
                        else:
                            face, normal = np.array(last_target.face_xyz), np.array(last_target.press_direction)
                            move_xyz(face-(calibration['tip_radius_m']+clearance)*normal,
                                     last_target, 4. if index == 0 else 2., settle_s=.6)
                        updated = measure()
                        shift = float(np.linalg.norm(np.array(updated.target.face_xyz)-observation.target.face_xyz))
                        observation, last_target = updated, updated.target
                        report['approach_remeasurement_count'] += 1
                        row.update(sequence=updated.sequence, captured_at_s=updated.captured_at,
                                   frame_ids=list(getattr(camera, 'last_measurement_frame_ids', ())),
                                   face_xyz_m=list(last_target.face_xyz), press_direction=list(last_target.press_direction),
                                   target_shift_m=shift)
                        refresh()
                        preview_button_path(kin, updated, calibration['tip_radius_m'], args, policy,
                                            clearances=STAGED_CLEARANCES_M[index:], searches=report['ik_searches'])
                        face, normal = np.array(last_target.face_xyz), np.array(last_target.press_direction)
                        reasons = ['target_shift'] if shift > .008 else []
                        try:
                            aligned, result = align_fixed_target(updated, calibration['tip_radius_m'], clearance,
                                single_timing, time.monotonic, motion.wait,
                                lambda: (refresh(), kin.tip_position())[1],
                                lambda goal, duration, settle: move_xyz(goal, last_target, duration, settle_s=settle),
                                tolerance_m=policy.fk_tolerance_m, abort_error_m=policy.fk_abort_m)
                            report['alignment_steps'].append({'clearance_m': clearance, 'sequence': updated.sequence, **result})
                        except AlignmentNotSettled:
                            reasons.append('fk_unsettled')
                        refresh()
                        error = kin.tip_position()-(face-(calibration['tip_radius_m']+clearance)*normal)
                        distance = float(np.linalg.norm(error))
                        row.update(fk_error_xyz_m=error.tolist(), fk_error_m=distance, retry_required=reasons)
                        if distance > policy.fk_abort_m:
                            raise ValueError(f'補正後のFK誤差が{policy.fk_abort_m*1000:g}mmを超えました')
                        if distance > policy.fk_tolerance_m and 'fk_unsettled' not in reasons:
                            reasons.append('fk_unsettled')
                        if not reasons:
                            row['accepted'] = True
                            report['final_alignment_error_m'] = distance
                            break
                        if run['extra_retry_count'] >= policy.approach_extra_retries:
                            raise TrialPause('接近の追加確認を使い切りました', code='alignment')
                        run['extra_retry_count'] += 1
                        report['approach_extra_retry_count'] += 1
                        extra = True
                    except Exception as exc:
                        row['error'] = str(exc)
                        raise
                    finally:
                        row['duration_s'] = time.monotonic()-row['started_at_s']
            run['completed'] = True
            return aligned, STAGED_CLEARANCES_M[-1]
        finally:
            run['duration_s'] = time.monotonic()-run['started_at_s']

    def prepare_contact():
        nonlocal last_target, enabled, controller
        observation = measure()
        last_target = observation.target
        if staged or policy.field:
            refresh()
            preview_button_path(kin, observation, calibration['tip_radius_m'], args, policy,
                                searches=report['ik_searches'])
            # 初回の計画計算が遅ければ、古い画像で腕を有効化しない。
            if not 0 <= time.monotonic()-observation.captured_at <= .5:
                raise ObservationUnavailable('初回のボタン計画後に画像が失効しました')
        if not enabled:
            motion.enable()
            enabled = True
            motion.start_approach()
        if staged:
            return staged_approach(observation)
        face = np.array(last_target.face_xyz)
        normal = np.array(last_target.press_direction)
        wait_point = face - (calibration["tip_radius_m"] + args.clearance) * normal
        if single:
            move_xyz(face - (calibration["tip_radius_m"] + 0.09)*normal, last_target, 4.0)
        move_xyz(wait_point, last_target, 1.0 if single else 4.0,
                 settle_s=single_timing.capture_settle_s if single else timing.settle_s)
        if single:
            observation = measure()
            report['approach_remeasurement_count'] += 1
            last_target = observation.target

            def read_tip():
                refresh()
                return kin.tip_position()

            controller, result = align_fixed_target(
                observation, calibration["tip_radius_m"], args.clearance,
                single_timing, time.monotonic, motion.wait, read_tip,
                lambda goal, duration, settle: move_xyz(goal, last_target, duration, settle_s=settle),
                tolerance_m=policy.fk_tolerance_m,
                abort_error_m=policy.fk_abort_m if policy != ButtonTrialPolicy() else None)
            report["alignment_steps"].append({"sequence": observation.sequence,
                                              "captured_at_s": observation.captured_at,
                                              "target_xyz_m": list(last_target.face_xyz), **result})
            report["final_alignment_error_m"] = result["final_error_m"]
        cadence = CorrectionCadence(timing, time.monotonic, motion.wait) if not single else None
        while not single:
            # 間隔待ちは撮影より先。2秒前の計測を次の指令に使わない。
            cadence.wait_for_next()
            observation = measure()
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
        return controller, args.clearance

    def return_home(*, after_push=False):
        nonlocal return_started
        return_started = True
        refresh()
        if after_push and not report.get('retreat_completed'):
            face, normal = np.array(last_target.face_xyz), np.array(last_target.press_direction)
            move_xyz(face-(calibration['tip_radius_m']+args.clearance)*normal, last_target, 2., True)
            report['retreat_completed'] = True
        if enabled and len(motion.approach.samples) > 1:
            if not motion.approach.frozen:
                motion.freeze_approach()
            face, normal = np.array(last_target.face_xyz), np.array(last_target.press_direction)
            targets = motion.approach.reversed_targets(CONTROL_HZ)
            validate_return_path(kin, [motion.current_target(), *targets], face, normal)
            motion.move(targets[0], 1.)

            def validate_return_step(target):
                refresh()
                kin.validate_path(np.asarray(target), face, normal)

            motion.replay(targets, validate_return_step)
            report['return_samples'] = len(targets)
        deadline, settled = time.monotonic()+2., 0
        while time.monotonic() < deadline:
            refresh()
            joints = arm.current_configuration()[0][22:29]
            error = float(np.max(np.abs(np.array(joints)-motion.home)))
            report['return_error_rad'] = error
            settled = settled+1 if error <= np.deg2rad(2) else 0
            if settled >= 3:
                report['arm_returned'] = report['home_completed'] = True
                return
            motion.wait(1/CONTROL_HZ)
        raise RuntimeError('腕が開始姿勢へ戻ったことを確認できません')

    def handle_pause(error, *, allow_resume):
        if not policy.field:
            raise error
        pause = error if isinstance(error, TrialPause) else TrialPause(str(error), code='observation')
        action = args.pause_handler(pause, allow_resume=allow_resume)
        if action == 'resume' and allow_resume:
            camera.reset_measurement_reference()
            return True
        if action == 'return':
            finish_return()
        report['cancelled'] = True
        return False

    def finish_return():
        while True:
            try:
                return_home(after_push=pushing)
                return
            except (TrialPause, ObservationUnavailable) as exc:
                if not policy.field:
                    raise
                pause = exc if isinstance(exc, TrialPause) else TrialPause(str(exc))
                if args.pause_handler(pause, allow_resume=False) != 'return':
                    report['cancelled'] = True
                    return

    try:
        while True:
            try:
                controller, final_clearance = prepare_contact()
                break
            except (TrialPause, ObservationUnavailable) as exc:
                if not handle_pause(exc, allow_resume=True):
                    return report
        if diagnostic:
            finish_return()
            report['diagnostic_completed'] = report['home_completed'] and not report.get('cancelled', False)
            return report
        # 接触後は再検出による横方向の修正を行わない。
        face = np.array(last_target.face_xyz)
        normal = np.array(last_target.press_direction)
        radius = calibration["tip_radius_m"]
        read_press_tip(face, normal)
        bias = controller.command_xyz - (face - (radius + final_clearance) * normal)
        bias -= (bias @ normal) * normal
        baseline_frame = camera.last_lamp_frame
        baseline = lamp_level(baseline_frame.image, baseline_frame.bbox)
        report["lamp_baseline"] = {"level": baseline, "bbox": [float(v) for v in baseline_frame.bbox],
                                   "captured_at_s": float(baseline_frame.captured_at),
                                   "frame_id": int(baseline_frame.frame_id)}
        snapshot(baseline_frame, "before", report["lamp_baseline"])
        motion.freeze_approach()
        pushing = True
        for attempt in (1, 2):
            report['retreat_completed'] = False
            if smooth:
                move_smooth(face, normal, bias)
            else:
                move_normal(face, normal, -0.003, bias, last_target, "contact", False)
                move_normal(face, normal, args.stroke, bias, last_target, "press", True)
            move_xyz(face - (radius + args.clearance) * normal + bias, last_target, 2.0, True)
            report['retreat_completed'] = True
            report["lamp_confirmed"] = check_lamp(baseline, attempt)
            if report["lamp_confirmed"] is not False or attempt == 2:
                break
            report["retry_count"] += 1
        report['motion_completed'] = True
        finish_return()
        report["completed"] = report['home_completed'] and not report.get('cancelled', False)
        report["result_reason"] = report["lamp_checks"][-1]["reason"]
        print(f"押下・復帰を完了しました。点灯確認: {report['result_reason']}")
    except (TrialPause, ObservationUnavailable) as exc:
        handle_pause(exc, allow_resume=False)
    except BaseException as exc:
        exc.button_report = report
        report["error"] = str(exc)
        report['fatal_error'] = True
        # 状態が正常で、無接触と確認できる経路だけで後退する。
        if not policy.field and last_target is not None and not return_started:
            try:
                retreat = np.array(last_target.face_xyz) - (calibration["tip_radius_m"] + args.clearance) * np.array(last_target.press_direction)
                move_xyz(retreat, last_target, 2.0, True)
            except Exception as retreat_error:
                report["retreat_error"] = str(retreat_error)
        raise
    finally:
        active_error = sys.exc_info()[1]
        if active_error is not None:
            active_error.button_report = report
            if not isinstance(active_error, (TrialPause, ObservationUnavailable)):
                report.setdefault('error', str(active_error))
                report['fatal_error'] = True
        try:
            motion.close()
        except Exception as exc:
            motion.release_error = str(exc)
        report["elapsed_time_s"] = time.monotonic() - execution_started
        report["remeasurement_count"] = max(0, report["measurement_count"] - 1)
        report['release_completed'] = motion.release_error is None
        if motion.release_error is not None:
            report["release_error"] = motion.release_error
            report["completed"] = False
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + "\n")
    return report


def build_argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Menagerie の29DoF g1.xml")
    parser.add_argument("--weights", required=True)
    parser.add_argument("--base-transform", required=True, help="カメラ・手先・停止姿勢を実測した校正JSON")
    parser.add_argument("--target-pixel", type=int, nargs=2)
    parser.add_argument("--stroke", required=True, type=float, help="実物に合わせた押込量 [m]")
    parser.add_argument("--clearance", type=float, default=0.05)
    parser.add_argument("--lamp-min-rise", type=float, default=30.0,
                        help="押下前後の明るい部分の増光しきい値（0～255の値）")
    parser.add_argument("--network-interface", default="enp3s0")
    parser.add_argument("--arm-socket", help="SDKを別のPython環境で実行する場合のUnixソケット")
    parser.add_argument("--state-json", help="計画表示用。motor_q_rad (29個) を持つ状態JSON")
    parser.add_argument("--output", required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--calibrated", action="store_true")
    parser.add_argument('--stop-before-contact', action='store_true')
    parser.add_argument('--gravity-scale', type=float, default=0.)
    parser.add_argument('--dynamics-feedforward', action='store_true')
    parser.add_argument('--confidence', type=float, default=.25)
    parser.add_argument('--detector-imgsz', type=int, default=640)
    parser.add_argument('--device', default='auto')
    add_camera_arguments(parser)
    add_correction_arguments(parser)
    add_trial_arguments(parser, task='button')
    return parser


def button_exit_code(report, diagnostic=False):
    if report.get('fatal_error') or report.get('release_error') or report.get('cleanup_errors'):
        return 2
    if report.get('cancelled'):
        return 1
    complete = report.get('completed') and report.get('arm_returned') and report.get('release_completed')
    if diagnostic:
        return 0 if all(report.get(k) for k in ('diagnostic_completed', 'home_completed', 'release_completed')) else 2
    return 0 if complete and report.get('lamp_confirmed') is True else 2


def main(argv=None):
    from dataclasses import asdict
    from pose_history import PoseHistory
    from trial_recording import TrialRecorder
    from button_perception import ButtonPerceptionWorker
    import hashlib
    import subprocess

    class OperatorCancelled(Exception):
        pass

    parser = build_argument_parser()
    try:
        args, policy = resolve_trial_arguments(parser, argv, task='button')
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    output = Path(args.output)
    rgbd_directory = output.with_suffix('').with_name(output.stem+'_rgbd')
    lamp_directory = output.with_suffix('').with_name(output.stem+'_lamp')
    if policy.record_rgbd and any(p.exists() for p in (output, rgbd_directory, lamp_directory)):
        parser.error('過去の試行を残すため、新しい--outputを指定してください')
    camera = arm = motion = monitor = recorder = calibration = None
    report = {'schema': 'g1-live-button-alignment-v1', 'task': 'button', 'completed': False,
              'physical_press_success': None, 'lamp_confirmed': None}
    applied = {'profile': policy.profile, 'trial_policy': asdict(policy), 'arguments': vars(args).copy(),
               'clock': {'epoch_s': time.time(), 'monotonic_s': time.monotonic()}}
    samples, commands = [], []
    history = PoseHistory()
    code = 2
    try:
        timing = CorrectionTiming.from_args(args)
        single_timing = SingleCorrectionTiming.from_args(args)
        if not np.isfinite(args.lamp_min_rise) or not 0 < args.lamp_min_rise <= 255:
            raise ValueError("lamp-min-riseは0より大きく255以下が必要です")
        if (not np.isfinite(args.stroke) or not 0.0015 <= args.stroke <= 0.015
                or not np.isfinite(args.clearance) or not 0.02 <= args.clearance <= 0.05):
            raise ValueError("strokeは1.5～15mm、clearanceは2～5cmが必要です")
        if args.alignment_mode == 'iterative' and policy.approach_mode == 'staged':
            raise ValueError('staged接近とiterative整列は併用できません')
        if (not np.isfinite(args.gravity_scale) or not 0 <= args.gravity_scale <= 1
                or (args.dynamics_feedforward and not args.gravity_scale)
                or not np.isfinite(args.confidence) or not 0 < args.confidence < 1
                or not 320 <= args.detector_imgsz <= 1920 or args.detector_imgsz % 32):
            raise ValueError('重力補償・検出の設定が不正です')
        if args.execute and not args.calibrated:
            raise ValueError("実測校正後に --execute --calibrated を指定してください")
        if not args.execute and not args.state_json:
            raise ValueError("計画表示には現在関節角の --state-json が必要です")
        calibration, _ = read_calibration(args.base_transform)
        if policy.field or policy.approach_mode == 'staged' or policy.fk_tolerance_m > .003:
            validate_fk_margin(calibration, policy)
        if policy.field or policy.record_rgbd:
            sources = sorted(Path(__file__).parent.glob('*.py'))+sorted(Path(__file__).parents[1].glob('*.py'))
            applied['source_sha256'] = {str(p.relative_to(Path(__file__).parents[3])):
                hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
            applied['calibration_sha256'] = hashlib.sha256(Path(args.base_transform).read_bytes()).hexdigest()
            applied['model_sha256'] = hashlib.sha256(Path(args.model).expanduser().read_bytes()).hexdigest()
            try:
                applied['git_commit'] = subprocess.check_output(['git', '-C', str(Path(__file__).parents[3]),
                    'rev-parse', 'HEAD'], text=True, stderr=subprocess.DEVNULL, timeout=2.).strip()
            except (OSError, subprocess.SubprocessError):
                applied['git_commit'] = None
        print('適用する試行設定: '+json.dumps(applied, ensure_ascii=False))
        model, rest = load_robot(Path(args.model).resolve(), tuple(calibration["tip_offset_m"]),
                              float(calibration["pelvis_height_m"]), float(calibration["tip_radius_m"]))
        kin = ArmKinematics(model)
        reference_q = rest.copy()
        reference_q[12:15] = calibration["reference_waist_q_rad"]
        kin.update_fixed_base(reference_q, calibration["reference_imu_rpy_rad"])
        runtime = J1CameraRuntime(args.j1gen_root)
        # カメラFKもモデルを書き換えるため、制御用モデルを共有しない。
        sensor_model, _ = load_robot(Path(args.model).expanduser().resolve(), tuple(calibration['tip_offset_m']),
                                    float(calibration['pelvis_height_m']), float(calibration['tip_radius_m']))
        sensor_kin = ArmKinematics(sensor_model)
        sensor_kin.update_fixed_base(reference_q, calibration['reference_imu_rpy_rad'])
        camera_geometry = CalibratedHeadCamera(runtime, sensor_kin, calibration)
        if policy.record_rgbd:
            recorder = TrialRecorder(rgbd_directory)

        def state_sample(configuration):
            history.append(configuration)
            q, imu, stamp, leg, gyro = configuration
            samples.append({'received_at_s': stamp, 'checked_at_s': time.monotonic(),
                'motor_q_rad': list(q), 'imu_rpy_rad': list(imu),
                'leg_speed_rad_s': leg, 'gyro_rad_s': gyro})

        detector = None
        if policy.field:
            YoloDetector = runtime.module('perception_bridge').perception('detector.yolo_detector').YoloDetector
            detector = YoloDetector(model_name=args.weights, classes=['button'],
                confidence_threshold=args.confidence, device=args.device, imgsz=args.detector_imgsz)
            detector.detect(np.zeros((480, 640, 3), dtype=np.uint8))
            if args.device == 'cpu':
                import torch
                torch.set_num_threads(2)
        if args.execute:
            if args.arm_socket:
                from arm_bridge import ArmProxy
                arm = ArmProxy(args.arm_socket)
            else:
                arm = ArmSdk(args.network_interface)
            if hasattr(arm, 'publish'):
                publish = arm.publish

                def recorded_publish(right, weight, *positional, **kwargs):
                    result = publish(right, weight, *positional, **kwargs)
                    commands.append({'sent_at_s': time.monotonic(), 'right_q_rad': list(map(float, right)),
                        'weight': float(weight), 'right_tau_nm': list(map(float, kwargs.get('right_tau', (0.,)*7)))})
                    return result

                arm.publish = recorded_publish
            arm.stop_walking()
            for _ in range(6):
                configuration = arm.current_configuration()
                if policy.field:
                    from field_motion import FieldMonitor
                    state_sample(configuration)
                    if monitor is None:
                        monitor = FieldMonitor(policy, configuration)
                    monitor.update(configuration)
                else:
                    validate_stationary_pose(configuration, calibration)
                time.sleep(0.1)
            gravity = None
            if args.gravity_scale:
                from gravity import GravityCompensation
                gravity = GravityCompensation(args.model, calibration, args.gravity_scale,
                                              dynamics=args.dynamics_feedforward)
            if policy.field:
                from field_motion import FieldArmMotion, FieldOperator
                motion = FieldArmMotion(arm, calibration, monitor=monitor, gravity=gravity, state_sink=state_sample)
                operator = FieldOperator(motion, None)
            elif gravity is not None or policy.record_rgbd:
                motion = ArmMotion(arm, calibration, gravity=gravity, state_sink=state_sample)
            print("Navigationの移動指令を止め、機体の静止と手先校正を確認してください。")
            answer = (operator.read('実機の右腕を動かす場合だけ YES と入力:') if policy.field else
                      input('実機の右腕を動かす場合だけ YES と入力:'))
            if answer.strip().upper() != 'YES':
                report['cancelled'] = True
                raise OperatorCancelled()
            if policy.field:
                motion.begin_trial()

            def frame_transform(frame, stamp):
                configuration = arm.current_configuration()
                history.append(configuration)
                q, imu = history.at(stamp, configuration, max_distance_s=.1)
                return camera_geometry.transform(q, imu), q, imu

            def current_transform():
                configuration = arm.current_configuration()
                validate_stationary_pose(configuration, calibration)
                if not 0 <= time.monotonic()-configuration[2] <= .1:
                    raise RuntimeError('カメラ計測に対応する実機状態が古すぎます')
                return camera_geometry.transform(np.array(configuration[0]), configuration[1])
        else:
            state = json.loads(Path(args.state_json).read_text())
            configuration = (state['motor_q_rad'], state['imu_rpy_rad'], time.monotonic(), 0, 0)
            if not policy.field:
                validate_stationary_pose(configuration, calibration)
            else:
                from field_motion import FieldMonitor
                monitor = FieldMonitor(policy, configuration)
                monitor.update(configuration)
            kin.update_fixed_base(np.array(configuration[0]), configuration[1])
            current_transform = camera_geometry.transform(np.array(state["motor_q_rad"]), state["imu_rpy_rad"])

            def frame_transform(frame, stamp):
                return current_transform, np.array(configuration[0]), np.array(configuration[1])

        if policy.field or recorder is not None:
            def factory():
                return RgbdLocalizer(args.weights, np.eye(4), tuple(args.target_pixel) if args.target_pixel else None,
                    confidence=args.confidence, runtime=runtime, camera_config=args.camera_config, host=args.host,
                    detector=detector, position_stability_m=policy.position_stability_m,
                    device=args.device, imgsz=args.detector_imgsz)

            def perception_wait(seconds):
                if motion is not None:
                    motion.check()
                time.sleep(seconds)

            camera = ButtonPerceptionWorker(factory, frame_transform, wait=perception_wait,
                                            recoverable=policy.field, recorder=recorder)
            if policy.field and args.execute:
                operator.localizer = camera
                args.pause_handler = operator
        else:
            camera = RgbdLocalizer(args.weights, current_transform,
                tuple(args.target_pixel) if args.target_pixel else None,
                confidence=args.confidence, runtime=runtime, camera_config=args.camera_config, host=args.host,
                position_stability_m=policy.position_stability_m, device=args.device, imgsz=args.detector_imgsz)
        if args.execute:
            report = execute_alignment(arm, camera, kin, calibration, args, motion=motion, policy=policy)
            code = button_exit_code(report, args.stop_before_contact)
        else:
            observation = camera.observe()
            if not 0 <= time.monotonic() - observation.captured_at <= 0.5:
                raise ValueError("ボタン計測が古すぎます")
            target = observation.target
            radius = calibration["tip_radius_m"]
            poses = preview_button_path(kin, observation, radius, args, policy)
            report = {"schema": "g1-live-button-preview-v1", "poses": poses,
                      "target_xyz_m": list(target.face_xyz), "physical_press_success": None,
                      "lamp_confirmed": None, "lamp_check": "not_run",
                      "lamp_min_rise": args.lamp_min_rise, "return_method": "reverse_approach",
                      "alignment_mode": args.alignment_mode,
                      "normal_motion": args.normal_motion if args.alignment_mode == "single" else "stepped",
                      "requires_mid_approach_remeasurement": args.alignment_mode == "single",
                      "single_correction_timing": vars(single_timing),
                      "correction_timing": vars(timing), 'task': 'button', 'profile': policy.profile,
                      'approach_mode': policy.approach_mode,
                      'approach_clearances_m': list(STAGED_CLEARANCES_M) if policy.approach_mode == 'staged' else [args.clearance],
                      'fk_tolerance_m': policy.fk_tolerance_m, 'fk_abort_m': policy.fk_abort_m,
                      'stop_before_contact': args.stop_before_contact}
            print(json.dumps(report, indent=2))
            print("計画表示のみ。DDS指令は送信していません。")
            code = 0
    except OperatorCancelled:
        code = 1
    except (ValueError, RuntimeError, OSError, KeyError, TypeError, ImportError) as exc:
        report = getattr(exc, 'button_report', report)
        report['error'] = str(exc)
        print(f"ERROR: {exc}", file=sys.stderr)
        code = 2
    except KeyboardInterrupt as exc:
        report = getattr(exc, 'button_report', report)
        report['error'] = '操作者による中断'
        code = 130
    finally:
        # 中途の初期化失敗・ワーカー終了失敗でも他の解除と記録を続ける。
        if motion is not None:
            try:
                motion.close()
                if motion.release_error:
                    raise RuntimeError(motion.release_error)
            except Exception as exc:
                report['release_error'] = str(exc)
                report['release_completed'] = report['completed'] = report['diagnostic_completed'] = False
                code = 2
        for name, resource in (('camera', camera), ('arm', arm)):
            if resource is None:
                continue
            try:
                if name == 'arm' and hasattr(resource, 'release'):
                    resource.release()
                if hasattr(resource, 'close'):
                    resource.close()
            except Exception as exc:
                report.setdefault('cleanup_errors', {})[name] = str(exc)
                report['completed'] = report['diagnostic_completed'] = False
                if name == 'arm':
                    report['release_completed'] = False
                code = 2
        if recorder is not None:
            report['rgbd_recording'] = recorder.close()
        if isinstance(camera, ButtonPerceptionWorker):
            report.update(measurements=list(camera.measurements), perception_frames=list(camera.diagnostics),
                          processing_times=list(camera.processing_times), perception_rejections=dict(camera.rejections))
        if monitor is not None:
            report.update(monitor_references=monitor.references, monitor_events=monitor.events)
        report.update(applied_settings=applied, calibration=calibration, monitor_samples=samples, arm_commands=commands)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    return code


if __name__ == "__main__":
    raise SystemExit(main())
