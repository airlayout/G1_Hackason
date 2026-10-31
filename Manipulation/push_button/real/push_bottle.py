#!/usr/bin/env python3
"""台上のペットボトルを前方に押す。既定はRGB-Dによる計画表示だけ。"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import subprocess
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bottle import (BottleFrame, BottleLocalizer, BottlePushRoutine, add_bottle_arguments,
                    add_table, make_bottle_detector, settings_from_args)
from kinematics import ArmKinematics, load_robot
from vision import CameraIntrinsics
from align_button import ArmMotion, read_calibration, validate_stationary_pose
from j1gen_camera import CalibratedHeadCamera, J1CameraRuntime, J1RgbdCamera, add_camera_arguments
from push_button import ArmSdk
from bottle_perception import PerceptionEngine, TableEstimate, make_perception_worker
from pose_history import PoseHistory
from trial_policy import add_trial_arguments, resolve_trial_arguments, trial_exit_code
from trial_recording import TrialRecorder


def build_argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Menagerieの29DoF g1.xml")
    parser.add_argument("--base-transform", required=True, help="既存の実測カメラ・手先・静止姿勢の校正JSON")
    parser.add_argument("--state-json", help="計画表示時に必要な現在の29軸関節角とIMU")
    parser.add_argument("--network-interface", default="enp3s0")
    parser.add_argument("--arm-socket")
    parser.add_argument("--output", required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--calibrated", action="store_true")
    parser.add_argument("--table-edge-tolerance", type=float, default=.004,
                        help="台の対辺から求めた補正量の許容差[m]。0.004〜0.015")
    add_bottle_arguments(parser)
    add_camera_arguments(parser)
    add_trial_arguments(parser)
    return parser


def main(argv=None):
    parser = build_argument_parser()
    try:
        args, policy = resolve_trial_arguments(parser, argv)
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    recording_directory = Path(args.output).with_suffix('').with_name(Path(args.output).stem+'_rgbd')
    if policy.record_rgbd and (Path(args.output).exists() or recording_directory.exists()):
        parser.error('過去の試行を残すため、新しい--outputを指定してください（JSONまたはRGB-D保存先が存在します）')
    camera = arm = routine = worker = motion = recorder = monitor = operator = None
    calibration = None
    worker_cameras = []
    monitor_samples = []
    commands = []
    table_reference = None
    table_rejections = []
    report = {"schema": "g1-bottle-push-v1", "task": "bottle", "completed": False,
              "physical_push_success": None}
    applied = {'profile': policy.profile, 'trial_policy': asdict(policy), 'arguments': vars(args),
               'clock': {'epoch_s': time.time(), 'monotonic_s': time.monotonic()}}
    print('適用する試行設定: '+json.dumps(applied, ensure_ascii=False))
    try:
        settings = settings_from_args(args)
        if args.execute and not args.calibrated:
            raise ValueError("実測校正後に--execute --calibratedを指定してください")
        if not args.execute and not args.state_json:
            raise ValueError("計画表示には現在関節角の--state-jsonが必要です")
        calibration, _ = read_calibration(args.base_transform, require_button=False)
        sources = sorted(Path(__file__).parent.glob('*.py'))
        sources += sorted(Path(__file__).parents[1].glob('*.py'))
        applied['source_sha256'] = {str(path.relative_to(Path(__file__).parents[3])):
            hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
        try:
            applied['git_commit'] = subprocess.check_output(
                ['git', '-C', str(Path(__file__).parents[3]), 'rev-parse', 'HEAD'],
                text=True, stderr=subprocess.DEVNULL, timeout=2.).strip()
        except (OSError, subprocess.SubprocessError):
            applied['git_commit'] = None
        applied['calibration_sha256'] = hashlib.sha256(Path(args.base_transform).read_bytes()).hexdigest()
        applied['model_sha256'] = hashlib.sha256(Path(args.model).expanduser().read_bytes()).hexdigest()
        if policy.record_rgbd:
            recorder = TrialRecorder(recording_directory)
        model, rest = load_robot(Path(args.model).expanduser().resolve(), tuple(calibration["tip_offset_m"]),
                                 float(calibration["pelvis_height_m"]), float(calibration["tip_radius_m"]),
                                 scene_builder=lambda root: add_table(root, settings))
        kin = ArmKinematics(model)
        reference = rest.copy()
        reference[12:15] = calibration["reference_waist_q_rad"]
        kin.update_fixed_base(reference, calibration["reference_imu_rpy_rad"])
        runtime = J1CameraRuntime(args.j1gen_root)
        # Camera FK mutates its model: it must never share the controller's model.
        sensor_model, _ = load_robot(Path(args.model).expanduser().resolve(), tuple(calibration["tip_offset_m"]),
                                    float(calibration["pelvis_height_m"]), float(calibration["tip_radius_m"]))
        sensor_kin = ArmKinematics(sensor_model)
        sensor_kin.update_fixed_base(reference, calibration["reference_imu_rpy_rad"])
        geometry = CalibratedHeadCamera(runtime, sensor_kin, calibration)
        pose_history = PoseHistory()
        if args.track_table:
            from table_reference import TableReference
            table_reference = TableReference(settings, float(calibration["pelvis_height_m"]),
                                             opposite_edge_tolerance_m=policy.table_edge_tolerance,
                                             selected_edges=None if policy.table_x_edge == policy.table_y_edge == 'auto'
                                             else (policy.table_x_edge, policy.table_y_edge))

        def table_snapshot():
            if table_reference is None or table_reference.stamp is None:
                return None
            return TableEstimate(table_reference.stamp, tuple(table_reference.offset), tuple(table_reference.velocity))

        def sensor_frame(frame, stamp, configuration):
            q, imu = pose_history.at(stamp, configuration, max_distance_s=.1)
            if recorder is not None:
                recorder.submit(frame, stamp, motor_q_rad=q, imu_rpy_rad=imu)
            nominal = geometry.transform(q, imu)
            if table_reference is not None:
                base = sensor_kin.body_transform('pelvis')
                yaw = imu[2]-calibration['reference_imu_rpy_rad'][2]
                cy, sy = np.cos(yaw), np.sin(yaw)
                heading = np.eye(4)
                heading[:2, :2] = [[cy, -sy], [sy, cy]]
                nominal, base = heading@nominal, heading@base
                delta = table_reference.update(frame.depth_m(), frame.intrinsics, nominal, base, stamp)
                nominal = delta@nominal
            intr = frame.intrinsics
            return BottleFrame(frame.color_bgr, frame.depth_m(),
                               CameraIntrinsics(intr.fx, intr.fy, intr.cx, intr.cy), nominal, stamp, frame.frame_id)

        detector = make_bottle_detector(args.weights, args.confidence, args.device, runtime=runtime,
                                        imgsz=args.detector_imgsz)
        if args.execute:
            # Pay lazy model initialization before opening the robot connection.
            detector.detect(np.zeros((480, 640, 3), dtype=np.uint8))
            if args.device == 'cpu':
                # Ultralytics may reset the thread count during lazy backend setup.
                import torch
                torch.set_num_threads(2)
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
                                     'weight': float(weight), 'right_tau_nm': list(map(float,
                                         kwargs.get('right_tau', (0.,)*7)))})
                    return result

                arm.publish = recorded_publish
            def state_sample(configuration):
                pose_history.append(configuration)
                joints, imu, stamp, leg_speed, gyro = configuration
                monitor_samples.append({'received_at_s': stamp, 'checked_at_s': time.monotonic(),
                    'leg_speed_rad_s': leg_speed, 'gyro_rad_s': gyro,
                    'imu_rpy_rad': list(imu), 'waist_q_rad': list(joints[12:15]),
                    'motor_q_rad': list(joints)})

            arm.stop_walking()
            for _ in range(6):
                observed=arm.current_configuration()
                state_sample(observed)
                if policy.field:
                    from field_motion import FieldMonitor
                    if monitor is None:
                        monitor = FieldMonitor(policy, observed)
                    monitor.update(observed)
                else:
                    validate_stationary_pose(observed, calibration)
                time.sleep(0.1)
            if policy.field:
                from field_motion import FieldArmMotion, FieldOperator
                gravity = None
                if args.gravity_scale:
                    from gravity import GravityCompensation
                    gravity = GravityCompensation(args.model, calibration, args.gravity_scale,
                                                  dynamics=args.dynamics_feedforward)
                motion = FieldArmMotion(arm, calibration, monitor=monitor, gravity=gravity, state_sink=state_sample)
                operator = FieldOperator(motion, None)
            print("Navigationを停止し、台の寸法・移動先の空き・手先校正を確認してください。")
            confirmation = (operator.read('実機の右腕を動かす場合だけ YES と入力:')
                            if operator is not None else input('実機の右腕を動かす場合だけ YES と入力:'))
            if (confirmation.strip().upper() if policy.field else confirmation.strip()) != "YES":
                report["cancelled"] = True
                return 1 if policy.field else 0
            if policy.field:
                motion.begin_trial()

            def refresh():
                configuration = arm.current_configuration()
                state_sample(configuration)
                if policy.field:
                    monitor.update(configuration)
                    motion.check()
                else:
                    validate_stationary_pose(configuration, calibration)
                if not 0 <= time.monotonic()-configuration[2] <= .1:
                    raise RuntimeError('押下制御の状態が100msより古い、または時刻が不正です')
                offset = worker.table_offset(strict=False) if args.track_table else None
                yaw = configuration[1][2]-calibration['reference_imu_rpy_rad'][2] if args.track_table else 0.
                kin.update_fixed_base(np.array(configuration[0]), configuration[1], base_offset=offset, yaw=yaw)
                return configuration

            def read_sensor():
                # The worker owns this socket for its entire lifetime.
                if not worker_cameras:
                    worker_cameras.append(J1RgbdCamera(runtime, args.camera_config, args.host))
                delivered = worker_cameras[0].poll()
                if delivered is None:
                    return None
                frame, stamp = delivered
                configuration = arm.current_configuration()
                pose_history.append(configuration)
                try:
                    return sensor_frame(frame, stamp, configuration)
                except ValueError as exc:
                    table_rejections.append({'time_s': time.monotonic(), 'reason': str(exc)})
                    raise

            def close_sensor():
                if worker_cameras:
                    worker_cameras[0].close()

            engine = PerceptionEngine(read_sensor, detector, settings, time.monotonic, args.target_pixel,
                                      table_snapshot=table_snapshot, close=close_sensor,
                                      position_stability_m=policy.position_stability_m, recoverable=policy.field)
            def perception_wait(seconds):
                configuration = arm.current_configuration()
                state_sample(configuration)
                if policy.field:
                    monitor.update(configuration)
                    motion.check()
                else:
                    validate_stationary_pose(configuration, calibration)
                time.sleep(seconds)

            worker = make_perception_worker(engine, time.monotonic, perception_wait)
            localizer = worker
            if not policy.field:
                refresh()
            else:
                kin.update_fixed_base(np.array(observed[0]), observed[1])
        else:
            state = json.loads(Path(args.state_json).read_text())
            configuration = (state["motor_q_rad"], state["imu_rpy_rad"], time.monotonic(), 0, 0)
            if not policy.field:
                validate_stationary_pose(configuration, calibration)
            else:
                from field_motion import FieldMonitor
                monitor = FieldMonitor(policy, configuration)
                monitor.update(configuration)
            kin.update_fixed_base(np.array(configuration[0]), configuration[1])
            transform = geometry.transform(np.array(configuration[0]), configuration[1])
            camera = J1RgbdCamera(runtime, args.camera_config, args.host)

            def read_frame(after, deadline):
                epoch_after = time.time()-(time.monotonic()-after)
                frame, stamp = camera.read_after(epoch_after, deadline)
                if recorder is not None:
                    recorder.submit(frame, stamp, motor_q_rad=configuration[0], imu_rpy_rad=configuration[1])
                intr = frame.intrinsics
                frame_transform = transform
                if table_reference is not None:
                    frame_transform = table_reference.update(frame.depth_m(), intr, transform,
                        sensor_kin.body_transform('pelvis'), stamp)@transform
                return BottleFrame(frame.color_bgr, frame.depth_m(),
                                   CameraIntrinsics(intr.fx, intr.fy, intr.cx, intr.cy), frame_transform, stamp, frame.frame_id)

            localizer = BottleLocalizer(read_frame, detector, settings, time.monotonic, args.target_pixel,
                                        position_stability_m=policy.position_stability_m)
            refresh = lambda: None
        if args.execute:
            if not policy.field and args.gravity_scale:
                from gravity import GravityCompensation
                motion = ArmMotion(arm, calibration, gravity=GravityCompensation(args.model, calibration, args.gravity_scale,
                                                                             dynamics=args.dynamics_feedforward),
                                   state_sink=state_sample)
            elif not policy.field:
                motion = ArmMotion(arm, calibration, state_sink=state_sample)
            pause_handler = None
            if policy.field:
                operator.localizer = localizer
                pause_handler = operator
            routine = BottlePushRoutine(kin, motion, localizer, refresh, time.monotonic,
                                        settings, calibration["tip_radius_m"], calibration["position_uncertainty_m"],
                                        tracking_check=None if table_reference is None else worker.table_offset,
                                        live_tracking=True, stop_before_contact=args.stop_before_contact,
                                        pause_handler=pause_handler, approach_mode=policy.approach_mode,
                                        approach_extra_retries=policy.approach_extra_retries)
            report = routine.execute()
            code = trial_exit_code(report, policy, args.stop_before_contact)
            if policy.field:
                print('試行結果: '+json.dumps({key: report.get(key) for key in (
                    'motion_completed', 'home_completed', 'release_completed', 'visual_success',
                    'measured_displacement_m', 'error')}, ensure_ascii=False))
                if code == 0 and not args.stop_before_contact:
                    print('押下指令・復帰・解除の処理が完了しました。ボトルの実際の移動と直立は別に記録します。')
                    assessment = {}
                    for key, prompt in (('moved', 'ボトルは意図した方向へ動きましたか'),
                                        ('upright', 'ボトルは直立を保ちましたか')):
                        try:
                            answer = operator.read(prompt+' [y/n/u=未確認]: ', check_live=False)
                        except EOFError:
                            answer = 'u'
                        assessment[key] = True if answer in ('y', 'yes') else False if answer in ('n', 'no') else None
                    report['operator_assessment'] = assessment
            return code
        routine = BottlePushRoutine(kin, None, localizer, refresh, time.monotonic,
                                    settings, calibration["tip_radius_m"], calibration["position_uncertainty_m"],
                                    approach_mode=policy.approach_mode,
                                    approach_extra_retries=policy.approach_extra_retries)
        observation = localizer.observe()
        report = routine.preview(observation)
        report["measurements"] = localizer.measurements
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print("計画表示のみ。DDS指令は送信していません。")
        return 0
    except (ValueError, RuntimeError, OSError, KeyError, TypeError, ImportError) as exc:
        report = routine.report if routine is not None else report
        report["error"] = str(exc)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        report = routine.report if routine is not None else report
        report["error"] = "操作者による中断"
        return 130
    finally:
        release_failed = False
        if motion is not None and routine is None and hasattr(motion, 'close'):
            motion.close()
            if getattr(motion, 'release_error', None):
                release_failed = True
                report['release_error'] = motion.release_error
        if worker is not None:
            try:
                worker.close()
            except (RuntimeError, OSError) as exc:
                report["perception_close_error"] = str(exc)
                report['completed'] = report['diagnostic_completed'] = False
                release_failed = True
            report["processing_times"] = worker.engine.processing_times
            report["perception_rejections"] = dict(worker.rejections)
            report['measurements'] = worker.measurements
            report['measurement_count'] = len(report['measurements'])
            report['perception_frames'] = worker.diagnostics
        if monitor_samples:
            report["monitor_samples"] = monitor_samples
            report["monitor_limits"] = {"leg_speed_rad_s": policy.leg_speed_rad_s, "gyro_rad_s": policy.gyro_rad_s,
                "waist_delta_rad": float(np.deg2rad(policy.waist_delta_deg)), "imu_delta_deg": policy.imu_delta_deg, "state_age_s": .4,
                "control_state_age_s": .1}
        if camera is not None:
            try:
                camera.close()
            except (RuntimeError,OSError) as exc:
                report['camera_close_error'] = str(exc)
        if arm is not None:
            try:
                arm.release()
                if args.arm_socket:
                    arm.close()
            except (OSError, RuntimeError) as exc:
                report["release_error"] = str(exc)
                report["completed"] = False
                report["release_completed"] = False
                report['diagnostic_completed'] = False
                release_failed = True
        output = Path(args.output)
        if routine is not None and 'ik_searches' in routine.report:
            report['ik_searches'] = routine.report['ik_searches']
        if table_reference is not None:
            report['table_edge_tolerance_m'] = table_reference.opposite_edge_tolerance_m
            report['table_reference'] = table_reference.measurements
            report['table_diagnostics'] = table_reference.diagnostics
            report['table_rejections'] = table_rejections
        if recorder is not None:
            report['rgbd_recording'] = recorder.close()
        report['applied_settings'] = applied
        report['calibration'] = calibration
        report['arm_commands'] = commands
        if monitor is not None:
            report['trial_events'] = monitor.events
            report['monitor_references'] = monitor.references
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
        if release_failed:
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
