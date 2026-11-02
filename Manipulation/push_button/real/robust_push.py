#!/usr/bin/env python3
"""Opt-in robust planner using live RGB-D; SDK is opened only with --execute."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import threading

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robust_options import add_arguments, parse
from robust_scene import estimate_bottle
from robust_planner import search
from robust_perception import ContinuousObserver, LatestFrameSampler, Observation, snapshot
from robust_execution import execute, validate_start
from vision import CameraIntrinsics, estimate_button_target
from kinematics import load_robot, ArmKinematics
from j1gen_camera import J1CameraRuntime, J1RgbdCamera, CalibratedHeadCamera, add_camera_arguments
from align_button import read_calibration, validate_stationary_pose, ArmMotion
from pose_history import PoseHistory
from button_feedback import lamp_level
from button_detector import make_button_detector, DEFAULT_CONFIDENCE, DEFAULT_IMAGE_SIZE
from motion_limits import CONTROL_PERIOD_S
from robust_policy import execution_policy
from robust_conditions import (BUTTON_CONDITION_COUNT,validate_button_plan,
                               validate_button_scenario_settings)


def collect_outcome_observations(observer, *, now=time.monotonic, sleep=time.sleep):
    """Wait briefly for three new measurements without treating held images as results."""
    results, last_id = [], observer.latest(allow_stale=True).frame_id
    deadline = now()+2.
    while len(results) < 3 and now() < deadline:
        final = observer.latest(allow_stale=True)
        if final.frame_id > last_id and 0 <= now()-final.captured_at <= .5:
            results.append(final); last_id = final.frame_id
        sleep(.02)
    return results


def button_lamp_outcome(baseline, observations, min_rise):
    """Mixed/absent measurements cannot confirm either lamp state."""
    if (baseline is None or not np.isfinite(baseline) or len(observations) != 3
            or any(v.lamp is None or not np.isfinite(v.lamp) for v in observations)):
        return None
    lit = [v.lamp-baseline >= min_rise for v in observations]
    return True if all(lit) else False if not any(lit) else None


def main(argv=None, *, task=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=('button','bottle'), default=task or 'button')
    parser.add_argument('--model', required=True)
    parser.add_argument('--base-transform', required=True)
    parser.add_argument('--state-json')
    parser.add_argument('--output', required=True)
    parser.add_argument('--weights', help='ボタンは省略時に同梱final8を使う')
    parser.add_argument('--button-direction', choices=('up', 'down'))
    parser.add_argument('--target-pixel', type=int, nargs=2)
    parser.add_argument('--confidence', type=float)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--detector-imgsz', type=int)
    parser.add_argument('--network-interface', default='enp3s0')
    parser.add_argument('--arm-socket')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--calibrated', action='store_true')
    parser.add_argument('--stop-before-contact', action='store_true')
    parser.add_argument('--static-target', action='store_true',
        help='ボトルの接触手前診断のみ: 初回の静止計測後は画像の経過時間で停止しない')
    parser.add_argument('--allow-stale-observation', action='store_true',
        help='ボトル・画像補正offのみ: 初回3枚以降は画像の経過時間で停止せず、古い間は検証済み軌道の推定位置を使う')
    parser.add_argument('--execute-after-plan',action='store_true',
        help='ボトルのデータ収集: 3条件合格後に即実行し、姿勢差・FK誤差は記録のみ')
    parser.add_argument('--execute-verified-plan',action='store_true',
        help='ボタン: 4条件合格後、ボトルと同じ監視・同期送信で即実行。最後5mmから5cm退避まで目標固定')
    parser.add_argument('--lamp-min-rise',type=float,default=30.,help='ボタンの点灯判定に必要な増光量')
    add_arguments(parser); add_camera_arguments(parser)
    parser.set_defaults(motion_planner='robust')
    args, settings = parse(parser, argv, task=task)
    if args.motion_planner != 'robust':
        parser.error('Use real/run_push.py for the legacy planner')
    if task is not None:
        args.task = task
    if args.task == 'bottle' and not args.weights:
        parser.error('ボトルには--weightsが必要です')
    if args.confidence is None:
        args.confidence = DEFAULT_CONFIDENCE if args.task == 'button' else .25
    if args.detector_imgsz is None:
        args.detector_imgsz = DEFAULT_IMAGE_SIZE if args.task == 'button' else 960
    policy = execution_policy(args.execute_after_plan or args.execute_verified_plan)
    report = dict(schema='g1-robust-trial-v1', task=args.task, completed=False, physical_push_success=None,
                  visual_success=None,
                  settings=asdict(settings), arguments=vars(args))
    report['image_policy'] = dict(static_target_assumed=args.static_target,
        initial_three_fresh_frames_required=True,
        continuous_age_check=not (args.static_target or args.allow_stale_observation),
        stale_bottle_pose='verified_reference_prediction' if args.allow_stale_observation else None,
        outcome_frames_required=not (args.stop_before_contact or args.allow_stale_observation),
        capture_pose='current_stationary_state' if args.static_target else 'capture_time_history')
    if args.execute_verified_plan:
        report['image_policy'].update(button_contact_occlusion=not args.stop_before_contact,
            fixed_interval_timeout_s=None)
    report['execute_after_plan'] = args.execute_after_plan
    report['execute_verified_plan'] = args.execute_verified_plan
    report['execution_policy'] = policy.to_dict()
    report['planning_warnings'] = []
    report['pose_history_policy'] = dict(sample_period_s=CONTROL_PERIOD_S,
        warmup_s=.6,max_gap_s=.1,on_unmatched_frame='discard_and_recapture')
    report['pose_history_rejections'] = []
    arm = motion = observer = camera = None
    state_stop = threading.Event()
    state_thread = None
    output = Path(args.output)
    if output.exists():
        parser.error('Use a new --output; previous trial will not be overwritten')
    try:
        if not np.isfinite(args.lamp_min_rise) or args.lamp_min_rise <= 0:
            raise ValueError('--lamp-min-rise must be finite and positive')
        if args.execute_verified_plan and (not args.execute or args.task != 'button' or
                settings.scenarios != BUTTON_CONDITION_COUNT or args.execute_after_plan):
            raise ValueError('--execute-verified-plan requires button --execute and 4 scenarios, without --execute-after-plan')
        if args.task=='button' and args.execute:
            validate_button_scenario_settings(settings.scenarios,settings.position_span_m)
        if args.task == 'button' and (not isinstance(args.stroke,(int,float)) or isinstance(args.stroke,bool)
                or not np.isfinite(args.stroke) or not .0015 <= args.stroke <= .015):
            raise ValueError('Button mode requires a measured stroke between 1.5 and 15mm')
        if args.static_target and (args.task != 'bottle' or not args.stop_before_contact):
            raise ValueError('--static-target requires bottle --stop-before-contact')
        if args.allow_stale_observation and (args.task != 'bottle' or settings.bottle_feedback != 'off'):
            raise ValueError('--allow-stale-observation requires bottle with --bottle-feedback off')
        if args.execute_after_plan and (not args.execute or args.task != 'bottle' or settings.scenarios != 3):
            raise ValueError('--execute-after-plan requires bottle --execute and 3 scenarios')
        if args.execute and not args.calibrated:
            raise ValueError('--execute requires --calibrated')
        if not args.execute and not args.state_json:
            raise ValueError('Planning preview requires --state-json')
        calibration, _ = read_calibration(args.base_transform, require_button=args.task=='button',
                                          require_target_margin=args.execute_verified_plan)
        report['calibration'] = calibration
        runtime = J1CameraRuntime(args.j1gen_root)
        # Model warm-up occurs before the initial capture and search deadline.
        if args.task == 'button':
            detector = make_button_detector(args.weights, args.confidence, args.device, runtime=runtime,
                imgsz=args.detector_imgsz, direction=args.button_direction)
        else:
            detector_type = runtime.module('perception_bridge').perception('detector.yolo_detector').YoloDetector
            detector = detector_type(model_name=args.weights, classes=['bottle'],
                confidence_threshold=args.confidence, device=args.device, imgsz=args.detector_imgsz)
            detector.detect(np.zeros((480,640,3), dtype=np.uint8))
        model, rest = load_robot(Path(args.model).resolve(), tuple(calibration['tip_offset_m']),
            calibration['pelvis_height_m'],calibration['tip_radius_m'])
        sensor_kin = ArmKinematics(model)
        sensor_kin.update_fixed_base(rest, calibration['reference_imu_rpy_rad'])
        geometry = CalibratedHeadCamera(runtime, sensor_kin, calibration)
        history = PoseHistory()
        if args.execute:
            if args.arm_socket:
                from arm_bridge import ArmProxy
                arm = ArmProxy(args.arm_socket)
            else:
                from push_button import ArmSdk
                arm = ArmSdk(args.network_interface)
            arm.stop_walking()
            report['state_samples'] = []
            def record_state(value):
                history.append(value)
                report['state_samples'].append(dict(sampled_at_s=time.monotonic(),received_at_s=value[2],
                    motor_q_rad=list(value[0]),imu_rpy_rad=list(value[1]),
                    leg_speed_rad_s=float(value[3]),gyro_rad_s=float(value[4])))
            def read_configuration():
                value = arm.current_configuration()
                record_state(value)
                validate_stationary_pose(value,calibration,allow_pose_drift=policy.record_deviations)
                return value
            state_lock = threading.Lock()
            state_cache = dict(value=None,error=None)
            # Populate the same 0.6s window at the running 50Hz cadence. The
            # old 100ms cadence exceeded the 100ms interpolation limit with
            # even small RPC/scheduling overheads and rejected the first image.
            for _ in range(round(.6/CONTROL_PERIOD_S)):
                state_cache['value'] = read_configuration()
                time.sleep(CONTROL_PERIOD_S)
            def sample_state():
                try:
                    while not state_stop.is_set():
                        value = read_configuration()
                        with state_lock:
                            state_cache['value'] = value
                        state_stop.wait(CONTROL_PERIOD_S)
                except Exception as exc:
                    with state_lock:
                        state_cache['error'] = exc
            state_thread = threading.Thread(target=sample_state,name='robust-state',daemon=True)
            state_thread.start()
            def configuration():
                with state_lock:
                    value,error = state_cache['value'],state_cache['error']
                if error is not None:
                    raise RuntimeError(f'State sampler stopped: {error}') from error
                max_age = policy.state_timeout_s
                if value is None or not 0 <= time.monotonic()-value[2] <= max_age:
                    raise RuntimeError(f'Robot state older than {max_age*1000:.0f}ms')
                return value
        else:
            state = json.loads(Path(args.state_json).read_text())
            def configuration():
                return (state['motor_q_rad'],state['imu_rpy_rad'],time.monotonic(),0.,0.)
        camera = J1RgbdCamera(runtime,args.camera_config,args.host)
        reference = [None]
        stable_base = [args.bottle_base_height]
        perception_rejections = []
        sampler = LatestFrameSampler()
        def read():
            delivered = sampler.consume(camera.poll(), time.monotonic())
            if delivered is None:
                return None
            frame, stamp = delivered
            if not 0 <= time.monotonic()-stamp <= .5:
                return None
            config = configuration()
            # The camera is torso-fixed. A stationary-target diagnostic uses the
            # fresh, stationary torso state rather than an image-time history lookup.
            try:
                q, imu = (history.at(stamp, config, max_distance_s=.1)
                          if args.execute and not args.static_target else config[:2])
            except ValueError as exc:
                # This image cannot be paired with a measured robot pose.
                # Discard it and let the normal camera loop take a newer one;
                # never substitute the current pose or stop on a single gap.
                report['pose_history_rejections'].append(dict(frame_id=frame.frame_id,
                    captured_at_s=float(stamp),rejected_at_s=time.monotonic(),reason=str(exc)))
                return None
            transform = geometry.transform(q,imu)
            intr = frame.intrinsics
            intr = CameraIntrinsics(intr.fx,intr.fy,intr.cx,intr.cy)
            candidates = []
            for detected in detector.detect(frame.color_bgr):
                if reference[0] is None and args.target_pixel:
                    u,v = args.target_pixel; x0,y0,x1,y1 = detected.bbox
                    if not x0 <= u <= x1 or not y0 <= v <= y1:
                        continue
                try:
                    if args.task == 'bottle':
                        observed = estimate_bottle(frame.depth_m(),detected.bbox,intr,transform,
                            base_height=stable_base[0],
                            contact_height=args.contact_height)
                        lamp = None
                    else:
                        target = estimate_button_target(frame.depth_m(),detected.bbox,intr,transform,
                            expected_face_xyz=None if reference[0] is None else reference[0]['face'],
                            tracking=reference[0] is not None)
                        observed = dict(face=target.face_xyz, normal=target.press_direction,
                                        radius=calibration['button_radius_m'])
                        lamp = lamp_level(frame.color_bgr,detected.bbox)
                    if reference[0] is not None and (np.linalg.norm(np.array(observed['face'])-reference[0]['face']) > .025
                            or abs(observed['radius']-reference[0]['radius']) > .005):
                        continue
                    candidates.append((observed,lamp))
                except ValueError as exc:
                    perception_rejections.append(str(exc))
            if len(candidates) != 1:
                return None
            observed,lamp = candidates[0]
            reference[0] = observed
            return Observation(observed,tuple(q),tuple(imu),stamp,frame.frame_id,lamp)
        observer = ContinuousObserver(read,camera.close)
        initial = observer.initial()
        if args.task == 'bottle':
            stable_base[0] = initial.geometry['base_height']
        scene = snapshot(args.task,initial,calibration,args)
        def latest_observation():
            if args.static_target or args.allow_stale_observation:
                return observer.latest(allow_stale=True)
            return observer.latest()
        def monitor():
            warnings = validate_start(scene,configuration(),latest_observation(),
                                      record_deviations=policy.record_deviations)
            for warning in warnings:
                if warning not in report['planning_warnings']:
                    report['planning_warnings'].append(warning)
        plan = search(args.model,calibration,scene,settings,monitor=monitor)
        report['plan'] = plan.to_dict()
        if plan.candidate is None:
            return 1
        if args.task=='button' and args.execute:
            validate_button_plan(report['plan'])
        if not args.execute:
            report['planning_completed'] = True
            return 0
        if args.execute_after_plan or args.execute_verified_plan:
            if args.execute_verified_plan:
                monitor()
            action = ('接触手前の確認' if args.stop_before_contact else
                f'{scene.stroke*1000:g}mmのボタン押下' if args.task == 'button' else
                f'{scene.push_distance*100:.1f}cmの押し込み')
            print(f'シミュレーション{settings.scenarios}条件合格。直ちに'+action+'動作を開始します。',flush=True)
        else:
            monitor()
            print('Navigationを停止し、Bモデルの配置条件と校正を確認してください。')
            if input('実機の右腕を動かす場合だけ YES と入力: ').strip() != 'YES':
                report['cancelled'] = True
                return 1
            monitor()
        torque = {'value':np.zeros(7),'waist':np.zeros(3)}
        motion = ArmMotion(arm,calibration,gravity=lambda config,velocity=None: torque['value'].copy(),
                           waist_gravity=lambda config: torque['waist'].copy(),
                           state_sink=record_state,limits=settings.motion_limits,
                           allow_pose_drift=policy.record_deviations,synchronous_stream=policy.synchronous_stream)
        report['upper_body_hold'] = motion.upper_body_hold
        report['execution'] = execute(plan,args.model,calibration,motion,configuration,latest_observation,settings,
            set_torque=lambda value: torque.update(value=np.array(value)),diagnostic=args.stop_before_contact,
            record_deviations=policy.record_deviations,set_waist_torque=lambda value: torque.update(waist=np.array(value)),
            allow_stale_observation=args.allow_stale_observation,
            observe_held=(lambda:observer.latest(allow_stale=True))
                if args.execute_verified_plan and not args.stop_before_contact else None)
        report['command_publications'] = motion.command_publications
        if motion.error is not None:
            report['motion_thread_error'] = str(motion.error)
        report['completed'] = report['execution']['completed']
        if args.execute_verified_plan and report['execution'].get('error'):
            return 2
        if report['completed'] and not args.stop_before_contact:
            # Outcome measurements do not update an execution target or restart the push.
            try:
                results = collect_outcome_observations(observer)
            except RuntimeError as exc:
                if not args.allow_stale_observation and args.task != 'button':
                    raise
                results = []
                report['outcome_error'] = str(exc)
            report['outcome_frame_count'] = len(results)
            report['outcome_available'] = len(results) == 3
            if len(results) != 3:
                report['outcome_error'] = report.get('outcome_error','Three fresh outcome frames unavailable')
                if not args.allow_stale_observation and args.task != 'button':
                    raise RuntimeError(report['outcome_error'])
            if args.task == 'button':
                report['visual_success'] = button_lamp_outcome(initial.lamp,results,args.lamp_min_rise)
                report['lamp_check'] = dict(baseline=initial.lamp,min_rise=args.lamp_min_rise,
                    levels=[v.lamp for v in results],frame_ids=[v.frame_id for v in results],
                    status='lit' if report['visual_success'] is True else
                        'no_rise' if report['visual_success'] is False else 'unknown')
                report['motion_completed'] = bool(report['execution'].get('motion_completed'))
                report['physical_press_success'] = None
            elif args.task == 'bottle' and len(results) == 3:
                deltas = np.array([v.geometry['face'] for v in results])-scene.face
                delta = np.median(deltas,axis=0)
                report['measured_displacement_m'] = delta.tolist()
                report['visual_success'] = bool(np.all(np.abs(deltas[:,0]-scene.push_distance) <= .01)
                                                and np.all(np.abs(deltas[:,1]) <= .01))
        outcome_ok = (args.stop_before_contact or report.get('visual_success') is True
                      or (args.allow_stale_observation and report.get('visual_success') is None))
        return 0 if report['completed'] and outcome_ok else 1
    except KeyboardInterrupt:
        report['error'] = 'Interrupted'; return 130
    except (ValueError,RuntimeError,OSError,KeyError,ImportError) as exc:
        report['error'] = str(exc); return 2
    finally:
        for resource in (motion,observer):
            if resource is not None and hasattr(resource,'close'):
                try:
                    resource.close()
                except Exception as exc:
                    report.setdefault('cleanup_errors',[]).append(str(exc))
        if observer is None and camera is not None:
            try:
                camera.close()
            except Exception as exc:
                report.setdefault('cleanup_errors',[]).append(str(exc))
        state_stop.set()
        if state_thread is not None:
            state_thread.join(timeout=1.)
            if state_thread.is_alive():
                report.setdefault('cleanup_errors',[]).append('State sampler did not stop')
        if arm is not None and hasattr(arm,'release'):
            try:
                arm.release()
            except Exception as exc:
                report.setdefault('cleanup_errors',[]).append(str(exc))
        if arm is not None and hasattr(arm,'close'):
            try:
                arm.close()
            except Exception as exc:
                report.setdefault('cleanup_errors',[]).append(str(exc))
        if observer is not None:
            report['perception_frames'] = observer.frames
            report['perception_rejections'] = dict(observer.rejections)
            report['geometry_rejections'] = perception_rejections
        output.parent.mkdir(parents=True,exist_ok=True)
        if report.get('cleanup_errors'):
            report['completed'] = False
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if report.get('cleanup_errors'):
            return 2


if __name__ == '__main__':
    raise SystemExit(main())
