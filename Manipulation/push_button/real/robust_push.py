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
    add_arguments(parser); add_camera_arguments(parser)
    parser.set_defaults(motion_planner='robust')
    args, settings = parse(parser, argv)
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
    report = dict(schema='g1-robust-trial-v1', task=args.task, completed=False, physical_push_success=None,
                  settings=asdict(settings), arguments=vars(args))
    arm = motion = observer = camera = None
    state_stop = threading.Event()
    state_thread = None
    output = Path(args.output)
    if output.exists():
        parser.error('Use a new --output; previous trial will not be overwritten')
    try:
        raw = list(sys.argv[1:] if argv is None else argv)
        supplied = any(v == '--stroke' or v.startswith('--stroke=') for v in raw)
        if args.trial_config:
            supplied = supplied or 'stroke' in json.loads(Path(args.trial_config).read_text())
        if args.task == 'button' and not supplied:
            raise ValueError('Button mode requires an explicit measured --stroke (CLI or trial JSON)')
        if args.execute and not args.calibrated:
            raise ValueError('--execute requires --calibrated')
        if not args.execute and not args.state_json:
            raise ValueError('Planning preview requires --state-json')
        calibration, _ = read_calibration(args.base_transform, require_button=args.task=='button',
                                          require_target_margin=False)
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
            def read_configuration():
                value = arm.current_configuration()
                validate_stationary_pose(value, calibration)
                history.append(value)
                return value
            state_lock = threading.Lock()
            state_cache = dict(value=None,error=None)
            for _ in range(6):
                state_cache['value'] = read_configuration(); time.sleep(.1)
            def sample_state():
                try:
                    while not state_stop.is_set():
                        value = read_configuration()
                        with state_lock:
                            state_cache['value'] = value
                        state_stop.wait(.02)
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
                if value is None or not 0 <= time.monotonic()-value[2] <= .1:
                    raise RuntimeError('Robot state older than 100ms')
                return value
        else:
            state = json.loads(Path(args.state_json).read_text())
            def configuration():
                return (state['motor_q_rad'],state['imu_rpy_rad'],time.monotonic(),0.,0.)
        camera = J1RgbdCamera(runtime,args.camera_config,args.host,max_age_s=None)
        reference = [None]
        stable_base = [args.bottle_base_height]
        perception_rejections = []
        sampler = LatestFrameSampler()
        def read():
            delivered = sampler.consume(camera.poll(), time.monotonic())
            if delivered is None:
                return None
            frame, stamp = delivered
            config = configuration()
            q, imu = history.at(stamp, config) if args.execute else config[:2]
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
        def monitor():
            validate_start(scene,configuration(),observer.latest())
        plan = search(args.model,calibration,scene,settings,monitor=monitor)
        report['plan'] = plan.to_dict()
        if plan.candidate is None:
            return 1
        if not args.execute:
            report['planning_completed'] = True
            return 0
        monitor()
        print('Navigationを停止し、Bモデルの配置条件と校正を確認してください。')
        if input('実機の右腕を動かす場合だけ YES と入力: ').strip() != 'YES':
            report['cancelled'] = True
            return 1
        monitor()
        torque = {'value':np.zeros(7)}
        motion = ArmMotion(arm,calibration,gravity=lambda config,velocity=None: torque['value'].copy(),
                           state_sink=history.append,limits=settings.motion_limits)
        report['execution'] = execute(plan,args.model,calibration,motion,configuration,observer.latest,settings,
            set_torque=lambda value: torque.update(value=np.array(value)),diagnostic=args.stop_before_contact)
        report['completed'] = report['execution']['completed']
        if report['completed'] and not args.stop_before_contact:
            # Outcome measurements do not update an execution target or restart the push.
            results, last_id = [], observer.latest().frame_id
            deadline = time.monotonic()+2.
            while len(results) < 3 and time.monotonic() < deadline:
                final = observer.latest()
                if final.frame_id > last_id:
                    results.append(final); last_id = final.frame_id
                time.sleep(.02)
            if len(results) != 3:
                raise RuntimeError('Three distinct outcome frames unavailable')
            if args.task == 'button':
                report['visual_success'] = bool(initial.lamp is not None and all(
                    v.lamp is not None and v.lamp-initial.lamp >= 30 for v in results))
            else:
                deltas = np.array([v.geometry['face'] for v in results])-scene.face
                delta = np.median(deltas,axis=0)
                report['measured_displacement_m'] = delta.tolist()
                report['visual_success'] = bool(np.all(np.abs(deltas[:,0]-scene.push_distance) <= .01)
                                                and np.all(np.abs(deltas[:,1]) <= .01))
        return 0 if report['completed'] and (args.stop_before_contact or report.get('visual_success')) else 1
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
