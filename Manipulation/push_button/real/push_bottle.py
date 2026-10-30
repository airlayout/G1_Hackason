#!/usr/bin/env python3
"""台上のペットボトルを前方に押す。既定はRGB-Dによる計画表示だけ。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Menagerieの29DoF g1.xml")
    parser.add_argument("--base-transform", required=True, help="既存の実測カメラ・手先・静止姿勢の校正JSON")
    parser.add_argument("--state-json", help="計画表示時に必要な現在の29軸関節角とIMU")
    parser.add_argument("--network-interface", default="enp3s0")
    parser.add_argument("--arm-socket")
    parser.add_argument("--output", required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--calibrated", action="store_true")
    add_bottle_arguments(parser)
    add_camera_arguments(parser)
    args = parser.parse_args(argv)
    camera = arm = routine = None
    table_reference = None
    table_rejections = []
    report = {"schema": "g1-bottle-push-v1", "task": "bottle", "completed": False,
              "physical_push_success": None}
    try:
        settings = settings_from_args(args)
        if args.execute and not args.calibrated:
            raise ValueError("実測校正後に--execute --calibratedを指定してください")
        if not args.execute and not args.state_json:
            raise ValueError("計画表示には現在関節角の--state-jsonが必要です")
        calibration, _ = read_calibration(args.base_transform, require_button=False)
        model, rest = load_robot(Path(args.model).expanduser().resolve(), tuple(calibration["tip_offset_m"]),
                                 float(calibration["pelvis_height_m"]), float(calibration["tip_radius_m"]),
                                 scene_builder=lambda root: add_table(root, settings))
        kin = ArmKinematics(model)
        reference = rest.copy()
        reference[12:15] = calibration["reference_waist_q_rad"]
        kin.update_fixed_base(reference, calibration["reference_imu_rpy_rad"])
        runtime = J1CameraRuntime(args.j1gen_root)
        geometry = CalibratedHeadCamera(runtime, kin, calibration)
        if args.track_table:
            from table_reference import TableReference
            from pose_history import PoseHistory
            table_reference = TableReference(settings, float(calibration["pelvis_height_m"]))
            pose_history=PoseHistory()

            def table_observation(frame,stamp,configuration):
                q,rpy=pose_history.at(stamp,configuration)
                nominal=geometry.transform(q,rpy)
                base=kin.body_transform('pelvis')
                yaw=rpy[2]-calibration['reference_imu_rpy_rad'][2]
                cy,sy=np.cos(yaw),np.sin(yaw)
                heading=np.eye(4);heading[:2,:2]=[[cy,-sy],[sy,cy]]
                nominal,base=heading@nominal,heading@base
                delta=table_reference.update(frame.depth_m(),frame.intrinsics,nominal,base,stamp)
                return delta@nominal
        detector = make_bottle_detector(args.weights, args.confidence, args.device, runtime=runtime,
                                        imgsz=args.detector_imgsz)
        if args.execute:
            if args.arm_socket:
                from arm_bridge import ArmProxy
                arm = ArmProxy(args.arm_socket)
            else:
                arm = ArmSdk(args.network_interface)
            arm.stop_walking()
            for _ in range(6):
                observed=arm.current_configuration()
                validate_stationary_pose(observed, calibration)
                if table_reference is not None:
                    pose_history.append(observed)
                time.sleep(0.1)
            print("Navigationを停止し、台の寸法・移動先の空き・手先校正を確認してください。")
            if input("実機の右腕を動かす場合だけ YES と入力: ").strip() != "YES":
                report["cancelled"] = True
                return 0

            last_table_poll = [-np.inf]

            def refresh(poll_table=True):
                configuration = arm.current_configuration()
                validate_stationary_pose(configuration, calibration)
                if table_reference is not None:
                    pose_history.append(configuration)
                # 記録関節経路の逆再生とhomeでは画像待ちを追加しない。
                # 関節・IMU・通信の監視は継続し、最後の台位置で経路を検査する。
                returning = routine is not None and routine.report.get('phase') in ('abort_return', 'home', 'done')
                if (poll_table and not returning and table_reference is not None and camera is not None
                        and time.monotonic()-last_table_poll[0] >= .05):
                    last_table_poll[0] = time.monotonic()
                    delivered = camera.poll()
                    if delivered is not None:
                        frame,stamp = delivered
                        try:
                            table_observation(frame,stamp,configuration)
                        except ValueError as exc:
                            table_rejections.append({'time_s':time.monotonic(),'reason':str(exc)})
                offset = None if table_reference is None else table_reference.predict(time.monotonic(),strict=False)
                yaw = 0. if table_reference is None else configuration[1][2]-calibration['reference_imu_rpy_rad'][2]
                kin.update_fixed_base(np.array(configuration[0]), configuration[1],base_offset=offset,yaw=yaw)
                return configuration

            def current_transform():
                configuration = refresh(False)
                if time.monotonic()-configuration[2] > 0.10:
                    raise RuntimeError("画像計測に対応する実機状態が古すぎます")
                return geometry.transform(np.array(configuration[0]), configuration[1])
        else:
            state = json.loads(Path(args.state_json).read_text())
            configuration = (state["motor_q_rad"], state["imu_rpy_rad"], time.monotonic(), 0, 0)
            validate_stationary_pose(configuration, calibration)
            kin.update_fixed_base(np.array(configuration[0]), configuration[1])
            transform = geometry.transform(np.array(configuration[0]), configuration[1])
            current_transform = lambda: transform
            refresh = lambda: None
        camera = J1RgbdCamera(runtime, args.camera_config, args.host)

        def read_frame(after, deadline):
            epoch_after = time.time()-(time.monotonic()-after)
            frame, captured_at = camera.read_after(epoch_after, deadline)
            intr = frame.intrinsics
            transform = current_transform()
            if table_reference is not None:
                observed=arm.current_configuration() if args.execute else configuration
                transform=table_observation(frame,captured_at,observed)
                if args.execute:
                    refresh(False)
            return BottleFrame(frame.color_bgr, frame.depth_m(),
                               CameraIntrinsics(intr.fx, intr.fy, intr.cx, intr.cy),
                               transform, captured_at, frame.frame_id)

        localizer = BottleLocalizer(read_frame, detector, settings, time.monotonic, args.target_pixel)
        if args.execute:
            if args.gravity_scale:
                from gravity import GravityCompensation
                motion = ArmMotion(arm, calibration, gravity=GravityCompensation(args.model, calibration, args.gravity_scale,
                                                                             dynamics=args.dynamics_feedforward),
                                   state_sink=pose_history.append if table_reference is not None else None)
            else:
                motion = (ArmMotion(arm, calibration,state_sink=pose_history.append)
                          if table_reference is not None else ArmMotion(arm,calibration))
            routine = BottlePushRoutine(kin, motion, localizer, refresh, time.monotonic,
                                        settings, calibration["tip_radius_m"], calibration["position_uncertainty_m"],
                                        tracking_check=None if table_reference is None else lambda:table_reference.predict(time.monotonic()))
            report = routine.execute()
            return 0 if report["completed"] and report["visual_success"] is True else 1
        routine = BottlePushRoutine(kin, None, localizer, refresh, time.monotonic,
                                    settings, calibration["tip_radius_m"], calibration["position_uncertainty_m"])
        observation = localizer.observe()
        report = routine.preview(observation)
        report["measurements"] = localizer.measurements
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print("計画表示のみ。DDS指令は送信していません。")
        return 0
    except (ValueError, RuntimeError, OSError, KeyError, TypeError, ImportError) as exc:
        report["error"] = str(exc)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        report = routine.report if routine is not None else report
        report["error"] = "操作者による中断"
        return 130
    finally:
        release_failed = False
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
                release_failed = True
        output = Path(args.output)
        if table_reference is not None:
            report['table_reference'] = table_reference.measurements
            report['table_rejections'] = table_rejections
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
        if release_failed:
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
