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
        detector = make_bottle_detector(args.weights, args.confidence, args.device, runtime=runtime)
        if args.execute:
            if args.arm_socket:
                from arm_bridge import ArmProxy
                arm = ArmProxy(args.arm_socket)
            else:
                arm = ArmSdk(args.network_interface)
            arm.stop_walking()
            for _ in range(6):
                validate_stationary_pose(arm.current_configuration(), calibration)
                time.sleep(0.1)
            print("Navigationを停止し、台の寸法・移動先の空き・手先校正を確認してください。")
            if input("実機の右腕を動かす場合だけ YES と入力: ").strip() != "YES":
                report["cancelled"] = True
                return 0

            def refresh():
                configuration = arm.current_configuration()
                validate_stationary_pose(configuration, calibration)
                kin.update_fixed_base(np.array(configuration[0]), configuration[1])
                return configuration

            def current_transform():
                configuration = refresh()
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
            return BottleFrame(frame.color_bgr, frame.depth_m(),
                               CameraIntrinsics(intr.fx, intr.fy, intr.cx, intr.cy),
                               current_transform(), captured_at, frame.frame_id)

        localizer = BottleLocalizer(read_frame, detector, settings, time.monotonic, args.target_pixel)
        if args.execute:
            motion = ArmMotion(arm, calibration)
            routine = BottlePushRoutine(kin, motion, localizer, refresh, time.monotonic,
                                        settings, calibration["tip_radius_m"], calibration["position_uncertainty_m"])
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
            camera.close()
        if arm is not None:
            try:
                arm.release()
                if args.arm_socket:
                    arm.close()
            except (OSError, RuntimeError) as exc:
                report["release_error"] = str(exc)
                report["completed"] = False
                release_failed = True
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
        if release_failed:
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
