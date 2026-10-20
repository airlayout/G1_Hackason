"""G1歩行中のRGB-D・LiDAR・身体状態を、モデル入力へ変換可能なHDF5へ収集する。"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

from g1_e2e.data.config import SensorConfig
from g1_e2e.robot import RobotSpec
from sim.envs.config import FlatConfig
from sim.runtime import ControllerConfig, add_run_arguments, configure_app, run_metadata


def main():
    from isaaclab.app import AppLauncher

    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    add_run_arguments(parser, root, controller="unitree", velocity=(0.3, 0.0, 0.0))
    parser.add_argument("--sensors-config", type=Path, default=root / "configs/sensors/g1_rgbd_lidar.yaml")
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(
        device=f"cuda:{os.environ.get('G1_E2E_SELECTED_GPU', '0')}", headless=True, enable_cameras=True
    )
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--stepsは1以上です")
    sensors = SensorConfig.load(args.sensors_config)
    robot = RobotSpec.load(args.robot_config)
    environment = FlatConfig.load(args.env_config)
    sensors.periods_in_steps(environment.control_dt)
    try:
        controller_config = ControllerConfig.from_args(args, root)
    except ValueError as error:
        parser.error(str(error))
    gpu = configure_app(args, root, capture=True)
    output = args.output or root / "datasets" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    app = AppLauncher(
        args,
        active_gpu=gpu,
        multi_gpu=False,
        renderer="PathTracing",
        samples_per_pixel_per_frame=8,
        max_bounces=3,
        width=sensors.camera.width,
        height=sensors.camera.height,
    ).app
    env = rig = writer = None
    complete, rows, falls = False, 0, 0
    started = time.monotonic()
    try:
        from g1_e2e.data.dataset import DatasetWriter
        from sim.envs.flat import G1FlatEnv
        from sim.sensors.rig import SensorRig

        controller = controller_config.create(robot, environment.control_dt)
        env = G1FlatEnv(root, robot, environment, device=args.device)
        rig = SensorRig(env, sensors)
        metadata = {
            **run_metadata(root, args, controller_config),
            **rig.metadata,
            "joint_names_sdk_order": robot.joint_names,
            "control_dt_s": environment.control_dt,
            "alignment": "observation_t + latest sensors at or before t + action_(t-1) -> action_t -> next_state",
        }
        (output / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
        writer = DatasetWriter(output / "dataset.h5", metadata, rig.calibration)
        observation = env.reset(seed=args.seed)
        controller.reset()
        camera_index = lidar_index = -1
        previous = None
        quality = {"rgb_mean": 0.0, "rgb_saturated": 0.0, "depth_valid": 0.0, "lidar_returns": 0}
        for _ in range(args.steps):
            if not app.is_running():
                break
            if observation.step % rig.camera_stride == 0:
                frame = rig.capture_camera(observation)
                camera_index = writer.append_camera(frame)
                quality["rgb_mean"] += float(frame.rgb.mean())
                quality["rgb_saturated"] += float((frame.rgb >= 254).all(axis=-1).mean())
                quality["depth_valid"] += float(frame.depth_valid.mean())
            if observation.step % rig.lidar_stride == 0:
                frame = rig.capture_lidar(observation)
                lidar_index = writer.append_lidar(frame)
                quality["lidar_returns"] += int(frame.valid.sum())
            acceleration = rig.specific_force(observation)
            result = env.step(controller.act(observation))
            writer.append_step(
                observation,
                result,
                camera_index=camera_index,
                lidar_index=lidar_index,
                previous_command=previous,
                specific_force_body_m_s2=acceleration,
            )
            previous, observation = result.applied_command, result.observation
            rows += 1
            falls += int(result.terminated)
            if rows % 50 == 0:
                print(f"[COLLECT] {rows}/{args.steps}; {rows * environment.control_dt:.2f} s", flush=True)
            if result.terminated or result.truncated:
                if not args.reset_on_done:
                    break
                observation = env.reset(seed=args.seed + observation.episode_id + 1)
                controller.reset()
                previous = None
                camera_index = lidar_index = -1
        complete = rows == args.steps
        camera_count = int(writer.file.attrs["committed_camera_frames"])
        lidar_count = int(writer.file.attrs["committed_lidar_frames"])
        summary = {
            "steps": rows,
            "requested_steps": args.steps,
            "falls": falls,
            "simulated_seconds": rows * environment.control_dt,
            "complete": complete,
            "camera_frames": camera_count,
            "lidar_frames": lidar_count,
            "rgb_mean_u8": quality["rgb_mean"] / camera_count if camera_count else None,
            "rgb_saturated_pixel_fraction": quality["rgb_saturated"] / camera_count if camera_count else None,
            "depth_valid_fraction": quality["depth_valid"] / camera_count if camera_count else None,
            "lidar_valid_returns_mean": quality["lidar_returns"] / lidar_count if lidar_count else None,
            "wall_seconds": time.monotonic() - started,
            "dataset": str(output / "dataset.h5"),
        }
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(f"[SAVED] {json.dumps(summary)}", flush=True)
    finally:
        if writer is not None:
            writer.close(complete=complete)
        if rig is not None:
            rig.close()
        if env is not None:
            env.close()
        app.close()
    if not complete:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
