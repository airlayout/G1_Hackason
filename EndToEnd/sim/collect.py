"""G1歩行中のRGB-D・LiDAR・身体状態を、モデル入力へ変換可能なHDF5へ収集する。"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
import traceback

from g1_e2e.data.config import SensorConfig
from g1_e2e.robot import RobotSpec
from sim.envs.config import FlatConfig
from sim.runtime import ControllerConfig, add_run_arguments, configure_app, run_metadata


def main():
    from isaaclab.app import AppLauncher

    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    add_run_arguments(parser, root, controller="unitree", velocity=(0.3, 0.0, 0.0))
    parser.add_argument("--record-observer", action="store_true", help="同期した外部映像もobserver.h5へ保存")
    parser.add_argument("--sensors-config", type=Path, default=root / "configs/sensors/g1_rgbd_lidar.yaml")
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(
        device=f"cuda:{os.environ.get('G1_E2E_SELECTED_GPU', '0')}",
        headless=True,
        enable_cameras=True,
        steps=150,  # 既定の公開MID-360走査表は4秒分。収集は3秒から開始する。
        env_config=root / "configs/task/sensor_collection.yaml",
    )
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--stepsは1以上です")
    robot = RobotSpec.load(args.robot_config)
    sensors = SensorConfig.load(args.sensors_config, robot_urdf=root / robot.urdf)
    environment = FlatConfig.load(args.env_config)
    sensors.periods_in_steps(environment.physics_dt)
    if sensors.lidar.model == "livox_csv":
        from g1_e2e.data.livox import load_livox_pattern

        try:
            load_livox_pattern(
                root, sensors.lidar.pattern_path, sensors.lidar.repeat_pattern
            ).require_duration(args.steps * environment.control_dt)
        except (ValueError, FileNotFoundError) as error:
            parser.error(str(error))
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
    complete = False
    started = time.monotonic()
    try:
        from g1_e2e.data.dataset import DatasetWriter
        from g1_e2e.data.fidelity import sensor_fidelity
        from g1_e2e.data.observer import ObserverWriter
        from sim.collection import CollectionRecorder
        from sim.envs.flat import G1FlatEnv
        from sim.sensors.rig import SensorRig
        from sim.sensors.observer import ObserverCamera

        with ExitStack() as resources:
            controller = controller_config.create(robot, environment.control_dt)
            env = G1FlatEnv(root, robot, environment, device=args.device)
            resources.callback(env.close)
            rig = SensorRig(env, sensors)
            resources.callback(rig.close)
            observer = observer_writer = None
            if args.record_observer:
                observer = ObserverCamera(env)
                resources.callback(observer.close)
                observer_writer = ObserverWriter(output / "observer.h5", observer.resolution)
                resources.callback(lambda: observer_writer.close(complete=complete))
            metadata = {
                **run_metadata(root, args, controller_config),
                **rig.metadata,
                "joint_names_sdk_order": robot.joint_names,
                "control_dt_s": environment.control_dt,
                "alignment": "observation_t + latest sensors at or before t + action_(t-1) -> action_t -> next_state",
                "observer": "observer.h5: simultaneous third-person RGB; excluded from model inputs"
                if observer
                else None,
                "hardware_fidelity": sensor_fidelity(sensors, environment),
            }
            (output / "hardware-audit.json").write_text(
                json.dumps(metadata["hardware_fidelity"], ensure_ascii=False, indent=2) + "\n"
            )
            (output / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
            writer = DatasetWriter(output / "dataset.h5", metadata, rig.calibration)
            resources.callback(lambda: writer.close(complete=complete))
            recorder = CollectionRecorder(
                env, rig, writer, observer=observer, observer_writer=observer_writer
            )
            observation = env.reset(seed=args.seed)
            recorder.start_episode(observation, args.steps)
            controller.reset()
            rows = falls = 0
            while rows < args.steps and app.is_running():
                recorder.prepare(observation)
                result = env.step(controller.act(observation), after_physics_step=recorder.after_physics_step)
                recorder.append_step(observation, result)
                observation = result.observation
                rows += 1
                falls += int(result.terminated)
                if rows % 50 == 0:
                    print(f"[COLLECT] {rows}/{args.steps}; {rows * environment.control_dt:.2f} s", flush=True)
                if result.terminated or result.truncated:
                    if not args.reset_on_done or rows == args.steps:
                        break
                    observation = env.reset(seed=args.seed + observation.episode_id + 1)
                    recorder.start_episode(observation, args.steps - rows)
                    controller.reset()
            summary = {
                "steps": rows,
                "requested_steps": args.steps,
                "falls": falls,
                "simulated_seconds": rows * environment.control_dt,
                "complete": rows == args.steps,
                **recorder.summary,
                "wall_seconds": time.monotonic() - started,
                "dataset": str(output / "dataset.h5"),
            }
            (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            complete = summary["complete"]
            print(f"[SAVED] {json.dumps(summary)}", flush=True)
    except Exception:
        # Kit終了処理の前に、例外を確実にログへ残す。
        traceback.print_exc()
        raise
    finally:
        app.close()
    if not complete:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
