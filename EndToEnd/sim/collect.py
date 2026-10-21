"""G1歩行中のRGB・LiDAR・身体状態を、モデル入力へ変換可能なHDF5へ収集する。"""

from __future__ import annotations

import argparse
import cProfile
from contextlib import ExitStack
from dataclasses import replace
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import pstats
import shutil
import time
import traceback

from g1_e2e.data.config import SensorConfig
from g1_e2e.data.scene import OfficeConfig
from g1_e2e.robot import RobotSpec
from sim.envs.config import FlatConfig
from sim.runtime import ControllerConfig, add_run_arguments, configure_app, run_metadata


def main():
    from isaaclab.app import AppLauncher

    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    add_run_arguments(parser, root, controller="unitree", velocity=(0.3, 0.0, 0.0))
    parser.add_argument("--record-observer", action="store_true", help="同期した外部映像もobserver.h5へ保存")
    parser.add_argument("--sensors-config", type=Path, default=root / "configs/sensors/g1_rgb_lidar.yaml")
    parser.add_argument("--scene-config", type=Path, help="公式OfficeのUSD・巡回設定")
    parser.add_argument("--patrol", action="store_true", help="通行領域を巡回する上位教師で速度指令を生成")
    parser.add_argument("--profile", action="store_true", help="収集ループの処理時間をprofile.txtへ保存")
    parser.add_argument(
        "--repeat-lidar-pattern", action="store_true", help="公開4秒走査表の周期再生を明示的に許可"
    )
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
    if args.repeat_lidar_pattern:
        sensors = replace(sensors, lidar=replace(sensors.lidar, repeat_pattern=True))
    office = OfficeConfig.load(args.scene_config) if args.scene_config else None
    if args.patrol and (office is None or args.controller != "unitree"):
        parser.error("--patrolには--scene-configと--controller unitreeが必要です")
    if office is not None and not (office.directory(root) / "office-assets.lock.json").is_file():
        parser.error("先にbash EndToEnd/prepare-office.shでOfficeを取得してください")
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
    training_accepted = True
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
            env = G1FlatEnv(root, robot, environment, device=args.device, office=office)
            resources.callback(env.close)
            rig = SensorRig(env, sensors)
            resources.callback(rig.close)
            teacher = None
            if args.patrol:
                from g1_e2e.navigation.patrol import PatrolTeacher
                from sim.envs.office import navigation_grid

                grid = navigation_grid(rig.scene_vertices, rig.scene_faces, office)
                grid.save(output / "navigation-grid.npz")
                teacher = PatrolTeacher(grid, office)
                print(
                    f"[READY] Office patrol: {int(grid.walkable.sum())} free cells; {len(rig.scene_faces)} triangles",
                    flush=True,
                )
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
                "scene": env.scene_metadata,
                "patrol_teacher": {
                    "kind": "coverage-grid A* and path-following velocity commands",
                    "uses_simulation_truth": True,
                    "model_inputs_include_teacher_map_or_goal": False,
                    "scene_config_yaml": args.scene_config.read_text(),
                }
                if teacher
                else None,
                "velocity_command": {
                    "components": ["base_vx_m_s", "base_vy_m_s", "base_yaw_rate_rad_s"],
                    "teacher": "applied high-level command, distinct from measured base velocity",
                    "available": hasattr(controller, "velocity_command"),
                },
                "alignment": "observation_t + latest sensors at or before t + action_(t-1) -> action_t -> next_state",
                "observer": "observer.h5: simultaneous third-person RGB; excluded from model inputs"
                if observer
                else None,
                "hardware_fidelity": sensor_fidelity(sensors, environment),
            }
            if office:
                shutil.copyfile(
                    office.directory(root) / "office-assets.lock.json", output / "scene-assets.lock.json"
                )
            (output / "hardware-audit.json").write_text(
                json.dumps(metadata["hardware_fidelity"], ensure_ascii=False, indent=2) + "\n"
            )
            (output / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
            writer = DatasetWriter(output / "dataset.h5", metadata, rig.calibration)
            resources.callback(lambda: writer.close(complete=complete))
            recorder = CollectionRecorder(
                env, rig, writer, observer=observer, observer_writer=observer_writer
            )
            observation = env.reset(
                seed=args.seed, spawn_xy_yaw=teacher.spawn(args.seed) if teacher else None
            )
            recorder.start_episode(observation, args.steps)
            controller.reset()
            from sim.sensors.audit import mounting_audit

            (output / "mounting-audit.json").write_text(
                json.dumps(mounting_audit(env, rig, root / robot.urdf), ensure_ascii=False, indent=2) + "\n"
            )
            rows = falls = 0
            patrol_episodes = []
            profile = resources.enter_context(cProfile.Profile()) if args.profile else None
            collection_started = time.monotonic()
            while rows < args.steps and app.is_running():
                if teacher:
                    controller.set_velocity(teacher.command(observation))
                recorder.prepare(observation)
                command = controller.act(observation)
                velocity_command = getattr(controller, "velocity_command", None)
                result = env.step(
                    command, render=not args.headless, after_physics_step=recorder.after_physics_step
                )
                recorder.append_step(observation, result, velocity_command=velocity_command)
                observation = result.observation
                rows += 1
                falls += int(result.terminated)
                if rows % 50 == 0:
                    print(f"[COLLECT] {rows}/{args.steps}; {rows * environment.control_dt:.2f} s", flush=True)
                if result.terminated or result.truncated:
                    if not args.reset_on_done or rows == args.steps:
                        break
                    seed = args.seed + observation.episode_id + 1
                    if teacher:
                        patrol_episodes.append(teacher.summary)
                        teacher.reset()
                    observation = env.reset(seed=seed, spawn_xy_yaw=teacher.spawn(seed) if teacher else None)
                    recorder.start_episode(observation, args.steps - rows)
                    controller.reset()
            collection_seconds = time.monotonic() - collection_started
            if profile is not None:
                profile.disable()
                profile.dump_stats(str(output / "profile.pstats"))
                with (output / "profile.txt").open("w") as report:
                    pstats.Stats(profile, stream=report).strip_dirs().sort_stats("cumulative").print_stats(50)
            summary = {
                "steps": rows,
                "requested_steps": args.steps,
                "falls": falls,
                "simulated_seconds": rows * environment.control_dt,
                "complete": rows == args.steps,
                "patrol_episodes": [*patrol_episodes, teacher.summary] if teacher else [],
                **recorder.summary,
                "wall_seconds": time.monotonic() - started,
                "collection_wall_seconds": collection_seconds,
                "collection_realtime_factor": rows * environment.control_dt / collection_seconds,
                "dataset": str(output / "dataset.h5"),
            }
            if teacher:
                from g1_e2e.data.quality import patrol_quality

                summary["patrol_quality"] = patrol_quality(summary, office)
                training_accepted = summary["patrol_quality"]["accepted"]
                writer.file.attrs["patrol_training_accepted"] = training_accepted
            (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            complete = summary["complete"]
            print(f"[SAVED] {json.dumps(summary)}", flush=True)
    except Exception:
        # Kit終了処理の前に、例外を確実にログへ残す。
        traceback.print_exc()
        raise
    finally:
        app.close()
    if not complete or not training_accepted:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
