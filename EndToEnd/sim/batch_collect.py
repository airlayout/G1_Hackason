"""Officeの複製環境、タイルRTX描画、GPUセンサーで同じ入力を並列収集する。"""

import argparse
import fcntl
import gc
from contextlib import ExitStack
from dataclasses import replace
from datetime import datetime, timezone
import json
import math
import hashlib
import os
from pathlib import Path
import time
import traceback
import shutil

from g1_e2e.data.config import SensorConfig
from g1_e2e.data.scene import OfficeConfig
from g1_e2e.robot import RobotSpec
from sim.batch.schedule import CaptureSchedule
from sim.envs.config import FlatConfig
from sim.runtime import ControllerConfig, add_run_arguments, configure_app, run_metadata


def main():
    from isaaclab.app import AppLauncher

    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    add_run_arguments(parser, root, controller="unitree", velocity=(0.3, 0, 0))
    parser.add_argument("--num-envs", type=int, default=4)
    parser.add_argument("--scene-config", type=Path, default=root / "configs/scenes/office.yaml")
    parser.add_argument("--sensors-config", type=Path, default=root / "configs/sensors/g1_rgb_lidar.yaml")
    parser.add_argument("--camera-updates", type=int, default=1)
    parser.add_argument("--max-bounces", type=int, default=3)
    parser.add_argument(
        "--warmup-seconds", type=float, default=0.2, help="記録開始前の姿勢を落ち着かせる物理時間"
    )
    parser.add_argument(
        "--path-samples",
        type=int,
        default=8,
        help="Path Tracingの1画素あたりのサンプル数。解像度・周期は変更しない",
    )
    parser.add_argument("--compression", choices=["none", "lzf"], default="lzf")
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(
        device=f"cuda:{os.environ.get('G1_E2E_SELECTED_GPU', '0')}",
        headless=True,
        enable_cameras=True,
        steps=150,
        env_config=root / "configs/task/office_patrol.yaml",
    )
    args = parser.parse_args()
    if args.num_envs < 1 or args.camera_updates < 1 or args.path_samples < 1 or args.max_bounces < 3:
        parser.error("num-envs・camera-updates・path-samplesは正数、max-bouncesは3以上です")
    if not math.isfinite(args.warmup_seconds) or args.warmup_seconds < 0:
        parser.error("warmup-secondsは有限の非負値です")
    spec = RobotSpec.load(args.robot_config)
    config = FlatConfig.load(args.env_config)
    sensors = SensorConfig.load(args.sensors_config, robot_urdf=root / spec.urdf)
    sensors = replace(sensors, lidar=replace(sensors.lidar, repeat_pattern=True))
    schedule = CaptureSchedule.from_configs(config, sensors)
    try:
        schedule.validate_steps(args.steps)
    except ValueError as error:
        parser.error(str(error))
    if args.reset_on_done:
        parser.error("並列収集は1 episode単位です。別seedのshardで継続してください")
    if args.controller != "unitree":
        parser.error("並列巡回の歩行controllerはunitreeを指定してください")
    if args.steps * config.control_dt > config.episode_seconds:
        parser.error("1回の収集はepisode_seconds以下にしてください。別seedのshardで継続できます")
    office = OfficeConfig.load(args.scene_config)
    if hashlib.sha256(office.usd_path(root).read_bytes()).hexdigest() != office.asset_sha256:
        parser.error("Office USDのSHA-256が設定と異なります")
    controller_config = ControllerConfig.from_args(args, root)
    gpu = configure_app(args, root, capture=True)
    output = (
        args.output or root / "datasets" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    ).resolve()
    output.mkdir(parents=True, exist_ok=False)
    lock = (root / "assets/cache/parallel-collection.lock").open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("別の並列収集シミュレーターが実行中です。終了を待ってください") from None
    lock.seek(0)
    lock.truncate()
    lock.write(str(os.getpid()))
    lock.flush()
    shutil.copyfile(office.directory(root) / "office-assets.lock.json", output / "scene-assets.lock.json")
    app = AppLauncher(
        args,
        active_gpu=gpu,
        multi_gpu=False,
        renderer="PathTracing",
        samples_per_pixel_per_frame=args.path_samples,
        width=sensors.camera.width,
        height=sensors.camera.height,
    ).app
    env = rig = recorder = None
    accepted = None
    started = time.perf_counter()
    try:
        import numpy as np
        import torch
        from g1_e2e.data.fidelity import sensor_fidelity
        from g1_e2e.data.quality import patrol_quality
        from g1_e2e.navigation.patrol import PatrolTeacher
        from sim.batch.environment import VectorPatrolEnv, shared_controllers
        from sim.batch.recording import BatchRecorder
        from sim.batch.collection import ParallelCollector
        from sim.batch.sensors import VectorSensors
        from sim.envs.office import navigation_grid

        env = VectorPatrolEnv(root, spec, config, office, args.num_envs, args.device)
        rig = VectorSensors(env, sensors, schedule, render_updates=args.camera_updates)
        grid = navigation_grid(rig.vertices, rig.faces, office)
        grid.save(output / "navigation-grid.npz")
        np.savez(output / "scene-mesh.npz", vertices=rig.vertices, faces=rig.faces)
        teachers = [PatrolTeacher(grid, office) for _ in range(args.num_envs)]
        controllers = shared_controllers(controller_config.create(spec, config.control_dt), args.num_envs)
        observations = env.reset([teacher.spawn(args.seed + i) for i, teacher in enumerate(teachers)])
        for _ in range(round(args.warmup_seconds / config.physics_dt)):
            env.physics_step()
        env.ticks = 0
        observations = env.observe()
        env.sim.forward()
        rig.capture_inertial(0)
        rig.warmup()
        metadata = {
            **run_metadata(root, args, controller_config),
            "sensor_config": sensors.to_dict(),
            "control_dt_s": config.control_dt,
            "joint_names_sdk_order": spec.joint_names,
            "joint_torque": "implicit PD estimate at current control observation: clip(Kp*(held_target-q)-Kd*dq, effort limits); not a force sensor",
            "primary_imu": sensors.body_imus.primary,
            "hardware_fidelity": sensor_fidelity(sensors, config),
            "gravity_world_m_s2": list(env.sim.cfg.gravity),
            "scene": {
                "kind": "office",
                "source": office.url,
                "asset_lock_sha256": hashlib.sha256(
                    (output / "scene-assets.lock.json").read_bytes()
                ).hexdigest(),
            },
            "parallel": {
                "num_envs": args.num_envs,
                "spacing_m": env.spacing_m,
                "world_frame": "per-environment Office coordinates; clone translation removed",
                "renderer": "PathTracing",
                "render_updates": args.camera_updates,
                "path_samples_per_pixel_per_update": args.path_samples,
                "max_bounces": args.max_bounces,
                "light_counts": env.light_counts,
                "compression": args.compression,
                "unrecorded_settling_seconds": round(args.warmup_seconds / config.physics_dt)
                * config.physics_dt,
                "clock": "seconds from recording epoch after settling; physics state is not teleported again",
            },
            "lidar": {
                **rig.pattern.metadata,
                **rig.pattern.source,
                "pose_interpolation_interval_s": config.physics_dt,
                "geometry": "static Office; self occlusion and dynamic objects unmodeled",
            },
            "velocity_command": {
                "components": ["base_vx_m_s", "base_vy_m_s", "base_yaw_rate_rad_s"],
                "teacher": "applied high-level velocity command",
            },
            "patrol_teacher": {
                "uses_simulation_truth": True,
                "model_inputs_include_teacher_map_or_goal": False,
            },
        }
        (output / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
        recorder = BatchRecorder(
            output,
            metadata,
            rig.calibration,
            args.num_envs,
            schedule=schedule,
            imu_names=rig.names,
            primary_imu=sensors.body_imus.primary,
            point_rate=rig.pattern.rate,
            compression=None if args.compression == "none" else args.compression,
        )
        collector = ParallelCollector(env, rig, recorder, teachers, controllers, schedule)
        print(
            f"[READY] {args.num_envs} G1; {len(rig.faces)} shared LiDAR triangles; PathTracing",
            flush=True,
        )
        torch.cuda.synchronize(args.device)
        collection_start = time.perf_counter()
        observations = collector.run(observations, args.steps)
        recorder.drain()
        # 最終commit・圧縮・ディスクへのflushを含めて計測する。
        summaries = []
        for i, teacher in enumerate(teachers):
            teacher.command(observations[i])
            summary = {
                "simulated_seconds": args.steps * config.control_dt,
                "falls": int(collector.fallen[i]),
                "patrol_episodes": [teacher.summary],
            }
            summaries.append({**summary, "patrol_quality": patrol_quality(summary, office)})
        accepted = [s["patrol_quality"]["accepted"] for s in summaries]
        recorder.close(complete=True, accepted=accepted)
        recorder = None
        torch.cuda.synchronize(args.device)
        elapsed = time.perf_counter() - collection_start
        sim_seconds = args.steps * config.control_dt
        summary = {
            "complete": True,
            "num_envs": args.num_envs,
            "steps_per_env": args.steps,
            "simulated_seconds_per_env": sim_seconds,
            "aggregate_simulated_seconds": sim_seconds * args.num_envs,
            "collection_wall_seconds": elapsed,
            "aggregate_realtime_factor": sim_seconds * args.num_envs / elapsed,
            "per_environment_realtime_factor": sim_seconds / elapsed,
            "accepted_environments": sum(accepted),
            "accepted_realtime_factor": sum(accepted) * sim_seconds / elapsed,
            "sensor_frames_per_env": schedule.frame_counts(args.steps),
            "stage_wall_seconds": collector.durations,
            "wall_seconds_including_setup": time.perf_counter() - started,
            "cuda_peak_allocated_bytes": torch.cuda.max_memory_allocated(args.device),
            "environments": summaries,
        }
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(f"[SAVED] {json.dumps(summary)}", flush=True)
    except Exception:
        (output / "failure.json").write_text(
            json.dumps({"error": traceback.format_exc()}, ensure_ascii=False, indent=2) + "\n"
        )
        traceback.print_exc()
        raise
    finally:
        # 先の終了処理が失敗しても、残るセンサー・物理scene・Kitをすべて閉じる。
        with ExitStack() as cleanup:
            cleanup.callback(lock.close)
            cleanup.callback(app.close)
            if env is not None:
                cleanup.callback(env.close)
            # Kit終了前にRTX annotatorの循環参照も解放する。
            cleanup.callback(gc.collect)
            if rig is not None:
                cleanup.callback(rig.close)
                rig = None
            if recorder is not None:
                cleanup.callback(recorder.close, complete=False, accepted=accepted)


if __name__ == "__main__":
    main()
