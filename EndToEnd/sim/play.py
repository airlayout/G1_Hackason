"""G1平地環境に任意のControllerを接続する実行入口。import時にはアプリを起動しない。"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

import numpy as np

from g1_e2e.robot import RobotSpec
from g1_e2e.data.scene import OfficeConfig
from sim.envs.config import FlatConfig
from sim.runtime import ControllerConfig, add_run_arguments, configure_app, run_metadata


def main() -> None:
    """引数の検証後にIsaacを1回だけ起動する。"""
    from isaaclab.app import AppLauncher

    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    add_run_arguments(parser, root, controller="hold", velocity=(0.0, 0.0, 0.0))
    parser.add_argument("--fixed-base", action="store_true", help="基部を固定して関節指令だけを検証")
    parser.add_argument("--real-time", action="store_true", help="実時間に合わせて表示")
    parser.add_argument("--scene-config", type=Path, help="公式Officeでの物理検証")
    parser.add_argument(
        "--patrol", action="store_true", help="カメラ・LiDARを取得せず巡回教師と歩行方策を検証"
    )
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device=f"cuda:{os.environ.get('G1_E2E_SELECTED_GPU', '0')}")
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--stepsは1以上を指定してください")
    spec = RobotSpec.load(args.robot_config)
    config = FlatConfig.load(args.env_config)
    office = OfficeConfig.load(args.scene_config) if args.scene_config else None
    if args.patrol and (office is None or args.controller != "unitree"):
        parser.error("--patrolには--scene-configと--controller unitreeが必要です")
    try:
        controller_config = ControllerConfig.from_args(args, root)
    except ValueError as error:
        parser.error(str(error))
    output = args.output or root / "runs" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output.mkdir(parents=True, exist_ok=True)
    if any((output / name).exists() for name in ["summary.json", "trajectory.npz", "provenance.json"]):
        parser.error(f"出力先は使用済みです: {output}")

    gpu_index = configure_app(args, root)
    app = AppLauncher(args, active_gpu=gpu_index, multi_gpu=False).app
    env = None
    try:
        # Isaac依存モジュールは必ずAppLauncherの起動後に読み込む。
        from sim.envs.flat import G1FlatEnv

        controller = controller_config.create(spec, config.control_dt)
        env = G1FlatEnv(root, spec, config, device=args.device, fixed_base=args.fixed_base, office=office)
        teacher = None
        if args.patrol:
            from g1_e2e.navigation.patrol import PatrolTeacher
            from sim.envs.office import navigation_grid
            from sim.sensors.scene import static_mesh

            points, faces, _ = static_mesh(env.sim.stage, env.static_roots)
            grid = navigation_grid(points, faces, office)
            grid.save(output / "navigation-grid.npz")
            teacher = PatrolTeacher(grid, office)
        provenance = run_metadata(root, args, controller_config) | {"fixed_base": args.fixed_base}
        (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
        summary = run(app, env, controller, args, output, teacher=teacher)
        print(f"[RESULT] {json.dumps(summary)}", flush=True)
        print(f"[SAVED] {output}", flush=True)
    finally:
        if env is not None:
            env.close()
        app.close()
    if (
        summary["falls"]
        or summary["completed_steps"] != args.steps
        or not summary.get("patrol_quality", {}).get("accepted", True)
    ):
        raise SystemExit(1)


def run(app, env, controller, args: argparse.Namespace, output: Path, *, teacher=None) -> dict:
    """環境とcontrollerを接続し、動作検証用の遷移と集計を保存する。"""
    config = env.config
    obs = env.reset(seed=args.seed, spawn_xy_yaw=teacher.spawn(args.seed) if teacher else None)
    controller.reset()
    rows: dict[str, list] = {}
    falls = 0
    min_height = float("inf")
    elapsed = 0
    distance = 0.0
    previous_position = obs.privileged["base_position_world_m"].copy()
    result = None
    patrol_episodes = []
    start = time.monotonic()
    for _ in range(args.steps):
        if not app.is_running():
            break
        if teacher:
            controller.set_velocity(teacher.command(obs))
        command = controller.act(obs)
        result = env.step(command, render=not args.headless)
        next_obs = result.observation
        position = next_obs.privileged["base_position_world_m"]
        distance += float(np.linalg.norm((position - previous_position)[:2]))
        previous_position = position.copy()
        row = {
            "episode_id": obs.episode_id,
            "step": obs.step,
            "time_s": obs.simulation_time_s,
            "joint_position_rad": obs.joint_position_rad,
            "joint_velocity_rad_s": obs.joint_velocity_rad_s,
            "orientation_xyzw": obs.orientation_xyzw,
            "angular_velocity_body_rad_s": obs.angular_velocity_body_rad_s,
            "projected_gravity_body": obs.projected_gravity_body,
            "requested_position_rad": result.requested_command.position_rad,
            "requested_velocity_rad_s": result.requested_command.velocity_rad_s,
            "requested_feedforward_torque_nm": result.requested_command.feedforward_torque_nm,
            "applied_position_rad": result.applied_command.position_rad,
            "applied_velocity_rad_s": result.applied_command.velocity_rad_s,
            "applied_feedforward_torque_nm": result.applied_command.feedforward_torque_nm,
            "next_joint_position_rad": next_obs.joint_position_rad,
            "next_joint_velocity_rad_s": next_obs.joint_velocity_rad_s,
            "next_joint_torque_nm": next_obs.joint_torque_nm,
            "next_base_position_world_m": position,
            "next_base_linear_velocity_body_m_s": next_obs.privileged["base_linear_velocity_body_m_s"],
            "terminated": result.terminated,
            "truncated": result.truncated,
        }
        if hasattr(controller, "velocity_command"):
            row["velocity_command_requested"] = controller.velocity_command.requested
            row["velocity_command_applied"] = controller.velocity_command.applied
        for key, value in row.items():
            rows.setdefault(key, []).append(value)
        elapsed += 1
        falls += int(result.terminated)
        min_height = min(min_height, result.info["base_height_m"])
        if elapsed % 100 == 0:
            print(f"[STEP] {elapsed}/{args.steps}, height={result.info['base_height_m']:.3f} m", flush=True)
        obs = next_obs
        if result.terminated or result.truncated:
            print(f"[EPISODE] {result.info['termination_reason']}", flush=True)
            if not args.reset_on_done:
                break
            seed = args.seed + obs.episode_id + 1
            if teacher:
                patrol_episodes.append(teacher.summary)
                teacher.reset()
            obs = env.reset(seed=seed, spawn_xy_yaw=teacher.spawn(seed) if teacher else None)
            previous_position = obs.privileged["base_position_world_m"].copy()
            controller.reset()
        if args.real_time:
            wait = start + elapsed * config.control_dt - time.monotonic()
            if wait > 0:
                time.sleep(wait)
    np.savez_compressed(output / "trajectory.npz", **{k: np.asarray(v) for k, v in rows.items()})
    summary = {
        "requested_steps": args.steps,
        "completed_steps": elapsed,
        "falls": falls,
        "simulated_seconds": elapsed * config.control_dt,
        "wall_seconds": time.monotonic() - start,
        "distance_xy_m": distance,
        "minimum_base_height_m": min_height if elapsed else None,
        "final_base_position_world_m": (result.observation if result else obs)
        .privileged["base_position_world_m"]
        .tolist(),
        "joint_count": len(env.robot.joint_names),
        "termination_reason": result.info["termination_reason"] if result else "app_closed",
        "patrol_episodes": [*patrol_episodes, teacher.summary] if teacher else [],
    }
    if teacher:
        from g1_e2e.data.quality import patrol_quality

        summary["patrol_quality"] = patrol_quality(summary, env.office)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


if __name__ == "__main__":
    main()
