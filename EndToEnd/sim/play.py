"""G1平地環境に任意のControllerを接続する実行入口。import時にはアプリを起動しない。"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib
import json
import os
from pathlib import Path
import subprocess
import time


def git_output(path: Path, *args: str) -> str:
    """再現性に必要なGit情報を取得する。"""
    result = subprocess.run(["git", "-C", str(path), *args], capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def main() -> None:
    """引数の検証後にIsaacを1回だけ起動する。"""
    from isaaclab.app import AppLauncher

    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", default="hold", help="hold / unitree / module:create_controller")
    parser.add_argument("--controller-options", type=Path, help="任意モデル用のJSON設定")
    parser.add_argument("--robot-config", type=Path, default=root / "configs/robot/g1_29dof.yaml")
    parser.add_argument("--env-config", type=Path, default=root / "configs/task/flat.yaml")
    parser.add_argument(
        "--velocity", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("VX", "VY", "WZ")
    )
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--fixed-base", action="store_true", help="基部を固定して関節指令だけを検証")
    parser.add_argument("--reset-on-done", action="store_true", help="転倒・時間制限後にresetして続行")
    parser.add_argument("--real-time", action="store_true", help="実時間に合わせて表示")
    parser.add_argument("--output", type=Path, help="ログ保存先。既存の出力ファイルは上書きしません")
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device=f"cuda:{os.environ.get('G1_E2E_SELECTED_GPU', '0')}")
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--stepsは1以上を指定してください")
    output = args.output or root / "runs" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output.mkdir(parents=True, exist_ok=True)
    if any((output / name).exists() for name in ["summary.json", "trajectory.npz", "provenance.json"]):
        parser.error(f"出力先は使用済みです: {output}")

    from g1_e2e.robot import RobotSpec

    spec = RobotSpec.load(args.robot_config)
    aliases = {
        "hold": "g1_e2e.inference.hold:create_controller",
        "unitree": "g1_e2e.inference.unitree_onnx:create_controller",
    }
    controller_path = aliases.get(args.controller, args.controller)
    if ":" not in controller_path:
        parser.error("--controllerはmodule:factory形式で指定してください")
    module_name, factory_name = controller_path.split(":", 1)
    options = json.loads(args.controller_options.read_text()) if args.controller_options else {}
    options.update(root=str(root))
    if args.controller == "unitree":
        options["velocity"] = args.velocity
        if not (root / "assets/cache/demo/policy.onnx").is_file():
            parser.error("先にbash EndToEnd/setup.sh --with-demoを実行してください")
    args.kit_args = " ".join(
        filter(
            None,
            [
                f"--portable-root {root / 'assets/cache/kit'} --reset-user",
                "--/app/extensions/registryEnabled=false",
                os.environ.get("G1_E2E_KIT_ARGS", ""),
                args.kit_args,
            ],
        )
    )
    gpu_index = (
        int(args.device.split(":")[1])
        if ":" in args.device
        else int(os.environ.get("G1_E2E_SELECTED_GPU", "0"))
    )
    app = AppLauncher(args, active_gpu=gpu_index, multi_gpu=False).app

    # Isaac依存モジュールは必ずAppLauncherの起動後に読み込む。
    import numpy as np
    import torch

    from sim.envs.flat import FlatConfig, G1FlatEnv

    config = FlatConfig.load(args.env_config)
    factory = getattr(importlib.import_module(module_name), factory_name)
    controller = factory(robot=spec, control_dt=config.control_dt, options=options)
    env = G1FlatEnv(root, spec, config, device=args.device, fixed_base=args.fixed_base)
    isaac_path = Path(os.environ.get("ISAAC_SIM_PATH", "/opt/isaac-sim"))
    lab_path = Path(os.environ.get("ISAACLAB_PATH", root / "../_local/IsaacLab"))
    provenance = {
        "code_commit": git_output(root, "rev-parse", "HEAD"),
        "code_dirty": bool(git_output(root, "status", "--porcelain")),
        "code_diff_sha256": hashlib.sha256(git_output(root, "diff", "HEAD").encode()).hexdigest(),
        "isaac_sim_version": (isaac_path / "VERSION").read_text().strip(),
        "isaac_lab_commit": git_output(lab_path, "rev-parse", "HEAD"),
        "torch_version": torch.__version__,
        "device": args.device,
        "cuda_device_name": torch.cuda.get_device_name(args.device)
        if args.device.startswith("cuda")
        else None,
        "robot_config": args.robot_config.read_text(),
        "env_config": args.env_config.read_text(),
        "assets_manifest": json.loads((root / "assets/manifest.json").read_text()),
        "controller": controller_path,
        "controller_options": options,
        "seed": args.seed,
        "fixed_base": args.fixed_base,
    }
    # 新規・未commitのソースもhashを残す。キャッシュ・重み・ログは対象外。
    source_paths = [root / name for name in ["pyproject.toml", "uv.lock", "run.sh", "setup.sh"]]
    for directory in ["src", "sim", "scripts", "configs", "examples"]:
        source_paths.extend(
            path
            for path in (root / directory).rglob("*")
            if path.is_file() and "__pycache__" not in path.parts
        )
    provenance["source_sha256"] = {
        str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(source_paths)
    }
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    obs = env.reset(seed=args.seed)
    controller.reset()
    rows: dict[str, list] = {}
    falls = 0
    min_height = float("inf")
    elapsed = 0
    distance = 0.0
    previous_position = obs.privileged["base_position_world_m"].copy()
    result = None
    start = time.monotonic()
    for _ in range(args.steps):
        if not app.is_running():
            break
        command = controller.act(obs)
        result = env.step(command)
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
            obs = env.reset(seed=args.seed + obs.episode_id + 1)
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
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"[RESULT] {json.dumps(summary)}", flush=True)
    print(f"[SAVED] {output}", flush=True)
    env.close()
    app.close()
    if falls or elapsed != args.steps:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
