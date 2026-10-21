"""実行・収集に共通の引数、controllerの生成、起動設定、再現情報。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import importlib
import json
import os
from pathlib import Path
import subprocess

from g1_e2e.contracts import Controller
from g1_e2e.robot import RobotSpec


@dataclass(frozen=True)
class ControllerConfig:
    factory_path: str
    options: dict

    @classmethod
    def from_args(cls, args: argparse.Namespace, root: Path) -> ControllerConfig:
        aliases = {
            "hold": "g1_e2e.inference.hold:create_controller",
            "unitree": "g1_e2e.inference.unitree_onnx:create_controller",
        }
        path = aliases.get(args.controller, args.controller)
        if len(path.split(":")) != 2 or not all(path.split(":")):
            raise ValueError("--controllerはhold / unitree / module:factory形式です")
        options = json.loads(args.controller_options.read_text()) if args.controller_options else {}
        if not isinstance(options, dict):
            raise ValueError("--controller-optionsはJSON objectです")
        options["root"] = str(root)
        if args.controller == "unitree":
            if not (root / "assets/cache/demo/policy.onnx").is_file():
                raise ValueError("先にbash EndToEnd/setup.sh --with-demoを実行してください")
            options["velocity"] = args.velocity
        return cls(path, options)

    def create(self, robot: RobotSpec, control_dt: float) -> Controller:
        """アプリ起動後にfactoryをimportする。各入口はcontrollerの実装を知らない。"""
        module, name = self.factory_path.split(":")
        factory = getattr(importlib.import_module(module), name)
        return factory(robot=robot, control_dt=control_dt, options=dict(self.options))


def add_run_arguments(
    parser: argparse.ArgumentParser, root: Path, *, controller: str, velocity: tuple[float, float, float]
) -> None:
    parser.add_argument("--controller", default=controller, help="hold / unitree / module:create_controller")
    parser.add_argument("--controller-options", type=Path, help="controller用のJSON設定")
    parser.add_argument("--robot-config", type=Path, default=root / "configs/robot/g1_29dof.yaml")
    parser.add_argument("--env-config", type=Path, default=root / "configs/task/flat.yaml")
    parser.add_argument("--velocity", type=float, nargs=3, default=list(velocity), metavar=("VX", "VY", "WZ"))
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--reset-on-done", action="store_true", help="転倒・時間制限後にresetして続行")
    parser.add_argument("--output", type=Path, help="新しい実行結果の保存先")


def configure_app(args: argparse.Namespace, root: Path, *, capture: bool = False) -> int:
    """Kitの設定を揃え、物理計算と描画で使うGPUを返す。"""
    kit_args = [
        f"--portable-root {root / 'assets/cache/kit'} --reset-user",
        "--/app/extensions/registryEnabled=false",
    ]
    if capture:
        kit_args.extend(["--/app/asyncRendering=false", "--/rtx/hydra/supportMultiTickRate=false"])
    args.kit_args = " ".join(filter(None, [*kit_args, os.environ.get("G1_E2E_KIT_ARGS", ""), args.kit_args]))
    return (
        int(args.device.split(":")[1])
        if ":" in args.device
        else int(os.environ.get("G1_E2E_SELECTED_GPU", "0"))
    )


def _git_output(path: Path, *args: str) -> str:
    result = subprocess.run(["git", "-C", str(path), *args], capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def run_metadata(root: Path, args: argparse.Namespace, controller: ControllerConfig) -> dict:
    """新規・未commitのソースもhashへ含める。キャッシュ・重み・ログは含めない。"""
    import torch

    isaac = Path(os.environ.get("ISAAC_SIM_PATH", "/opt/isaac-sim"))
    lab = Path(os.environ.get("ISAACLAB_PATH", root / "../_local/IsaacLab"))
    files = [root / name for name in ["pyproject.toml", "uv.lock"]]
    files.extend(root.glob("*.sh"))
    for name in ["src", "sim", "scripts", "configs", "examples"]:
        files.extend(
            path for path in (root / name).rglob("*") if path.is_file() and "__pycache__" not in path.parts
        )
    return {
        "code_commit": _git_output(root, "rev-parse", "HEAD"),
        "code_dirty": bool(_git_output(root, "status", "--porcelain")),
        "code_diff_sha256": hashlib.sha256(_git_output(root, "diff", "HEAD").encode()).hexdigest(),
        "source_sha256": {
            str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(files)
        },
        "isaac_sim_version": (isaac / "VERSION").read_text().strip(),
        "isaac_lab_commit": _git_output(lab, "rev-parse", "HEAD"),
        "torch_version": torch.__version__,
        "device": args.device,
        "cuda_device_name": torch.cuda.get_device_name(args.device)
        if args.device.startswith("cuda")
        else None,
        "robot_config_yaml": args.robot_config.read_text(),
        "environment_config_yaml": args.env_config.read_text(),
        "asset_manifest": json.loads((root / "assets/manifest.json").read_text()),
        "controller": controller.factory_path,
        "controller_options": controller.options,
        "seed": args.seed,
    }
