"""アプリ起動前の設定とcontroller接続を、Isaac・GPUなしで検証する。"""

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from g1_e2e.robot import RobotSpec
from sim.envs.config import FlatConfig
from sim.runtime import ControllerConfig, add_run_arguments, configure_app, run_metadata

ROOT = Path(__file__).resolve().parents[1]


def test_custom_controller_receives_options_and_produces_sdk_order_command(tmp_path):
    settings = tmp_path / "controller.json"
    settings.write_text(json.dumps({"amplitude_rad": 0.1}))
    parser = argparse.ArgumentParser()
    add_run_arguments(parser, ROOT, controller="hold", velocity=(0, 0, 0))
    args = parser.parse_args(
        ["--controller", "examples.arm_sweep:create_controller", "--controller-options", str(settings)]
    )
    config = ControllerConfig.from_args(args, ROOT)
    robot = RobotSpec.load(args.robot_config)
    controller = config.create(robot, FlatConfig.load(args.env_config).control_dt)
    controller.reset()
    position = controller.act(SimpleNamespace(simulation_time_s=0.5)).validated().position_rad
    expected = robot.default_position.copy()
    expected[robot.joint_names.index("left_shoulder_pitch_joint")] += 0.1
    np.testing.assert_allclose(position, expected)
    assert config.options["root"] == str(ROOT)


@pytest.mark.parametrize("controller", ["unknown", "module:", ":factory", "module:a:b", "unitree"])
def test_invalid_or_unavailable_controller_is_rejected_before_launch(tmp_path, controller):
    args = argparse.Namespace(controller=controller, controller_options=None, velocity=[0.3, 0, 0])
    with pytest.raises(ValueError):
        ControllerConfig.from_args(args, tmp_path)


@pytest.mark.parametrize("capture", [False, True])
def test_app_configuration_preserves_selected_gpu_and_user_kit_options(monkeypatch, capture):
    monkeypatch.setenv("G1_E2E_SELECTED_GPU", "1")
    monkeypatch.setenv("G1_E2E_KIT_ARGS", "--/test/env=true")
    args = argparse.Namespace(device="cuda:2", kit_args="--/test/cli=true")
    assert configure_app(args, ROOT, capture=capture) == 2
    assert "--/test/env=true" in args.kit_args
    assert "--/test/cli=true" in args.kit_args
    assert ("--/app/asyncRendering=false" in args.kit_args) == capture
    args.device = "cpu"
    args.kit_args = ""
    assert configure_app(args, ROOT) == 1


@pytest.mark.parametrize("value", [0, -0.02, 0.003, 0.021, float("nan")])
def test_invalid_control_clock_is_rejected_without_isaac(value):
    with pytest.raises(ValueError):
        replace(FlatConfig(), control_dt=value)


def test_run_metadata_includes_untracked_sources_and_excludes_run_artifacts(tmp_path, monkeypatch):
    pytest.importorskip("torch")
    for name in ["pyproject.toml", "uv.lock", "run.sh", "collect.sh", "setup.sh"]:
        (tmp_path / name).write_text(name)
    source = tmp_path / "src/g1_e2e/untracked.py"
    source.parent.mkdir(parents=True)
    source.write_text("uncommitted source")
    artifacts = tmp_path / "runs"
    artifacts.mkdir()
    (artifacts / "ignored.py").write_text("generated")
    config = tmp_path / "configs/robot.yaml"
    config.parent.mkdir()
    config.write_text("robot config")
    assets = tmp_path / "assets"
    assets.mkdir()
    (assets / "manifest.json").write_text("{}")
    isaac = tmp_path / "isaac"
    isaac.mkdir()
    (isaac / "VERSION").write_text("6.2.0")
    monkeypatch.setenv("ISAAC_SIM_PATH", str(isaac))
    monkeypatch.setenv("ISAACLAB_PATH", str(tmp_path / "lab"))
    args = argparse.Namespace(device="cpu", robot_config=config, env_config=config, seed=7)
    metadata = run_metadata(tmp_path, args, ControllerConfig("module:factory", {}))
    assert (
        metadata["source_sha256"]["src/g1_e2e/untracked.py"]
        == hashlib.sha256(source.read_bytes()).hexdigest()
    )
    assert not any(path.startswith("runs/") for path in metadata["source_sha256"])
    assert metadata["isaac_sim_version"] == "6.2.0"
    assert metadata["cuda_device_name"] is None
