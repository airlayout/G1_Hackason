"""Isaacを起動せず、関節順とモデル接続の共通境界を検証する。"""

from __future__ import annotations

import importlib
from pathlib import Path
import re
import sys

import numpy as np
import pytest

from g1_e2e.contracts import JointCommand
from g1_e2e.robot import RobotSpec

ROOT = Path(__file__).resolve().parents[1]


def test_joint_names_match_hardware_specification() -> None:
    """実機I/O仕様書のSDK indexとの一致を確認する。"""
    text = (ROOT / "G1_IO_SPECIFICATION.md").read_text()
    appendix = text.split("## 付録A.", 1)[1].split("### A.1", 1)[0]
    entries = re.findall(r"^\| (\d+) \| ([a-z_]+_joint) \|", appendix, re.MULTILINE)
    expected = tuple(name for _, name in sorted(entries, key=lambda row: int(row[0])))
    assert len(expected) == 29
    assert RobotSpec.load(ROOT / "configs/robot/g1_29dof.yaml").joint_names == expected


def test_sdk_mapping_round_trip_for_shuffled_simulator_order() -> None:
    """位置と指令が同じ物理関節へ戻ることを、任意の配列順で検証する。"""
    robot = RobotSpec.load(ROOT / "configs/robot/g1_29dof.yaml")
    names = list(np.random.default_rng(31).permutation(robot.joint_names))
    indices = robot.simulator_indices(names)
    values_by_name = {name: float(index) + 0.125 for index, name in enumerate(robot.joint_names)}
    simulator_values = np.array([values_by_name[name] for name in names])
    sdk_values = simulator_values[indices]
    np.testing.assert_allclose(sdk_values, np.arange(29) + 0.125)
    restored = np.zeros(29)
    restored[indices] = sdk_values
    np.testing.assert_array_equal(restored, simulator_values)


def test_extra_hand_joint_is_rejected() -> None:
    robot = RobotSpec.load(ROOT / "configs/robot/g1_29dof.yaml")
    with pytest.raises(ValueError):
        robot.simulator_indices([*robot.joint_names, "left_hand_joint"])


@pytest.mark.parametrize("value", [np.zeros(35), np.zeros((1, 29)), np.full(29, np.nan), np.full(29, np.inf)])
def test_invalid_position_never_crosses_environment_boundary(value: np.ndarray) -> None:
    with pytest.raises(ValueError):
        JointCommand(value).validated()


@pytest.mark.parametrize("field", ["velocity_rad_s", "feedforward_torque_nm"])
def test_invalid_hybrid_command_is_rejected(field: str) -> None:
    with pytest.raises(ValueError):
        JointCommand(np.zeros(29), **{field: np.full(29, np.nan)}).validated()


def test_command_does_not_alias_model_output() -> None:
    output = np.zeros(29, dtype=np.float32)
    command = JointCommand(output).validated()
    output[:] = 2.0
    np.testing.assert_array_equal(command.position_rad, np.zeros(29))


def test_importing_entrypoint_does_not_launch_isaac() -> None:
    before = set(sys.modules)
    importlib.import_module("sim.play")
    assert not any(name.startswith(("isaaclab", "isaacsim", "omni.")) for name in set(sys.modules) - before)
