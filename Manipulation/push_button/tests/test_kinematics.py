"""実際のMenagerieモデルで、共有FK/IKとカメラ座標変換を検証する。"""

import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinematics import ArmKinematics, load_robot


@pytest.fixture
def robot():
    path = os.environ.get("G1_MUJOCO_MODEL")
    if not path:
        pytest.skip("G1_MUJOCO_MODEL に外部のMenagerieモデルを指定してください")
    model, rest = load_robot(Path(path), (0.110, -0.003, 0), 0.79)
    kin = ArmKinematics(model)
    kin.update(rest)
    return kin, rest


def test_ik_preserves_measured_state_and_reaches_metric_target(robot):
    kin, _ = robot
    original = kin.data.qpos.copy()
    target = np.array([0.30, -0.25, 1.0])
    result = kin.solve(target, np.array([1, 0, 0]))
    assert np.array_equal(kin.data.qpos, original)
    solved = original.copy()
    solved[kin.qadr] = result.joint_angles
    kin.update(solved)
    assert np.linalg.norm(kin.tip_position()-target) < 0.0001
    with pytest.raises(ValueError, match="収束"):
        kin.solve(np.array([10, 10, 10]), np.array([1, 0, 0]))


def test_common_body_motion_cancels_in_camera_relative_hand_position(robot):
    kin, rest = robot
    kin.update_fixed_base(rest, (0, 0, 0))
    initial = np.linalg.inv(kin.body_transform("torso_link")) @ np.r_[kin.tip_position(), 1]
    moved = rest.copy()
    moved[12] = 0.01
    kin.update_fixed_base(moved, (0.01, -0.01, 0))
    current = np.linalg.inv(kin.body_transform("torso_link")) @ np.r_[kin.tip_position(), 1]
    assert np.allclose(current, initial, atol=1e-8)


def test_swept_path_rejects_panel_penetration(robot):
    kin, _ = robot
    face, normal = np.array([0.28, -0.25, 1.0]), np.array([1, 0, 0])
    result = kin.solve(face+0.02*normal, normal)
    with pytest.raises(ValueError, match="パネル面"):
        kin.validate_path(result.joint_angles, face, normal)
