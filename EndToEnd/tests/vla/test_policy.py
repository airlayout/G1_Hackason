import base64
import io
import json

import numpy as np
from PIL import Image
import pytest
import yaml

from g1_e2e.vla.config import ROOT, load_config
from g1_e2e.vla.kinematics import WristKinematics
from g1_e2e.vla.policy import ObservationAdapter, arm_chunk


def make_observation(color=(255, 0, 0)):
    buffer = io.BytesIO()
    Image.new("RGB", (64, 48), color).save(buffer, format="JPEG")
    return {"q": np.zeros(29).tolist(), "jpeg": base64.b64encode(buffer.getvalue()).decode()}


def test_fk_named_joint_order_and_rotation_rows(tmp_path):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("""<robot name="test"><link name="base"/><link name="wrist"/>
        <joint name="arm" type="revolute"><parent link="base"/><child link="wrist"/>
        <origin xyz="1 2 3" rpy="0 0 0"/><axis xyz="0 0 1"/></joint></robot>""")
    names = [f"unused_{i}" for i in range(29)]
    names[17] = "arm"
    q = np.zeros(29)
    q[17] = np.pi / 2
    pose = WristKinematics(urdf, names, "base", ["wrist"]).poses(q)[0]
    np.testing.assert_allclose(pose, [1, 2, 3, 0, -1, 0, 1, 0, 0], atol=1e-7)


@pytest.fixture
def adapter():
    config = load_config()
    if not (ROOT / config["urdf"]).exists():
        pytest.skip("公式URDFはasset準備後に検証します")
    names = yaml.safe_load((ROOT / "configs/robot/g1_29dof.yaml").read_text())["joint_names"]
    return ObservationAdapter(config, names)


def test_adapter_rgb_history_state_shapes_and_no_invented_hand_control(adapter):
    past = make_observation((255, 0, 0))
    current = make_observation((0, 0, 255))
    adapter.record(past, now=1)
    adapter.record(current, now=1 + 20 / 30)
    observation = adapter.build(current, "Press the button.", now=1 + 20 / 30)
    frames = observation["video"]["ego_view"]
    assert frames.shape == (1, 2, 480, 640, 3)
    assert frames.dtype == np.uint8
    assert frames[0, 0, 0, 0, 0] > 250 and frames[0, 1, 0, 0, 2] > 250
    assert observation["state"]["left_arm"].shape == (1, 1, 7)
    assert observation["state"]["waist"].shape == (1, 1, 3)
    assert observation["state"]["left_wrist_eef_9d"].shape == (1, 1, 9)
    assert observation["language"]["annotation.human.task_description"] == [["Press the button."]]


def test_actions_are_absolute_joint_radians_and_only_arms_are_extracted():
    actions = {
        "left_arm": np.full((1, 40, 7), 0.2),
        "right_arm": np.full((1, 40, 7), -0.3),
        "waist": np.full((1, 40, 3), 999),
        "navigate_command": np.full((1, 40, 3), 999),
    }
    chunk = arm_chunk(actions, 8)
    assert chunk.shape == (8, 14)
    np.testing.assert_allclose(chunk[:, :7], 0.2)
    np.testing.assert_allclose(chunk[:, 7:], -0.3)


@pytest.mark.parametrize("shape", [(1, 40, 6), (40, 7), (2, 40, 7), (1, 2, 7)])
def test_incompatible_action_head_fails_instead_of_padding(shape):
    with pytest.raises(ValueError):
        arm_chunk({"left_arm": np.zeros(shape), "right_arm": np.zeros(shape)}, 8)


def test_bad_config_fails_before_connection(tmp_path):
    config = load_config()
    config["arm"]["max_velocity_rad_s"] = float("nan")
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="max_velocity"):
        load_config(path)
