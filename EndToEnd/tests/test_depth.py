"""公開depth校正の基線・画角・Z16量子化・RGB投影を検証する。"""

from pathlib import Path

import numpy as np
import pytest

from g1_e2e.data.config import SensorConfig
from g1_e2e.data.depth import align_depth, quantize_depth
from g1_e2e.robot import RobotSpec
from sim.envs.config import FlatConfig


def test_z16_scale_invalid_and_representable_range():
    z16, valid = quantize_depth(
        np.array([[1.2344, 1.2346, np.inf, 70.0]]), np.array([[True, True, False, True]]), 0.001
    )
    np.testing.assert_array_equal(z16, [[1234, 1235, 0, 0]])
    np.testing.assert_array_equal(valid, [[True, True, False, False]])


def test_depth_projection_uses_baseline_and_nearest_surface():
    transform = np.eye(4)
    transform[0, 3] = 0.01
    k = np.array([[100, 0, 0], [0, 100, 0], [0, 0, 1]])
    # 1mの点は1pixel右へ投影。0.5mの点は2pixel右へ投影され、同じpixelで前面を選ぶ。
    depth = np.zeros((3, 3), np.uint16)
    depth[1, 1:3] = [500, 1000]
    aligned, valid = align_depth(
        depth,
        depth > 0,
        depth_intrinsics=k,
        color_intrinsics=k,
        color_from_depth=transform,
        scale_m=0.001,
        color_shape=(3, 6),
    )
    expected = np.zeros((3, 6))
    expected[1:3, 3:5] = 0.5
    np.testing.assert_allclose(aligned, expected)
    np.testing.assert_array_equal(valid, expected > 0)


def test_standard_profiles_match_recorded_depth_calibration_and_physics_clock():
    root = Path(__file__).resolve().parents[1]
    robot = RobotSpec.load(root / "configs/robot/g1_29dof.yaml")
    if not (root / robot.urdf).exists():
        pytest.skip("setup.shで公式URDFを取得してください")
    sensors = SensorConfig.load(root / "configs/sensors/g1_rgbd_lidar.yaml", robot_urdf=root / robot.urdf)
    task = FlatConfig.load(root / "configs/task/sensor_collection.yaml")
    assert sensors.periods_in_steps(task.physics_dt) == (20, 60)
    assert 1 / sensors.camera.period_s == 30
    depth = sensors.camera.depth
    assert depth is not None
    np.testing.assert_allclose(depth.color_from_depth[:3, 3], [0.0149587858, -0.000382615894, 0.000263101188])
    row = next(
        line
        for line in (root / "G1_IO_SPECIFICATION.md").read_text().splitlines()
        if line.startswith("| stream.depth | format.z16 |")
    )
    fx, fy, cx, cy = map(float, row.split("|")[3:7])
    np.testing.assert_allclose(depth.pinhole.resized(640, 480), [[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
