"""URDFの取付変換と実機RGB校正から、同じ光線・画角が得られることを検証する。"""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import yaml

from g1_e2e.data.config import MountConfig, SensorConfig
from g1_e2e.data.geometry import MOUNT_FROM_OPTICAL, lidar_pattern, mount_matrix

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def urdf(tmp_path):
    # 固定版unitree_ros g1_29dof_rev_1_0.urdfのセンサーjointを抜粋。
    path = tmp_path / "robot.urdf"
    path.write_text("""<robot name="sensor_fixture">
      <joint name="imu_in_pelvis_joint" type="fixed">
        <parent link="pelvis"/><child link="imu_in_pelvis"/>
        <origin xyz="0.04525 0 -0.08339" rpy="0 0 0"/>
      </joint>
      <joint name="imu_in_torso_joint" type="fixed">
        <parent link="torso_link"/><child link="imu_in_torso"/>
        <origin xyz="-0.03959 -0.00224 0.14792" rpy="0 0 0"/>
      </joint>
      <joint name="d435_joint" type="fixed">
        <parent link="torso_link"/><child link="d435_link"/>
        <origin xyz="0.0576235 0.01753 0.42987" rpy="0 0.8307767239493009 0"/>
      </joint>
      <joint name="mid360_joint" type="fixed">
        <parent link="torso_link"/><child link="mid360_link"/>
        <origin xyz="0.0002835 0.00003 0.428434" rpy="3.141592653589793 0.05112069379091391 0"/>
      </joint>
      <joint name="moving_joint" type="revolute"><parent link="torso_link"/></joint>
    </robot>""")
    return path


def standard(urdf):
    return SensorConfig.load(ROOT / "configs/sensors/g1_rgb_lidar.yaml", robot_urdf=urdf)


def test_camera_optical_axis_points_down_at_urdf_angle(urdf):
    camera = standard(urdf).camera
    transform = mount_matrix(camera.mount.position_m, camera.mount.rpy_deg) @ MOUNT_FROM_OPTICAL
    np.testing.assert_allclose(transform[:3, 3], [0.0576235, 0.01753, 0.42987])
    forward = transform[:3, 2]
    np.testing.assert_allclose(np.rad2deg(np.arctan2(-forward[2], forward[0])), 47.6, atol=1e-5)
    assert camera.mount.body == "torso_link"


def test_lidar_up_axis_is_inverted_and_sensor_fov_is_preserved(urdf):
    lidar = standard(urdf).lidar
    transform = mount_matrix(lidar.mount.position_m, lidar.mount.rpy_deg)
    np.testing.assert_allclose(transform[:3, 3], [0.0002835, 0.00003, 0.428434])
    np.testing.assert_allclose(lidar.mount.rpy_deg, [180, 2.929, 0])
    assert transform[2, 2] < -0.99
    assert tuple(lidar.vertical_fov_deg) == (-7, 52)
    rays, _ = lidar_pattern(lidar.channels, lidar.columns, lidar.vertical_fov_deg)
    angles = np.rad2deg(np.arcsin(rays[:, 2]))
    np.testing.assert_allclose([angles.min(), angles.max()], [-7, 52], atol=1e-5)
    # センサーの上方向の光線が、機体では床へ向く（roll 180°の取り落としを検出）。
    ray = np.array([np.cos(np.deg2rad(52)), 0, np.sin(np.deg2rad(52))])
    assert (transform[:3, :3] @ ray)[2] < -0.8


def test_rgb_uses_measured_color_intrinsics_and_resizing_keeps_full_fov(urdf):
    camera = standard(urdf).camera
    row = next(
        line
        for line in (ROOT / "G1_IO_SPECIFICATION.md").read_text().splitlines()
        if line.startswith("| stream.color | format.rgb8 |")
    )
    fx, fy, cx, cy = map(float, row.split("|")[3:7])
    np.testing.assert_allclose(camera.intrinsics, [[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    assert (camera.width, camera.height) == (640, 480)
    assert standard(urdf).periods_in_steps(1 / 600) == (20, 60)
    assert 55 < camera.fov_deg[0] < 56 and 43 < camera.fov_deg[1] < 44
    smaller = replace(camera, width=128, height=128)
    np.testing.assert_allclose(smaller.fov_deg, camera.fov_deg, atol=1e-5)
    np.testing.assert_allclose(smaller.intrinsics[0, 0], fx * 0.2)
    np.testing.assert_allclose(smaller.intrinsics[1, 1], fy * (128 / 480))
    np.testing.assert_allclose(smaller.intrinsics[0, 2], (cx + 0.5) * 0.2 - 0.5)
    np.testing.assert_allclose(smaller.intrinsics[1, 2], (cy + 0.5) * (128 / 480) - 0.5)
    assert "horizontal_fov_deg" not in standard(urdf).to_dict()["camera"]


def test_mounts_follow_the_selected_urdf_instead_of_cached_numbers(urdf):
    urdf.write_text(urdf.read_text().replace("0.0576235", "0.123").replace("0.04525", "0.06789"))
    assert standard(urdf).camera.mount.position_m[0] == 0.123
    assert standard(urdf).body_imus.mounts["pelvis"].position_m[0] == 0.06789
    assert standard(urdf).body_imus.mounts["torso"].body == "torso_link"
    assert standard(urdf).body_imus.primary == "pelvis"
    assert standard(urdf).body_imus.mounts["lidar"] == standard(urdf).lidar.mount
    with pytest.raises(ValueError, match="fixed joint"):
        MountConfig.from_urdf(urdf, "moving_joint")
    with pytest.raises(ValueError, match="fixed joint"):
        MountConfig.from_urdf(urdf, "missing")


@pytest.mark.parametrize("conflict", ["mount", "pinhole"])
def test_ambiguous_configuration_is_rejected(urdf, tmp_path, conflict):
    data = yaml.safe_load((ROOT / "configs/sensors/g1_rgb_lidar.yaml").read_text())
    if conflict == "mount":
        data["camera"]["mount"]["position_m"] = [0, 0, 0]
    else:
        data["camera"]["horizontal_fov_deg"] = 80
    path = tmp_path / "ambiguous.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="併用できません"):
        SensorConfig.load(path, robot_urdf=urdf)
