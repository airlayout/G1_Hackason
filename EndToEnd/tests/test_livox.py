"""メーカー走査表、点時計、動き補正、未知のセンサー量を検証する。"""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from g1_e2e.data.livox import LivoxPattern, load_livox_pattern
from sim.sensors.livox import interpolate_poses
from g1_e2e.data.geometry import mount_matrix


def test_vendor_coordinate_conversion_and_finite_table_exhaustion():
    pattern = LivoxPattern([[1, 0, 90], [2, 90, 90], [3, 0, 30]])
    np.testing.assert_allclose(
        pattern.sample([0, 1, 2]), [[1, 0, 0], [0, 1, 0], [0.5, 0, np.sqrt(3) / 2]], atol=1e-7
    )
    assert pattern.duration_s == 3 / 200000
    with pytest.raises(ValueError, match="末尾"):
        pattern.sample([3])
    with pytest.raises(ValueError, match="秒分"):
        pattern.require_duration(0.1)
    cyclic = LivoxPattern([[1, 0, 90]], repeat=True)
    np.testing.assert_allclose(cyclic.sample([0, 1]), [[1, 0, 0], [1, 0, 0]], atol=1e-7)
    assert cyclic.metadata["repeat_pattern"] is True


def test_csv_column_is_not_misread_as_seconds():
    with pytest.raises(ValueError, match="点番号"):
        LivoxPattern([[0.000005, 0, 90], [0.000010, 1, 90]])


def test_per_point_pose_interpolation_matches_known_translation_and_rotation():
    start, end = np.eye(4), mount_matrix((1, 0, 0), (0, 0, 90))
    rotations, positions = interpolate_poses(start, end, [0, 0.5, 1])
    np.testing.assert_allclose(positions, [[0, 0, 0], [0.5, 0, 0], [1, 0, 0]])
    ray = np.einsum("nij,j->ni", rotations, [1, 0, 0])
    np.testing.assert_allclose(ray, [[1, 0, 0], [np.sqrt(0.5), np.sqrt(0.5), 0], [0, 1, 0]], atol=1e-7)
    with pytest.raises(ValueError, match="補間"):
        interpolate_poses(start, end, [1.01])


def test_real_vendor_table_has_changing_patterns_and_published_clock():
    root = Path(__file__).resolve().parents[1]
    if not (root / "assets/cache/sensors/mid360.csv").is_file():
        pytest.skip("setup.shでメーカーCSVを取得してください")
    pattern = load_livox_pattern(root, "assets/cache/sensors/mid360.csv")
    assert len(pattern.table) == 800000 and pattern.duration_s == 4
    first = pattern.sample(np.arange(20000))
    second = pattern.sample(np.arange(20000, 40000))
    assert not np.allclose(first, second)
    np.testing.assert_allclose(np.linalg.norm(first, axis=1), 1, atol=1e-6)
    assert pattern.metadata["csv_elevation_bounds_deg"][0] < -7
    assert pattern.metadata["csv_elevation_bounds_deg"][1] > 52


def test_model_deskews_world_points_and_does_not_invent_ring_or_intensity():
    from g1_e2e.models import ModelConfig, ModelInputAdapter
    from tests.test_data_collection import frames

    root = Path(__file__).resolve().parents[1]
    config = ModelConfig.load(root / "configs/model/recurrent_perceiver.yaml")
    config = replace(config, modalities={"lidar": config.modalities["lidar"]})
    _, frame = frames()
    lidar = dict(frame.__dict__)
    lidar.update(
        points_world_m=np.array([[3, 0, 0], [4, 0, 0], [0, 0, 0]], np.float32),
        ring_valid=np.zeros(3, bool),
        intensity_valid=np.zeros(3, bool),
    )
    sample = {
        "state": {"time_s": 0.1, "episode_id": 0, "base_pose_world": mount_matrix((2, 0, 0), (0, 0, 0))},
        "lidar": lidar,
        "lidar_calibration": {
            "point_time_offset_s": np.array([-0.1, 0, 0], np.float32),
            "ring": np.array([7, 8, 9], np.int32),
        },
    }
    tokens = ModelInputAdapter(config)(sample)["lidar"]
    np.testing.assert_allclose(tokens.features[0, :2, :3].numpy(), [[1, 0, 0], [2, 0, 0]])
    np.testing.assert_allclose(tokens.features[0, :2, 3:5].numpy(), 0)
    np.testing.assert_allclose(tokens.age_s[0, :2].numpy(), [0.2, 0.1])
