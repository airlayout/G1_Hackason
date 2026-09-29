"""RGB-D 推定が実寸と欠損の扱いを守るかを確かめる。"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision import (ButtonTarget, CameraIntrinsics, estimate_button_target,
                    validate_reachable_target)


INTRINSICS = CameraIntrinsics(250.0, 250.0, 160.0, 120.0)
BBOX = (170.0, 100.0, 210.0, 140.0)


def sample_depth() -> np.ndarray:
    depth = np.ones((240, 320), dtype=np.float32)
    depth[100:140, 170:210] = 0.95  # パネルより5 cm手前のボタン面
    return depth


def test_estimates_metric_face_and_press_direction() -> None:
    target = estimate_button_target(sample_depth(), BBOX, INTRINSICS, np.eye(4))
    assert np.allclose(target.face_xyz, (0.114, 0.0, 0.95), atol=0.002)
    assert np.allclose(target.press_direction, (0.0, 0.0, 1.0), atol=0.002)
    assert target.depth_valid_fraction == 1.0
    assert target.plane_inlier_fraction > 0.95


def test_rejects_missing_button_depth_or_panel() -> None:
    depth = sample_depth()
    depth[100:140, 170:210] = 0
    with pytest.raises(ValueError, match="中央の有効深度"):
        estimate_button_target(depth, BBOX, INTRINSICS, np.eye(4))

    depth = sample_depth()
    depth[:] = 0
    depth[100:140, 170:210] = 0.95
    with pytest.raises(ValueError, match="パネル深度"):
        estimate_button_target(depth, BBOX, INTRINSICS, np.eye(4))


def test_rejects_uncalibrated_transform() -> None:
    bad = np.eye(4)
    bad[0, 0] = 1.4
    with pytest.raises(ValueError, match="回転"):
        estimate_button_target(sample_depth(), BBOX, INTRINSICS, bad)


def test_rejects_wrong_surface_even_with_valid_depth() -> None:
    target = ButtonTarget((0.35, -0.25, 1.0), (0.0, 0.0, -1.0), 0.5, 1.0, 1.0)
    with pytest.raises(ValueError, match="押下方向"):
        validate_reachable_target(target)
