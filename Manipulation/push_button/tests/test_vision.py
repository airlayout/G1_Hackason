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


def test_tilted_face_is_intersected_at_bbox_center():
    ys, xs = np.mgrid[:240, :320]
    ray_x = (xs-INTRINSICS.cx)/INTRINSICS.fx
    depth = 1.0/(1+0.2*ray_x)
    depth[100:140, 170:210] = 0.95/(1+0.2*ray_x[100:140, 170:210])
    target = estimate_button_target(depth, BBOX, INTRINSICS, np.eye(4))
    z = 0.95/(1+0.2*0.12)
    assert np.allclose(target.face_xyz, (0.12*z, 0, z), atol=0.0002)


def test_partial_hand_occlusion_is_removed_but_majority_occlusion_is_rejected():
    depth = sample_depth()
    # 中央領域の約3割を、5cm手前の手先で隠す。
    depth[114:127, 183:187] = 0.90
    expected = np.array([0.114, 0, 0.95])
    target = estimate_button_target(depth, BBOX, INTRINSICS, np.eye(4), expected_face_xyz=expected)
    assert np.allclose(target.face_xyz, expected, atol=0.0002)
    assert 0.6 <= target.depth_valid_fraction < 1.0
    depth[113:128, 182:194] = 0.90
    with pytest.raises(ValueError, match="遮蔽"):
        estimate_button_target(depth, BBOX, INTRINSICS, np.eye(4), expected_face_xyz=expected)
