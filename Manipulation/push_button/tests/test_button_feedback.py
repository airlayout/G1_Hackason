"""細い輪の増光と、点灯・未点灯・判定不能の区別を検査する。"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from button_feedback import LampCheck, LampFrame, lamp_level


def ring_image(level):
    image = np.full((80, 80, 3), 70, dtype=np.uint8)
    yy, xx = np.mgrid[:80, :80]
    distance = np.hypot(xx+0.5-40, yy+0.5-40)
    image[(distance >= 18) & (distance <= 21)] = level
    return image


def frame(level, t, frame_id):
    return LampFrame(np.full((30, 30, 3), level, dtype=np.uint8), (5, 5, 25, 25), t, frame_id)


def test_thin_ring_is_visible_in_percentile_without_whole_box_average():
    before, after = ring_image(100), ring_image(220)
    assert after[20:60, 20:60].mean()-before[20:60, 20:60].mean() < 30
    assert lamp_level(after, (20, 20, 60, 60))-lamp_level(before, (20, 20, 60, 60)) > 100
    np.testing.assert_allclose(lamp_level(after, (20, 20, 60, 60)),
                               lamp_level(after[:, :, ::-1], (20, 20, 60, 60)))


@pytest.mark.parametrize("levels, expected", [([130]*3, True), ([129]*3, False),
                                              ([150, 100, 150], None)])
def test_requires_three_consistent_distinct_post_retreat_frames(levels, expected):
    check = LampCheck(100, 1.0)
    assert not check.observe(frame(200, 1.0, 1), 1.0)
    for i, level in enumerate(levels):
        stamp = 1.1+i*0.1
        assert check.observe(frame(level, stamp, i+2), stamp)
        if i < 2:
            assert check.result is None and not check.finished
    assert check.finished and check.result is expected


def test_duplicate_and_reordered_ids_do_not_count():
    check = LampCheck(100, 1.0)
    assert check.observe(frame(200, 1.1, 10), 1.1)
    assert not check.observe(frame(200, 1.1, 11), 1.2)
    assert not check.observe(frame(200, 1.2, 10), 1.2)
    assert len(check.frames) == 1 and check.result is None


@pytest.mark.parametrize("t, now", [(1.1, 1.7), (1.2, 1.1), (np.nan, 1.0)])
def test_invalid_and_stale_images_are_rejected(t, now):
    with pytest.raises(ValueError, match="時刻"):
        LampCheck(100, 1.0).observe(frame(200, t, 1), now)


@pytest.mark.parametrize("bbox", [(-1, 1, 10, 10), (1, 1, 50, 50), (1, 1, 2, 2),
                                  (1, 1, np.nan, 10)])
def test_invalid_roi_cannot_confirm_lamp(bbox):
    with pytest.raises(ValueError):
        lamp_level(np.zeros((30, 30, 3)), bbox)


@pytest.mark.parametrize("rise", [0, -1, 256, np.nan])
def test_invalid_threshold_is_rejected(rise):
    with pytest.raises(ValueError):
        LampCheck(100, 1.0, rise)


def test_lab_elevator_photos_preserve_lit_ring_and_reflection_limit():
    cv2 = pytest.importorskip("cv2")
    root = Path(__file__).parent / "data/lab_elevator"
    manifest = json.loads((root / "manifest.json").read_text())
    levels = {"lit": [], "unlit": [], "glint": []}
    for case in manifest["cases"]:
        image = cv2.imread(str(root / case["file"]))
        assert image is not None, case["file"]
        levels[case["kind"]].append(lamp_level(image, case["bbox"]))
    assert min(levels["lit"]) >= 220
    assert max(levels["unlit"]) <= 140
    assert min(levels["lit"])-max(levels["unlit"]) >= 60
    # 金属面の反射が強い基準画像では、点灯しても+30に届かないことがある。
    # 絶対輝度での救済は今回の仕様に含めず、この限界を固定する。
    assert min(levels["lit"])-max(levels["glint"]) < 30
