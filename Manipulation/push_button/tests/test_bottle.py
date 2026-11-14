"""ボトルのRGB-D幾何、対象選択、鮮度、押す距離の分離を検証する。"""

from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bottle import (BottleFrame, BottleLocalizer, BottleSettings, estimate_bottle_target,
                    validate_bottle_target, validate_push_distance)
from alignment import ContinuousNormalController
from task_cli import dispatch
from trajectory import validate_button_stroke
from vision import CameraIntrinsics


SETTINGS = BottleSettings(.75)
INTRINSICS = CameraIntrinsics(300., 300., 160., 120.)
TRANSFORM = np.array([[0., 0., 1., 0.], [-1., 0., 0., -.25], [0., -1., 0., .90], [0., 0., 0., 1.]])


def cylinder_image(x=.37, y=-.25):
    """既知の円筒への光線交差。台面・遠方の壁も同じ深度画像に入れる。"""
    vv, uu = np.mgrid[:240, :320]
    dx = np.ones_like(uu)
    dy = -(uu-160)/300.
    dz = -(vv-120)/300.
    oy = -.25-y
    a = dx*dx+dy*dy
    b = 2*(-x+oy*dy)
    c = x*x+oy*oy-.034**2
    discriminant = b*b-4*a*c
    t = (-b-np.sqrt(np.maximum(0, discriminant)))/(2*a)
    z = .90+t*dz
    mask = (discriminant > 0) & (t > 0) & (z > .75) & (z < 1.01)
    background = np.full(uu.shape, 1.2)
    table = np.divide(.75-.90, dz, out=np.full_like(dz, np.inf), where=dz < 0)
    depth = np.minimum(background, table)
    visible = mask & (t < depth)
    depth[visible] = t[visible]
    ys, xs = np.where(visible)
    box = (float(xs.min()), float(ys.min()), float(xs.max()+1), float(ys.max()+1))
    return depth, box


def target_at(x=.37, y=-.25):
    depth, box = cylinder_image(x, y)
    return estimate_bottle_target(depth, box, INTRINSICS, TRANSFORM, SETTINGS)


def test_metric_curved_surface_separates_table_and_background():
    target = target_at()
    assert target.center_xy == pytest.approx((.37, -.25), abs=.001)
    assert target.radius_m == pytest.approx(.034, abs=.001)
    assert target.face_xyz == pytest.approx((.336, -.25, .83), abs=.001)
    validate_bottle_target(target, SETTINGS)


def test_missing_or_planar_background_depth_does_not_become_bottle():
    depth, box = cylinder_image()
    with pytest.raises(ValueError, match="深度"):
        estimate_bottle_target(np.zeros_like(depth), box, INTRINSICS, TRANSFORM, SETTINGS)
    with pytest.raises(ValueError, match="曲面"):
        estimate_bottle_target(np.full_like(depth, .37), box, INTRINSICS, TRANSFORM, SETTINGS)


@pytest.mark.parametrize("transform", [np.zeros((4, 4)), np.eye(3), np.diag([1, 1, -1, 1])])
def test_rejects_invalid_calibration(transform):
    depth, box = cylinder_image()
    with pytest.raises(ValueError, match="校正"):
        estimate_bottle_target(depth, box, INTRINSICS, transform, SETTINGS)


def test_table_edge_and_workspace_are_checked_for_full_travel():
    target = target_at()
    with pytest.raises(ValueError, match="台の余裕"):
        validate_bottle_target(target, replace(SETTINGS, table_bounds=(.2, .43, -.6, .02)))
    with pytest.raises(ValueError, match="作業範囲"):
        validate_bottle_target(replace(target, face_xyz=(.7, -.25, .83)), SETTINGS)


def test_bottle_travel_does_not_expand_button_stroke_limits():
    assert validate_push_distance(.1) == .1
    with pytest.raises(ValueError, match="stroke"):
        validate_button_stroke(.05)
    for distance in (0, -.01, .101, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="push-distance"):
            validate_push_distance(distance)
    face = np.array([.336, -.25, .83])
    servo = ContinuousNormalController(face, np.array([1, 0, 0]), .015, .05,
                                       face-[.065, 0, 0], travel_validator=validate_push_distance)
    assert servo.stroke == .05
    with pytest.raises(ValueError, match="stroke"):
        ContinuousNormalController(face, np.array([1, 0, 0]), .015, .05, face-[.065, 0, 0])


class Camera:
    def __init__(self):
        self.t, self.sequence = 0., 0
        self.x = .37
        self.stale = self.duplicate = self.missing = False

    def now(self):
        return self.t

    def read(self, after, deadline):
        self.t += .1
        self.sequence += 0 if self.duplicate else 1
        depth, self.box = cylinder_image(self.x)
        return BottleFrame(np.zeros((240, 320, 3), np.uint8), depth, INTRINSICS, TRANSFORM,
                           self.t-1 if self.stale else self.t, self.sequence)

    def detect(self, image):
        if self.missing:
            return []
        return [SimpleNamespace(class_name="bottle", bbox=self.box)]

    def localizer(self, target_pixel=None):
        return BottleLocalizer(self.read, self, SETTINGS, self.now, target_pixel)


def test_three_new_frames_per_measurement_and_moved_target_association():
    camera = Camera()
    localizer = camera.localizer()
    first = localizer.observe()
    assert first.sequence == 3
    second = localizer.observe()
    assert second.sequence == 6
    camera.x += .05
    # 移動前の位置との25mmゲートを使い続けると再計測できない。
    localizer.expect_translation()
    moved = localizer.observe()
    assert moved.sequence == 9
    assert moved.target.center_xy[0]-first.target.center_xy[0] == pytest.approx(.05, abs=.001)


@pytest.mark.parametrize("failure", ["stale", "duplicate", "missing"])
def test_stale_duplicate_or_lost_frames_never_complete_measurement(failure):
    camera = Camera()
    localizer = camera.localizer()
    localizer.observe()
    setattr(camera, failure, True)
    with pytest.raises(RuntimeError, match="3フレーム"):
        localizer.observe(timeout_s=.6)


def test_detection_that_finishes_too_late_is_rejected():
    camera = Camera()
    detector = camera.detect

    def slow(image):
        camera.t += .6
        return detector(image)

    localizer = camera.localizer()
    localizer.detector = SimpleNamespace(detect=slow)
    with pytest.raises(RuntimeError, match="3フレーム"):
        localizer.observe(timeout_s=1.)


def test_multiple_targets_are_ambiguous_and_pixel_can_select_one():
    camera = Camera()
    original = camera.detect

    def duplicate(image):
        return original(image)*2

    localizer = camera.localizer()
    localizer.detector = SimpleNamespace(detect=duplicate)
    with pytest.raises(ValueError, match="一意"):
        localizer.observe()
    # 指定画素を含まない検出は位置推定前に除外する。
    def distractor(image):
        return original(image)+[SimpleNamespace(class_name="bottle", bbox=(0, 0, 20, 30))]

    localizer = camera.localizer((160, 190))
    localizer.detector = SimpleNamespace(detect=distractor)
    assert localizer.observe().sequence >= 3


def test_wrong_bottle_is_not_selected_at_predicted_destination():
    camera = Camera()
    localizer = camera.localizer()
    localizer.observe()
    localizer.expect_translation()
    camera.x += .10
    with pytest.raises(RuntimeError, match="3フレーム"):
        localizer.observe(timeout_s=.6)


def test_task_switch_preserves_selected_arguments_and_defaults_to_button():
    calls = []
    button = lambda argv: calls.append(("button", argv))
    bottle = lambda argv: calls.append(("bottle", argv))
    dispatch(button, bottle, ["--task", "bottle", "--push-distance", "0.05"])
    dispatch(button, bottle, ["--stroke", "0.0015"])
    assert calls == [("bottle", ["--push-distance", "0.05"]), ("button", ["--stroke", "0.0015"])]
    with pytest.raises(SystemExit):
        dispatch(button, bottle, ["--task", "unknown"])
