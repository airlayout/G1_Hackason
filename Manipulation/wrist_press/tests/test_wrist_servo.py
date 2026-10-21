from dataclasses import replace
from types import SimpleNamespace
from threading import Event

import numpy as np
import pytest

from Manipulation.wrist_press.controller import PressController
from Manipulation.wrist_press.perception import LatestPerception, Localizer, NotFound
from Manipulation.wrist_press.planning import PlanningError
from Manipulation.wrist_press.settings import load_settings
from Manipulation.wrist_press.types import CameraFrame, Sample, Target
from Manipulation.wrist_press.wrist_servo import WristTargetFilter, bounded_correction


def target(t, point=(.3, .006, 0), camera="wrist", name="up"):
    return Target("button", name, point, [1, 0, 0], .02, t, camera=camera,
                  wall_point=[.302, .006, 0])


def test_filter_unique_recent_exposures_and_head_surface():
    f = WristTargetFilter()
    head = target(0, camera="head")
    assert f.add(target(.1, (.31, .007, 0)), .1, head) is None
    assert f.add(target(.1, (.31, .007, 0)), .12, head) is None
    assert f.add(target(.2, (.309, .008, 0)), .2, head) is None
    result = f.add(target(.3, (.31, .0075, 0)), .3, head)
    assert result.point == pytest.approx([.3, .0075, 0])
    assert result.camera == "wrist"
    assert result.wall_point == pytest.approx(head.wall_point)
    assert not f.fresh(.46)  # Oldest of all three exposures, not just newest.
    assert not f.items


def test_filter_rejects_disagreement_wrong_target_and_requires_recovery():
    f = WristTargetFilter()
    head = target(0, camera="head")
    for t, y in ((.1, .006), (.2, .006), (.3, .011)):
        assert f.add(target(t, (.3, y, 0)), t, head) is None
    assert f.reason == "wrist positions disagree"
    assert f.add(target(.4, name="down"), .4, head) is None
    assert not f.items
    assert f.add(target(.5), .5, head) is None
    assert f.add(target(.6), .6, head) is None
    assert f.add(target(.7), .7, head) is not None
    assert f.add(target(.8, (.34, .006, 0)), .8, head) is None
    assert not f.items


def test_bounded_gain_has_no_normal_component_or_accumulated_bias():
    assert bounded_correction([0, .004, 0]) == pytest.approx([0, .002, 0])
    assert np.linalg.norm(bounded_correction([0, .1, .1])) == pytest.approx(.003)
    assert bounded_correction([0, 0, 0]) == pytest.approx([0, 0, 0])


def test_wrist_uses_head_association_despite_rotated_arrow_but_rejects_other_button():
    loc = Localizer.__new__(Localizer)
    loc.cfg = load_settings()
    loc.cfg['wrist_servo']['enabled'] = True
    loc.tracks = {}
    candidate = SimpleNamespace(arrow="down", bbox=(45, 45, 55, 55))
    loc._tracked_candidates = lambda frame: [candidate]
    f = CameraFrame(np.zeros((100, 100, 3), np.uint8), np.full((100, 100), .3),
                    np.array([[100, 0, 50], [0, 100, 50], [0, 0, 1]]), .1,
                    np.zeros(29), np.eye(4), camera="wrist")
    previous = Target("button", "up", [0, 0, .3], [0, 0, 1], .02, 0, camera="head")
    assert loc._button(f, "up", previous).name == "up"
    candidate.bbox = (75, 45, 85, 55)  # 90 mm away: another button, despite its label.
    with pytest.raises(NotFound):
        loc._button(f, "up", previous)
    candidate.bbox = (45, 45, 55, 55)
    loc.cfg['wrist_servo']['enabled'] = False
    with pytest.raises(NotFound):
        loc._button(f, "up", previous)


def controller(y=0, detected_y=.012):
    cfg = load_settings()
    cfg["wrist_servo"]["enabled"] = True

    class Robot:
        @staticmethod
        def tip(q):
            return q[22:25].copy()

        @staticmethod
        def solve(point, q, axis):
            out = q.copy()
            out[22:25] = point
            return out

    class Collision:
        allowed = True
        last_reason = "test obstacle"
        calls = 0

        def edge(self, *args):
            self.calls += 1
            return self.allowed

    class Localizer:
        timings = []
        cameras = []
        resets = 0
        missing = False

        def locate(self, frame, name, previous):
            self.cameras.append(frame.camera)
            if self.missing:
                raise NotFound("occluded")
            return target(frame.captured_at, (.3, detected_y, 0))

        def reset_track(self, camera):
            self.resets += 1

    c = PressController(cfg, Robot(), Collision(), Localizer())
    c.reset("up")
    c.phase = "align"
    c.phase_t = c.started = 0
    c.target = c.reference_target = target(0, camera="head")
    c.axis = np.eye(3)
    q = np.zeros(29)
    q[22:25] = [.272, y, 0]
    c.alignment_goal = q.copy()
    return c, q


def sample(t, q, image_t=None, missing=False, speed=0):
    stamp = t if image_t is None else image_t
    image = np.zeros((10, 10, 3), np.uint8)
    f = CameraFrame(image, None, np.eye(3), stamp, q, np.eye(4), camera="wrist")
    dq = np.zeros(29)
    dq[22] = speed
    head = replace(f, camera="head")
    return Sample(q.copy(), dq, t, head, None if missing else f)


def test_controller_corrects_only_on_new_wrist_frames_and_checks_collision():
    c, q = controller()
    try:
        for t in (.1, .2, .3):
            goal = c._align_wrist(sample(t, q))
        assert goal[22:25] == pytest.approx([.272, .003, 0])
        assert c.collision.calls == 1
        assert c.report.corrections == 1
        assert np.all(c.alignment_bias == 0)
        c._align_wrist(sample(.32, q, image_t=.3))
        assert c.report.corrections == 1
        assert set(c.localizer.cameras) == {"wrist"}
        c.collision.allowed = False
        with pytest.raises(PlanningError, match="unsafe wrist"):
            c._align_wrist(sample(.4, q))
    finally:
        c.close()


def test_press_requires_two_post_settle_exposures_and_low_speed():
    c, q = controller(y=.006, detected_y=.006)
    try:
        for t in (.1, .2, .3):
            c._align_wrist(sample(t, q))
        c._align_wrist(sample(.32, q, image_t=.3))
        assert c.wrist_confirmations == 0
        c._align_wrist(sample(.4, q, speed=.3))
        assert c.phase == "align"
        c._align_wrist(sample(.5, q))
        assert c.wrist_confirmations == 0
        c._align_wrist(sample(.6, q))
        assert c.wrist_confirmations == 1
        c._align_wrist(sample(.7, q))
        assert c.phase == "press"
        assert c.freeze_target.camera == "wrist"
        assert c.freeze_target.point[0] == .3
    finally:
        c.close()


def test_motion_between_images_and_delayed_exposures_reset_confirmation():
    c, q = controller(y=.006, detected_y=.006)
    try:
        for t in (.1, .2, .3, .4):
            c._align_wrist(sample(t, q))
        assert c.wrist_confirmations == 1
        c._align_wrist(sample(.42, q, image_t=.4, speed=.3))
        assert c.wrist_confirmations == 0
        assert c.wrist_settle_after is None
        c._align_wrist(sample(.55, q, image_t=.5))
        assert c.wrist_settle_after == .55
        c._align_wrist(sample(.62, q, image_t=.54))  # Arrived after stop, captured before it.
        assert c.wrist_confirmations == 0
        c._align_wrist(sample(.7, q, image_t=.6))
        assert c.wrist_confirmations == 1
        c._align_wrist(sample(.8, q, image_t=.7))
        assert c.phase == "press"
    finally:
        c.close()


def test_loss_holds_reacquires_and_never_falls_back_to_head():
    c, q = controller()
    try:
        for t in (.1, .2, .3):
            c._align_wrist(sample(t, q))
        checked_goal = c.alignment_goal.copy()
        assert c._align_wrist(sample(.4, q, missing=True)) == pytest.approx(checked_goal)
        c.localizer.missing = True
        c._align_wrist(sample(.5, q))
        assert c.localizer.resets == 1
        c.localizer.missing = False
        for t in (.6, .7):
            c._align_wrist(sample(t, q))
            assert c.report.corrections == 1
        c._align_wrist(sample(.8, q))
        assert c.report.corrections == 2
        assert set(c.localizer.cameras) == {"wrist"}
        c._align_wrist(sample(.9, q, missing=True))
        with pytest.raises(RuntimeError, match="reacquisition timeout"):
            c._align_wrist(sample(1.92, q, missing=True))
    finally:
        c.close()


def test_deadline_precedes_any_new_move_or_press():
    c, q = controller()
    try:
        with pytest.raises(RuntimeError, match="within its budget"):
            c._align_wrist(sample(2.02, q))
        assert c.localizer.cameras == []
    finally:
        c.close()


@pytest.mark.parametrize("mode,use_wrist", [("bottle", True), ("button", False)])
def test_opt_in_preserves_bottle_and_head_only(mode, use_wrist):
    c, _ = controller()
    cfg = c.cfg.copy()
    cfg.update(mode=mode, use_wrist=use_wrist)
    other = PressController(cfg, c.robot, c.collision, c.localizer)
    assert not other.wrist_servo_enabled
    other.close()
    c.close()


def test_async_failure_completion_is_reported_even_without_another_frame():
    release, started = Event(), Event()

    class Localizer:
        def locate(self, *args):
            started.set()
            release.wait(2)
            raise NotFound("occluded")

    pipeline = LatestPerception(Localizer(), asynchronous=True)
    try:
        pipeline.update(sample(.1, np.zeros(29)).wrist, "up")
        assert started.wait(2)
        release.set()
        with pytest.raises(NotFound):
            pipeline.future.result(timeout=2)
        assert pipeline.update(None, "up") is None
        assert pipeline.completed_key == ("wrist", .1, 0)
        assert pipeline.error == "occluded"
    finally:
        release.set()
        pipeline.close()


@pytest.mark.parametrize("override", [{"enabled": "false"}, {"window": 1}, {"gain": 2},
                                      {"max_step_m": .004}, {"lost_timeout_s": float("nan")}])
def test_invalid_servo_settings(override):
    cfg = load_settings()["wrist_servo"]
    with pytest.raises(ValueError):
        load_settings(wrist_servo={**cfg, **override})
