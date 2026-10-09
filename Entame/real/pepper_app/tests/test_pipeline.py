import time

import numpy as np
from conftest import frame_of, make_person, pose

from pepperapp.observations import DetectedObject
from pepperapp.pipeline import RecognitionLoop
from pepperapp.robot import DryRunRobot, LookConfig
from pepperapp.rules import Rule


class ScriptedSource:
    """Blank frames at ~20 fps."""
    name = "scripted"

    def __init__(self):
        self.closed = False

    def read(self):
        time.sleep(0.05)
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def close(self):
        self.closed = True


class WavingDetector:
    """Stands in for YOLO: one person (track 1) who waves for ~2 s, rests ~1 s, and repeats.

    Events fire when a gesture starts, so the rest is what makes a new wave event.
    """

    def __init__(self):
        self.n = 0

    def __call__(self, frame):
        self.n += 1
        if self.n % 60 >= 40:
            return frame_of(make_person(pose()))
        x = 300 + (20 if self.n % 4 < 2 else -20)
        return frame_of(make_person(pose(**{"7": (300, 140), "9": (x, 90)})))


class RecordingRobot(DryRunRobot):
    def __init__(self):
        super().__init__(wave_duration_s=0.1)
        self.animations, self.closed = [], False

    def play_animation(self, name):
        self.animations.append(name)
        super().play_animation(name)

    def close(self):
        self.closed = True


def _wait_for(predicate, timeout=6.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.05)
    return predicate()


WAVE_BACK = (Rule("wave", "wave", cooldown_s=30.0),)


def _loop(robot, reacting=True, rules=WAVE_BACK):
    return RecognitionLoop(ScriptedSource(), WavingDetector(), robot, rules,
                           look_config=LookConfig(), reacting=reacting)


def test_waving_person_makes_pepper_wave_back_once():
    robot = RecordingRobot()
    loop = _loop(robot)
    loop.start()
    try:
        assert _wait_for(lambda: robot.animations)
        assert _wait_for(lambda: loop.snapshot().gestures == ("#1: wave",))
        snapshot = loop.snapshot()
        time.sleep(0.5)
    finally:
        loop.stop()
    assert robot.animations == [LookConfig().wave_animation]   # cooldown 30 s
    texts = [entry.text for entry in loop.snapshot().log]
    assert "wave → wave" in texts and any(t.startswith("wave OK") for t in texts)
    assert snapshot.running and snapshot.persons == 1 and snapshot.frame_jpeg[:2] == b"\xff\xd8"
    assert robot.closed and loop._source.closed


def test_nothing_is_sent_until_reacting_is_on():
    robot = RecordingRobot()
    loop = _loop(robot, reacting=False)
    loop.start()
    try:
        time.sleep(1.5)
        assert robot.animations == []
        loop.set_reacting(True)
        assert _wait_for(lambda: robot.animations)
    finally:
        loop.stop()


def test_manual_action_and_rule_replacement():
    robot = RecordingRobot()
    loop = _loop(robot, reacting=False, rules=())
    loop.start()
    try:
        assert loop.run_action("look_front")
        assert _wait_for(lambda: robot.head_angles() == (0.0, LookConfig().front_pitch))
        loop.set_rules((Rule("wave", "wave"),))
        loop.set_reacting(True)
        assert _wait_for(lambda: robot.animations)
    finally:
        loop.stop()


def test_errors_are_logged_and_the_loop_keeps_going():
    class FailingDetector:
        def __call__(self, frame):
            raise RuntimeError("model broke")
    loop = RecognitionLoop(ScriptedSource(), FailingDetector(), RecordingRobot(), (),
                           look_config=LookConfig())
    loop.start()
    try:
        assert _wait_for(lambda: loop.snapshot().error is not None)
    finally:
        loop.stop()
    errors = [e for e in loop.snapshot().log if e.kind == "error"]
    assert len(errors) == 1 and "model broke" in errors[0].text


class ChairDetector:
    """A chair that never leaves the view."""

    def __call__(self, frame):
        return frame_of(objects=[DetectedObject("chair", (0.0, 200.0, 100.0, 400.0), 0.8)])


def test_turning_reactions_on_reacts_to_what_is_already_in_view():
    robot = RecordingRobot()
    loop = RecognitionLoop(ScriptedSource(), ChairDetector(), robot, (Rule("seen:chair", "look"),),
                           look_config=LookConfig(), reacting=False)
    loop.start()
    try:
        assert _wait_for(lambda: any(e.text == "seen:chair" for e in loop.snapshot().log))
        time.sleep(0.5)
        loop.set_reacting(True)
        assert _wait_for(lambda: any(e.text == "seen:chair → look" for e in loop.snapshot().log))
        assert _wait_for(lambda: robot.head_angles()[0] > 0.3)   # chair on the left -> turn left
    finally:
        loop.stop()
