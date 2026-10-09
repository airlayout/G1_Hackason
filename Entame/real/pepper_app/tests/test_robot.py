import math
import time

import pytest

from pepperapp.observations import Target
from pepperapp.robot import ActionExecutor, DryRunRobot, LookConfig, look_angles
from pepperapp.rules import Command

CONFIG = LookConfig()


def test_centered_target_is_inside_the_deadband():
    assert look_angles(Target(0.5, 0.5), (0.2, -0.1), CONFIG) is None


def test_target_on_the_right_turns_right_and_below_looks_down():
    yaw, pitch = look_angles(Target(0.75, 0.75), (0.0, 0.0), CONFIG)
    assert yaw == pytest.approx(-0.25 * math.radians(57.2))
    assert pitch == pytest.approx(min(0.25 * math.radians(44.3), CONFIG.pitch_max))
    assert yaw < 0 < pitch


def test_offset_is_added_to_the_current_head_angle_and_clamped():
    yaw, _ = look_angles(Target(0.0, 0.5), (0.3, 0.0), CONFIG)
    assert yaw == pytest.approx(0.3 + 0.5 * math.radians(57.2))
    yaw, pitch = look_angles(Target(0.0, 0.0), (0.95, -0.35), CONFIG)
    assert (yaw, pitch) == (CONFIG.yaw_limit, CONFIG.pitch_min)


@pytest.mark.parametrize("kwargs", [{"yaw_limit": 3.0}, {"pitch_min": -1.0}, {"speed": 0.9},
                                    {"wave_animation": "animations/Stand/Emotions/Happy_1"}])
def test_look_config_rejects_values_beyond_the_joint_limits(kwargs):
    with pytest.raises(ValueError):
        LookConfig(**kwargs)


class RecordingRobot(DryRunRobot):
    def __init__(self, fail=False):
        super().__init__(wave_duration_s=0.2)
        self.calls, self.fail = [], fail

    def set_head(self, yaw, pitch, speed):
        if self.fail:
            raise RuntimeError("ALMotion error")
        self.calls.append(("set_head", round(yaw, 3), round(pitch, 3), speed))
        super().set_head(yaw, pitch, speed)

    def play_animation(self, name):
        self.calls.append(("play_animation", name))
        super().play_animation(name)


def _wait(results, n=1, timeout=3.0):
    deadline = time.monotonic() + timeout
    while len(results) < n and time.monotonic() < deadline:
        time.sleep(0.01)
    return results


def test_executor_runs_one_action_at_a_time():
    robot, results = RecordingRobot(), []
    executor = ActionExecutor(robot, CONFIG, on_result=results.append)
    assert executor.submit(Command("wave", "wave", 0, 0.0))
    assert executor.busy
    assert not executor.submit(Command("look_front", "person_left", 1, 0.0))
    _wait(results)
    executor.shutdown()
    assert robot.calls == [("play_animation", CONFIG.wave_animation)]
    assert results[0].ok and not executor.busy


def test_look_uses_the_current_head_angle():
    robot, results = RecordingRobot(), []
    robot._head = (0.2, 0.0)
    executor = ActionExecutor(robot, CONFIG, on_result=results.append)
    executor.submit(Command("look", "wave", 0, 0.0, Target(0.25, 0.5)))
    _wait(results)
    executor.shutdown()
    expected_yaw = round(0.2 + 0.25 * math.radians(57.2), 3)
    assert robot.calls == [("set_head", expected_yaw, 0.0, CONFIG.speed)]


def test_failures_are_reported_not_swallowed():
    results = []
    executor = ActionExecutor(RecordingRobot(fail=True), CONFIG, on_result=results.append)
    executor.submit(Command("look_front", "manual", -1, 0.0))
    _wait(results)
    executor.shutdown()
    assert not results[0].ok and "ALMotion error" in results[0].detail
    assert not executor.busy


def test_look_without_target_and_unknown_action_fail():
    results = []
    executor = ActionExecutor(DryRunRobot(0.0), CONFIG, on_result=results.append)
    for action in ("look", "move"):
        executor.submit(Command(action, "manual", -1, 0.0))
        _wait(results, n=len(results) + 1)
    executor.shutdown()
    assert [r.ok for r in results] == [False, False]
    assert "未対応の操作" in results[1].detail


def test_submit_after_shutdown_is_refused_and_not_left_busy():
    executor = ActionExecutor(DryRunRobot(0.0), CONFIG)
    executor.shutdown()
    assert not executor.submit(Command("wave", "wave", 0, 0.0))
    assert not executor.busy


def test_concurrent_submits_accept_exactly_one():
    import threading
    executor = ActionExecutor(RecordingRobot(), CONFIG)
    accepted, barrier = [], threading.Barrier(8)

    def submit():
        barrier.wait()
        accepted.append(executor.submit(Command("wave", "wave", 0, 0.0)))
    threads = [threading.Thread(target=submit) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    executor.shutdown()
    assert accepted.count(True) == 1


def test_dry_run_robot_keeps_head_angles():
    robot = DryRunRobot(0.0)
    robot.set_head(0.4, -0.1, 0.1)
    assert robot.head_angles() == (0.4, -0.1)
