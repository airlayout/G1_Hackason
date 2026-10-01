from __future__ import annotations

import numpy as np

from common import NAMES
from resident_worker import (
    CONTROLLED_RETURN_CONSECUTIVE,
    CONTROLLED_RETURN_MAX_S,
    CONTROLLED_RETURN_MIN_S,
    adaptive_controlled_return_hold,
    controlled_return_then_release,
    q0_return_diagnostics,
)
from run_real_reaction import HOLD_DEVIATION_LIMIT


def row(error: float) -> dict:
    q = np.zeros(29)
    q[19] = error
    return {"phase": "q0_hold_after", "q": q.tolist()}


def test_controlled_return_passes_even_when_post_release_drifts() -> None:
    post = np.zeros(29)
    post[19] = .04
    result = q0_return_diagnostics([row(.02)], np.zeros(29), post)
    assert result["returned_to_q0"] is True
    assert result["controlled_q0_return_error_rad"] == .02
    assert result["post_release_q0_error_rad"] == .04
    assert result["post_release_worst_joint"] == NAMES[19]


def test_post_release_recovery_does_not_hide_failed_controlled_return() -> None:
    post = np.zeros(29)
    post[19] = .02
    result = q0_return_diagnostics([row(.04)], np.zeros(29), post)
    assert result["returned_to_q0"] is False
    assert result["controlled_q0_return_error_rad"] == .04
    assert result["post_release_q0_error_rad"] == .02


def test_controlled_return_threshold_remains_point_zero_three() -> None:
    assert HOLD_DEVIATION_LIMIT == .03
    assert q0_return_diagnostics(
        [row(.03)], np.zeros(29), np.zeros(29)
    )["returned_to_q0"] is True
    assert q0_return_diagnostics(
        [row(np.nextafter(.03, 1.0))], np.zeros(29), np.zeros(29)
    )["returned_to_q0"] is False


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Runtime:
    def __init__(self, clock, error):
        self.clock = clock
        self.error = error
        self.weights = []

    def sample(self, phase, q0):
        assert phase == "q0_hold_after"
        q = np.asarray(q0).copy()
        q[13] += self.error(self.clock())
        return q

    def send(self, phase, q0, weight):
        self.weights.append((phase, weight))

    def phase(self, name, seconds, pose, weight):
        assert name == "RELEASE"
        self.weights.append((name, weight(0)))
        self.weights.append((name, weight(1)))


def run(error):
    clock = Clock(); runtime = Runtime(clock, error)
    result = adaptive_controlled_return_hold(
        runtime, np.zeros(29), clock=clock, sleeper=clock.sleep)
    return result, runtime


def test_adaptive_return_waits_for_three_good_samples_after_slow_tracking():
    result, _ = run(lambda t: .13 if t < .30 else .025)
    assert result["passed"] is True
    assert .30 <= result["wait_s"] <= .35
    assert result["consecutive_passes"] == 3


def test_adaptive_return_timeout_still_releases_to_weight_zero():
    clock = Clock(); runtime = Runtime(clock, lambda t: .04)
    result = controlled_return_then_release(
        runtime, np.zeros(29), clock=clock, sleeper=clock.sleep)
    assert result["passed"] is False
    assert result["wait_s"] == CONTROLLED_RETURN_MAX_S
    assert runtime.weights[-1] == ("RELEASE", 0)


def test_one_threshold_crossing_does_not_pass():
    result, _ = run(lambda t: .029 if .10 <= t < .12 else .04)
    assert result["passed"] is False
    assert result["consecutive_passes"] == 0


def test_three_consecutive_samples_pass():
    result, _ = run(lambda t: .02 if t >= .20 else .04)
    assert result["passed"] is True
    assert result["consecutive_passes"] == CONTROLLED_RETURN_CONSECUTIVE


def test_already_returned_motion_does_not_wait_unnecessarily():
    result, _ = run(lambda t: .02)
    assert result["passed"] is True
    assert CONTROLLED_RETURN_MIN_S <= result["wait_s"] <= CONTROLLED_RETURN_MIN_S + .05
