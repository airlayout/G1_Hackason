from resident_worker import (RECOVERY_DEADLINE_S, classify_runtime_exception,
                             wait_until_settled)
from robot_transport import (RecoverableMotionEnvelopeExceeded,
                             RecoverablePreCommandStationarityError)


def test_envelope_errors_are_recoverable_only_in_hackathon_runtime():
    for error in (RecoverableMotionEnvelopeExceeded("tilt"),
                  RecoverablePreCommandStationarityError("arms")):
        assert classify_runtime_exception(error, False) == "HARD_FAULT"
        assert classify_runtime_exception(error, True) == "RECOVERABLE_ABORT"


def test_hard_fault_categories_never_become_recoverable():
    for text in ("Arm control ownership blocked: external writer",
                 "Arm motor error or high temperature",
                 "LowState absent/stale; refuse motion",
                 "DDS write failed; robot receipt is not guaranteed",
                 "Non-finite robot state"):
        assert classify_runtime_exception(RuntimeError(text), True) == "HARD_FAULT"
    assert classify_runtime_exception(None, True) == "SUCCESS"


class FakeClock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def settle(summary_at, preflight=lambda: None):
    clock = FakeClock()
    result = wait_until_settled(lambda: summary_at(clock.now), preflight,
                                clock=clock, sleeper=clock.sleep)
    return result, clock.now


CALM = {"imu_rpy": [.05, .04, 0.], "gyro": [.02, .01, .02]}
SWAY = {"imu_rpy": [.05, .04, 0.], "gyro": [.6, .1, .02]}


def test_settled_requires_a_continuous_calm_second():
    result, elapsed = settle(lambda t: SWAY if t < 3. else CALM)
    assert result is True and 4. <= elapsed < 4.3


def test_calm_gap_restarts_the_settling_window():
    result, elapsed = settle(lambda t: SWAY if t < .5 or 1.2 < t < 1.3 else CALM)
    assert result is True and elapsed >= 2.3


def test_persistent_sway_never_settles():
    result, elapsed = settle(lambda t: SWAY)
    assert result is False and elapsed >= RECOVERY_DEADLINE_S


def test_failed_start_gate_blocks_settling():
    def preflight():
        raise RuntimeError("ownership is not safe")
    result, _ = settle(lambda t: CALM, preflight)
    assert result is False
