import threading

import pytest

from hold_core import (MockCommandAdapter, command_fields, run_mock_hold,
                       run_periodic_loop)


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, seconds)


Q0 = tuple(index / 100.0 for index in range(29))


def test_mock_uses_production_fields_and_identical_q():
    clock = FakeClock()
    metrics, adapter = run_mock_hold(
        Q0, 0.75, 50.0, clock=clock.monotonic, sleeper=clock.sleep
    )
    assert metrics["termination_reason"] == "completed_duration"
    assert metrics["actual_duration_s"] == pytest.approx(0.75)
    assert metrics["command_generated_count"] == 39
    assert metrics["mock_send_count"] == 39
    assert metrics["all_commands_identical_q"]
    assert all(row["q"] == Q0 for row in adapter.commands)
    assert all(row["dq"] == (0.0,) * 29 for row in adapter.commands)
    assert adapter.commands[0]["kp"][:12] == (0.0,) * 12
    assert adapter.commands[0]["kp"][12:] == (60.0,) * 17
    assert adapter.commands[0]["kd"][12:] == (1.5,) * 17


def test_requested_rates_share_the_same_core():
    for duration, rate, count in ((0.75, 50.0, 39), (1.0, 50.0, 51),
                                  (0.75, 20.0, 16)):
        clock = FakeClock()
        metrics, _ = run_mock_hold(
            Q0, duration, rate, clock=clock.monotonic, sleeper=clock.sleep
        )
        assert metrics["termination_reason"] == "completed_duration"
        assert metrics["actual_duration_s"] == pytest.approx(duration)
        assert metrics["mock_send_count"] == count


def test_external_stop_and_cancel_are_distinct():
    clock = FakeClock()
    stopped = threading.Event()
    stopped.set()
    result = run_periodic_loop(
        0.75, 50.0, lambda *_: None, stop_event=stopped,
        clock=clock.monotonic, sleeper=clock.sleep,
    )
    assert result.termination_reason == "external_stop"
    cancelled = threading.Event()
    cancelled.set()
    result = run_periodic_loop(
        0.75, 50.0, lambda *_: None, cancel_event=cancelled,
        clock=clock.monotonic, sleeper=clock.sleep,
    )
    assert result.termination_reason == "cancelled"


def test_sender_failure_is_reported_with_exception():
    class BrokenAdapter(MockCommandAdapter):
        def send(self, fields):
            raise RuntimeError("mock sender broke")

    clock = FakeClock()
    metrics, _ = run_mock_hold(
        Q0, 0.75, 50.0, adapter=BrokenAdapter(clock.monotonic),
        clock=clock.monotonic, sleeper=clock.sleep,
    )
    assert metrics["termination_reason"] == "sender_failure"
    assert metrics["exception"] == "mock sender broke"
    assert "SenderFailure" in metrics["traceback"]


def test_command_fields_reject_bad_q_and_weight():
    with pytest.raises(ValueError, match="29 finite"):
        command_fields([0.0] * 28, 1.0, 0)
    with pytest.raises(ValueError, match="weight"):
        command_fields(Q0, 1.1, 0)


def test_suspended_loop_logs_timing_without_deadline_abort():
    clock = FakeClock()
    def slow_cycle(*_):
        clock.now += .2
    result = run_periodic_loop(
        .1, 10., slow_cycle, clock=clock.monotonic, sleeper=clock.sleep,
        maximum_lag_s=.1, enforce_deadline=False)
    assert result.termination_reason == 'completed_duration'
    assert max(row['processing_s'] for row in result.timings) == pytest.approx(.2)
