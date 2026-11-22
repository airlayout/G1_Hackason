"""DDS-independent periodic HOLD core shared by real and mock runtimes."""
from dataclasses import dataclass, field
import math
import statistics
import time
import traceback


TERMINATION_REASONS = {
    "completed_duration",
    "external_stop",
    "cancelled",
    "collision_stop",
    "watchdog_timeout",
    "worker_failure",
    "sender_failure",
    "exception",
    "unknown",
}


class HoldLoopError(RuntimeError):
    """A HOLD failure with a machine-readable termination reason."""

    def __init__(self, reason, message, timings=None):
        if reason not in TERMINATION_REASONS:
            raise ValueError(f"Unknown termination reason: {reason}")
        super().__init__(message)
        self.reason = reason
        self.timings = list(timings or [])


class SenderFailure(HoldLoopError):
    def __init__(self, message, timings=None):
        super().__init__("sender_failure", message, timings)


@dataclass
class PeriodicLoopResult:
    requested_duration_s: float
    target_rate_hz: float
    started_monotonic_s: float
    completed_monotonic_s: float
    termination_reason: str
    timings: list = field(default_factory=list)

    @property
    def actual_duration_s(self):
        return self.completed_monotonic_s - self.started_monotonic_s


def command_fields(q, weight, command_index, controlled=range(12, 29)):
    """Build the fields used by both LowCmd and the offline mock adapter."""
    q = tuple(float(value) for value in q)
    if len(q) != 29 or not all(math.isfinite(value) for value in q):
        raise ValueError("Need 29 finite HOLD target joint angles")
    weight = float(weight)
    if not math.isfinite(weight) or not 0.0 <= weight <= 1.0:
        raise ValueError("Invalid HOLD weight")
    controlled = frozenset(controlled)
    return {
        "command_index": int(command_index),
        "q": q,
        "dq": (0.0,) * 29,
        "tau": (0.0,) * 29,
        "kp": tuple(60.0 if i in controlled else 0.0 for i in range(29)),
        "kd": tuple(1.5 if i in controlled else 0.0 for i in range(29)),
        "weight": weight,
    }


def classify_exception(exc):
    if isinstance(exc, HoldLoopError):
        return exc.reason
    text = str(exc).lower()
    if "collision" in text:
        return "collision_stop"
    if "watchdog" in text or "deadline" in text or "stale" in text:
        return "watchdog_timeout"
    if "worker" in text:
        return "worker_failure"
    if "sender" in text or "write failed" in text or "dds write" in text:
        return "sender_failure"
    return "exception" if exc is not None else "unknown"


def run_periodic_loop(duration_s, rate_hz, cycle, *, stop_event=None,
                      cancel_event=None, clock=time.monotonic, sleeper=time.sleep,
                      maximum_lag_s=0.1, enforce_deadline=True):
    """Run the production deadline schedule without any transport dependency.

    Like the original Runtime.phase implementation, this includes both the
    t=0 and duration endpoints. A 0.75 s / 50 Hz request therefore produces
    39 cycles because ceil(37.5) intervals require 38 interval endpoints.
    """
    duration_s = float(duration_s)
    rate_hz = float(rate_hz)
    maximum_lag_s = float(maximum_lag_s)
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("HOLD duration must be finite and positive")
    if not math.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError("HOLD rate must be finite and positive")
    if not math.isfinite(maximum_lag_s) or maximum_lag_s <= 0:
        raise ValueError("Maximum lag must be finite and positive")

    steps = int(math.ceil(duration_s * rate_hz))
    started = clock()
    timings = []
    for index in range(steps + 1):
        if cancel_event is not None and cancel_event.is_set():
            return PeriodicLoopResult(duration_s, rate_hz, started, clock(),
                                      "cancelled", timings)
        if stop_event is not None and stop_event.is_set():
            return PeriodicLoopResult(duration_s, rate_hz, started, clock(),
                                      "external_stop", timings)
        deadline = started + index * duration_s / steps
        sleeper(max(0.0, deadline - clock()))
        cycle_started = clock()
        lag = cycle_started - deadline
        if enforce_deadline and lag > maximum_lag_s:
            raise HoldLoopError(
                "watchdog_timeout",
                f"Fast control loop deadline missed by >{maximum_lag_s * 1000:g} ms",
                timings,
            )
        try:
            cycle(index, index / steps, cycle_started)
        except HoldLoopError as exc:
            exc.timings = timings + list(exc.timings)
            raise
        except BaseException as exc:
            raise HoldLoopError(classify_exception(exc), str(exc), timings) from exc
        processing = clock() - cycle_started
        timings.append({
            "command_index": index,
            "started_monotonic_s": cycle_started,
            "deadline_monotonic_s": deadline,
            "deadline_lag_s": lag,
            "processing_s": processing,
        })
        if enforce_deadline and processing > maximum_lag_s:
            raise HoldLoopError(
                "watchdog_timeout",
                f"Fast control loop processing exceeded {maximum_lag_s * 1000:g} ms",
                timings,
            )
    return PeriodicLoopResult(duration_s, rate_hz, started, clock(),
                              "completed_duration", timings)


class MockCommandAdapter:
    """In-memory command sink. It never imports or constructs DDS objects."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.commands = []

    def send(self, fields):
        record = dict(fields)
        record["timestamp"] = self.clock()
        self.commands.append(record)


def timing_metrics(result, commands, generated_count, exception=None,
                   exception_traceback=None):
    timestamps = [row["timestamp"] for row in commands]
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:])]
    actual = result.actual_duration_s
    effective = ((len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
                 if len(timestamps) > 1 and timestamps[-1] > timestamps[0] else None)
    q_identical = not commands or all(row["q"] == commands[0]["q"] for row in commands)
    return {
        "requested_duration_s": result.requested_duration_s,
        "actual_duration_s": actual,
        "target_rate_hz": result.target_rate_hz,
        "command_generated_count": int(generated_count),
        "mock_send_count": len(commands),
        "first_send_timestamp": timestamps[0] if timestamps else None,
        "last_send_timestamp": timestamps[-1] if timestamps else None,
        "interval_min_s": min(intervals) if intervals else None,
        "interval_max_s": max(intervals) if intervals else None,
        "interval_mean_s": statistics.fmean(intervals) if intervals else None,
        "interval_median_s": statistics.median(intervals) if intervals else None,
        "effective_hz": effective,
        "termination_reason": result.termination_reason,
        "exception": None if exception is None else str(exception),
        "traceback": (None if exception is None else
                      (exception_traceback or traceback.format_exc())),
        "all_commands_identical_q": q_identical,
    }


def run_mock_hold(q0, duration_s, rate_hz, *, adapter=None, stop_event=None,
                  cancel_event=None, clock=time.monotonic, sleeper=time.sleep,
                  maximum_lag_s=0.1):
    if adapter is None:
        adapter = MockCommandAdapter(clock)
    generated = 0

    def cycle(index, _progress, _cycle_started):
        nonlocal generated
        fields = command_fields(q0, 1.0, index)
        generated += 1
        try:
            adapter.send(fields)
        except BaseException as exc:
            raise SenderFailure(str(exc)) from exc

    started = clock()
    try:
        result = run_periodic_loop(
            duration_s, rate_hz, cycle, stop_event=stop_event,
            cancel_event=cancel_event, clock=clock, sleeper=sleeper,
            maximum_lag_s=maximum_lag_s,
        )
        return timing_metrics(result, adapter.commands, generated), adapter
    except BaseException as exc:
        timings = getattr(exc, "timings", [])
        result = PeriodicLoopResult(float(duration_s), float(rate_hz), started,
                                    clock(), classify_exception(exc), timings)
        return timing_metrics(result, adapter.commands, generated, exc), adapter
