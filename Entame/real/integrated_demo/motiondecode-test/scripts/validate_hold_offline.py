#!/usr/bin/env python3
"""Windows-safe q0 HOLD validation using the production periodic core."""
import argparse
import datetime
import json
from pathlib import Path
import platform
import statistics
import sys
import threading
import time
import traceback
from types import SimpleNamespace as NS

from hold_core import (MockCommandAdapter, PeriodicLoopResult, classify_exception,
                       command_fields, run_mock_hold, timing_metrics)
from run_real_reaction import Runtime


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_Q0 = ROOT / "output/pc2_q0_hold_isolated_20260913_224000/fresh_q0.json"


def load_q0(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    q0 = data["all_q"]
    if len(q0) != 29:
        raise ValueError("q0 fixture must contain 29 joints")
    return q0


def passed(metrics):
    expected_min = metrics["requested_duration_s"]
    return bool(
        metrics["termination_reason"] == "completed_duration"
        and metrics["exception"] is None
        and metrics["actual_duration_s"] >= expected_min
        and metrics["actual_duration_s"] < expected_min + 0.25
        and metrics["command_generated_count"] == metrics["mock_send_count"]
        and metrics["all_commands_identical_q"]
    )


class RuntimeMockReader:
    def __init__(self, q0):
        motors = [NS(q=float(value), dq=0.0, motorstate=0, temperature=[25])
                  for value in q0]
        self.state = NS(mode_machine=2, mode_pr=0, tick=0,
                        imu_state=NS(rpy=[0.0, 0.0, 0.0], gyroscope=[0.0, 0.0, 0.0]),
                        motor_state=motors)
        self.lock = threading.Lock()
        self.latest = (time.monotonic(), self.state)
        self.history = []
        self.sequence = 0

    def get(self):
        now = time.monotonic()
        with self.lock:
            self.state.tick += 1
            self.latest = (now, self.state)
            self.sequence += 1
            self.history.append({
                "sequence": self.sequence,
                "monotonic_s": now,
                "tick": self.state.tick,
                "q": tuple(m.q for m in self.state.motor_state[:29]),
                "dq": tuple(m.dq for m in self.state.motor_state[:29]),
            })
            return self.state

    def motion_history_snapshot(self):
        with self.lock:
            return [dict(row) for row in self.history]


class RuntimeMockWriter:
    def __init__(self, adapter):
        self.adapter = adapter
        self.weight = 0.0

    def write(self, q, weight):
        fields = command_fields(q, weight, len(self.adapter.commands))
        self.adapter.send(fields)
        self.weight = float(weight)


class SafeOwnershipWatchdog:
    def snapshot(self):
        return {
            "ownership_safe": True,
            "reasons": [],
            "last_successful_check_age_s": 0.0,
            "latched_fault": False,
            "current": {"arm_action": {"status": "IDLE"}},
        }


def runtime_with_monitoring(q0, folder):
    """Run Runtime.phase + Runtime.sample + real collision worker with mock I/O."""
    folder.mkdir(parents=True, exist_ok=False)
    adapter = MockCommandAdapter()
    reader = RuntimeMockReader(q0)
    reader.get()
    writer = RuntimeMockWriter(adapter)
    runtime = Runtime(reader, writer, {"all_q": q0}, folder, "hold",
                      ownership_watchdog=SafeOwnershipWatchdog(),
                      baseline_worsening_log_only=True)
    sweep = runtime.reset_baseline({"all_q": q0})
    period = max(0.05, sweep["duration_s"] * 1.10)
    worker = runtime.make_collision_worker(period)
    worker.seed_safe_result({**sweep, "state_tick": reader.state.tick})
    worker.start()
    runtime.collision_worker = worker
    runtime.collision_stale_s = max(1.0, period + 2 * sweep["duration_s"])
    runtime.reset_velocity_sampling()
    started = time.monotonic()
    exception = None
    exception_traceback = None
    try:
        result = runtime.phase("HOLD", 0.75, lambda _u: q0, lambda _u: 1.0)
    except BaseException as exc:
        exception = exc
        exception_traceback = traceback.format_exc()
        result = PeriodicLoopResult(
            0.75, 50.0, started, time.monotonic(), classify_exception(exc),
            getattr(exc, "timings", []),
        )
    finally:
        worker.stop()
        worker_records = worker.timing_records()
        runtime.close_logs()
    metrics = timing_metrics(result, adapter.commands, len(adapter.commands), exception,
                             exception_traceback)
    metrics["result"] = "PASS" if passed(metrics) else "FAIL"
    metrics["runtime_path"] = "Runtime.phase -> Runtime.sample -> MockCommandAdapter"
    durations = [row["duration_s"] for row in worker_records if row.get("duration_s") is not None]
    metrics["collision_worker"] = {
        "sample_count": len(worker_records),
        "duration_mean_s": statistics.fmean(durations) if durations else None,
        "duration_max_s": max(durations) if durations else None,
        "latched_fault": any(row["latched_collision_fault"] for row in worker_records),
    }
    metrics["phase_terminations"] = runtime.phase_terminations
    return metrics


def execute_suite(q0, output_dir):
    cases = {}
    for name, duration, rate in (
        ("test_a", 0.75, 50.0),
        ("test_b", 1.00, 50.0),
        ("test_c", 0.75, 20.0),
    ):
        metrics, _adapter = run_mock_hold(q0, duration, rate)
        metrics["result"] = "PASS" if passed(metrics) else "FAIL"
        cases[name] = metrics

    repeats = []
    for run_index in range(1, 11):
        metrics, _adapter = run_mock_hold(q0, 0.75, 50.0)
        metrics["run"] = run_index
        metrics["result"] = "PASS" if passed(metrics) else "FAIL"
        repeats.append(metrics)
    durations = [row["actual_duration_s"] for row in repeats]
    repetition = {
        "runs": len(repeats),
        "min_duration_s": min(durations),
        "max_duration_s": max(durations),
        "mean_duration_s": statistics.fmean(durations),
        "median_duration_s": statistics.median(durations),
        "early_termination_count": sum(
            row["termination_reason"] != "completed_duration" for row in repeats
        ),
        "exception_count": sum(row["exception"] is not None for row in repeats),
        "result": "PASS" if all(row["result"] == "PASS" for row in repeats) else "FAIL",
        "run_details": repeats,
    }
    near_real = runtime_with_monitoring(q0, output_dir / "existing_runtime_mock")
    return cases, repetition, near_real


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--q0", type=Path, default=DEFAULT_Q0)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    timestamp = datetime.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or ROOT / "output" / f"windows_q0_hold_offline_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=False)
    q0 = load_q0(args.q0)
    cases, repetition, near_real = execute_suite(q0, output_dir)
    report = {
        "schema": "motiondecode-test.windows-hold-offline.v1",
        "environment": {
            "os": platform.platform(),
            "python": sys.version,
            "repository": str(ROOT),
            "q0_fixture": str(args.q0.resolve()),
            "g1_connected": False,
            "dds_used": False,
        },
        "cases": cases,
        "test_a_x10": repetition,
        "existing_runtime_mock": near_real,
        "all_commands_identical_q": all(
            row["all_commands_identical_q"]
            for row in list(cases.values()) + repetition["run_details"]
        ),
        "g1_command_sent": "NONE",
        "result": "PASS" if (
            all(row["result"] == "PASS" for row in cases.values())
            and repetition["result"] == "PASS"
            and near_real["result"] == "PASS"
        ) else "FAIL",
    }
    report_path = output_dir / "report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"REPORT={report_path}")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
