#!/usr/bin/env python3
"""Resident Unix-socket MotionDecode worker with warmed DDS and reaction assets."""
from __future__ import annotations

import argparse
import gc
import json
import signal
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from common import NAMES, ROOT, acquire_file_lock, interpolate, output_path, write_json
from reaction_profiles import (align_precomputed_reaction, precompute_named_reaction)
from resident_protocol import DryRunBackend, JsonUnixServer, ResidentController
from robot_transport import (ReadOnlyState, RecoverableMotionEnvelopeExceeded,
                             RecoverablePreCommandStationarityError,
                             state_summary, validate_state)
from run_real_reaction import (DT, FAST_ACQUIRE_S, FAST_HOLD_S, FAST_POST_RELEASE_S,
                               FAST_RELEASE_S, HOLD_DEVIATION_LIMIT, InitializedWriter,
                               PrevalidatedCollisionCache, Runtime,
                               experimental_preflight, evaluate_paths,
                               validate_upper, wait_for_fresh_idle_ownership)
from safety_monitor import OwnershipWatchdog


REACTION_SPECS = {
    # Public engine name -> exact already-tested trajectory profile.
    "surprise": ("surprise_100_hackathon_suspended", 1.0, 1.0, True),
    "found": ("found", .5, .25, False),
    "joy": ("joy_100_hackathon_suspended", 1.0, 1.0, True),
}

CONTROLLED_RETURN_MIN_S = .10
CONTROLLED_RETURN_MAX_S = .75
CONTROLLED_RETURN_CONSECUTIVE = 3


def classify_runtime_exception(exc: BaseException | None, hackathon_runtime: bool) -> str:
    if exc is None:
        return "SUCCESS"
    if hackathon_runtime and isinstance(exc, RecoverableMotionEnvelopeExceeded):
        return "RECOVERABLE_ABORT"
    if hackathon_runtime and isinstance(exc, RecoverablePreCommandStationarityError):
        return "RECOVERABLE_ABORT"
    return "HARD_FAULT"


def adaptive_controlled_return_hold(runtime, q0: np.ndarray, *,
                                    clock=time.monotonic, sleeper=time.sleep) -> dict:
    """Hold q0 at full weight until the existing tolerance is stably met."""
    q0 = np.asarray(q0, dtype=float)
    started = clock(); samples = 0; consecutive = 0
    measured = q0.copy(); error = float("inf"); worst_index = 12
    while True:
        measured = np.asarray(runtime.sample("q0_hold_after", q0), dtype=float)
        runtime.send("q0_hold_after", q0, 1.)
        elapsed = clock() - started
        delta = np.abs(measured[12:] - q0[12:])
        worst_index = 12 + int(np.argmax(delta)); error = float(delta[worst_index - 12])
        if elapsed + 1e-12 >= CONTROLLED_RETURN_MIN_S:
            samples += 1
            consecutive = consecutive + 1 if error <= HOLD_DEVIATION_LIMIT else 0
            if consecutive >= CONTROLLED_RETURN_CONSECUTIVE:
                return {
                    "passed": True, "wait_s": elapsed, "samples": samples,
                    "consecutive_passes": consecutive, "error_rad": error,
                    "worst_joint": NAMES[worst_index], "worst_joint_index": worst_index,
                }
        if elapsed + 1e-12 >= CONTROLLED_RETURN_MAX_S:
            return {
                "passed": False, "wait_s": elapsed, "samples": samples,
                "consecutive_passes": consecutive, "error_rad": error,
                "worst_joint": NAMES[worst_index], "worst_joint_index": worst_index,
            }
        sleeper(min(DT, CONTROLLED_RETURN_MAX_S - elapsed))


def controlled_return_then_release(runtime, q0: np.ndarray, **timing) -> dict:
    outcome = adaptive_controlled_return_hold(runtime, q0, **timing)
    runtime.phase("RELEASE", FAST_RELEASE_S, lambda u: q0, lambda u: 1-u)
    return outcome


def q0_return_diagnostics(rows: list[dict], q0: np.ndarray,
                          post_release_q: np.ndarray,
                          controlled_return: dict | None = None) -> dict:
    """Separate controlled q0 return from native post-release drift."""
    q0 = np.asarray(q0, dtype=float)
    post_release_q = np.asarray(post_release_q, dtype=float)
    controlled_rows = [row for row in rows if row.get("phase") == "q0_hold_after"]
    controlled_error = None
    if controlled_rows:
        controlled_q = np.asarray(controlled_rows[-1]["q"], dtype=float)
        controlled_error = float(np.max(np.abs(controlled_q[12:] - q0[12:])))
    controlled_return = controlled_return or {}
    controlled_passed = controlled_return.get(
        "passed",
        controlled_error is not None and controlled_error <= HOLD_DEVIATION_LIMIT,
    )
    post_delta = np.abs(post_release_q[12:] - q0[12:])
    post_index = 12 + int(np.argmax(post_delta))
    return {
        "returned_to_q0": bool(controlled_passed),
        "controlled_q0_return_error_rad": controlled_return.get(
            "error_rad", controlled_error),
        "controlled_q0_return_worst_joint": controlled_return.get("worst_joint"),
        "controlled_q0_return_worst_joint_index": controlled_return.get(
            "worst_joint_index"),
        "controlled_return_wait_s": controlled_return.get("wait_s"),
        "controlled_return_samples": controlled_return.get("samples"),
        "controlled_return_consecutive_passes": controlled_return.get(
            "consecutive_passes"),
        "post_release_q0_error_rad": float(post_delta[post_index - 12]),
        "post_release_worst_joint": NAMES[post_index],
        "post_release_worst_joint_index": post_index,
    }


class RealResidentBackend:
    """One process owns warmed read-only state, writer, model and assets."""

    def __init__(self, interface: str, expected_arm_pid: int, discovery_peers: list[str],
                 output_root: Path, reactions: set[str], lowstate_backend: str,
                 hackathon_runtime: bool = False) -> None:
        self.interface = interface; self.expected_arm_pid = expected_arm_pid
        self.discovery_peers = discovery_peers; self.output_root = output_root
        self.reader = self.writer = self.runtime = self.watchdog = None
        self.reactions = frozenset(reactions)
        self.lowstate_backend = lowstate_backend
        self.hackathon_suspended_mode = True
        self.hackathon_runtime = bool(hackathon_runtime)
        self.lock_file = None; self.assets = {}; self.preflights = {}
        self.full_sweep = None; self.folder = None; self.execution_index = 0

    def start(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.folder = output_path(self.output_root / f"resident_{stamp}" / "worker.json").parent
        self.lock_file = (ROOT / "output" / "robot.lock").open("a")
        acquire_file_lock(self.lock_file)

        # File parsing and retiming happen before READY and never in trigger path.
        for public_name in self.reactions:
            profile, arms, waist, fast = REACTION_SPECS[public_name]
            self.assets[public_name] = precompute_named_reaction(
                profile, arms, waist, fast_reaction=fast)

        self.reader = ReadOnlyState(self.interface, self.discovery_peers,
                                    self.expected_arm_pid,
                                    lowstate_backend=self.lowstate_backend)
        state = self.reader.wait(3.0); validate_state(
            state, initial=True,
            hackathon_suspended_mode=self.hackathon_suspended_mode)
        wait_for_fresh_idle_ownership(
            self.reader, allow_robot_internal_idle_traffic=True)
        self.reader.require_stationary()
        state = self.reader.get(); validate_state(
            state, initial=True,
            hackathon_suspended_mode=self.hackathon_suspended_mode)
        snapshot = state_summary(state)

        # All heavy trajectory and collision checks are completed before READY.
        for name, asset in self.assets.items():
            paths, _, durations, metadata = align_precomputed_reaction(
                asset, np.asarray(snapshot["all_q"]))
            report = evaluate_paths(paths, durations, snapshot, motion_layout="upper-body")
            hard = (experimental_preflight(report)["hard_safety_passed"]
                    if asset["profile"].get("validation_mode") == "hackathon_suspended"
                    else report["passed"])
            self.preflights[name] = {"passed": bool(hard), "report": report,
                                     "metadata": metadata}

        self.runtime = Runtime(
            self.reader, None, snapshot, self.folder, "resident",
            baseline_worsening_log_only=True, prevalidated_runtime=True,
            release_settling_warning_only=True, hackathon_suspended_mode=True,
            release_duration_s=FAST_RELEASE_S)
        provisional = np.asarray(snapshot["all_q"])
        self.writer = InitializedWriter(self.reader, provisional)
        self.runtime.writer = self.writer
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            own = self.reader.ownership_snapshot(
                own_writer=True, allow_arm_action_unknown=True,
                allow_robot_internal_idle_traffic=True)
            if own["passed"]: break
            if any(reason != "this controller writer is not exactly one"
                   for reason in own["reasons"]):
                raise RuntimeError("ownership changed during resident startup: " +
                                   "; ".join(own["reasons"]))
            time.sleep(.02)
        else:
            raise RuntimeError("resident writer discovery timed out")
        self.full_sweep = self.runtime.reset_baseline(snapshot)
        if (self.full_sweep["new_penetration_pairs"] or
                self.full_sweep["new_dangerous_pairs"]):
            raise RuntimeError("resident startup collision sweep failed")
        self.runtime.collision_worker = PrevalidatedCollisionCache(self.full_sweep)
        self.runtime.collision_stale_s = float("inf")
        self.watchdog = OwnershipWatchdog(
            lambda: self.reader.ownership_snapshot(
                own_writer=True, allow_arm_action_unknown=True,
                allow_robot_internal_idle_traffic=True), period_s=.1).start()
        if not self.watchdog.wait_first()["ownership_safe"]:
            raise RuntimeError("resident ownership watchdog is unsafe")
        self.runtime.ownership_watchdog = self.watchdog
        self.runtime.reset_velocity_sampling()
        write_json(self.folder / "worker.json", self.status())

    def status(self) -> dict:
        result = {"mode": "real", "socket_worker": True,
                  "lowstate_backend": self.lowstate_backend,
                  "preflight": {k: v["passed"] for k, v in self.preflights.items()},
                  "publisher_persistent": self.writer is not None,
                  "weight": None if self.writer is None else self.writer.weight}
        if self.reader is not None:
            try:
                now = time.monotonic()
                with self.reader.lock: received = self.reader.latest[0]
                result["lowstate_age_s"] = now - received
                own = self.reader.ownership_snapshot(
                    own_writer=self.writer is not None, allow_arm_action_unknown=True,
                    allow_robot_internal_idle_traffic=True)
                result["ownership_safe"] = own["passed"]
                result["arm_action"] = own["arm_action"]["status"]
                result["external_writers"] = len(own["external_arm_sdk_writers"])
            except BaseException as exc:
                result["status_error"] = str(exc)
        return result

    def preflight(self) -> dict:
        """Run the existing receive-only motion-start gates without commanding G1."""
        state = self.reader.get(max_age=.15)
        validate_state(
            state, initial=True,
            hackathon_suspended_mode=self.hackathon_suspended_mode)
        self.reader.require_stationary()
        validate_upper(np.asarray(state_summary(state)["all_q"]))
        ownership = self.watchdog.fresh_snapshot(max_age_s=.75,timeout_s=2.)
        if not ownership["ownership_safe"]:
            raise RuntimeError(
                "ownership is not safe: " + "; ".join(ownership["reasons"])
            )
        current_ownership = self.reader.ownership_snapshot(
            own_writer=True, allow_arm_action_unknown=False,
            allow_robot_internal_idle_traffic=True,
        )
        if not current_ownership["passed"]:
            raise RuntimeError(
                "current ownership is not safe: "
                + "; ".join(current_ownership["reasons"])
            )
        arm_action = current_ownership["arm_action"]["status"]
        if arm_action != "IDLE":
            raise RuntimeError(f"Arm Action is not IDLE: {arm_action}")
        if self.writer is None or self.writer.weight != 0.:
            raise RuntimeError("MotionDecode weight is not zero")
        return {"passed": True, **self.status()}

    def execute(self, reaction: str, request: dict, received: float) -> dict:
        if reaction not in self.assets or not self.preflights[reaction]["passed"]:
            raise RuntimeError(f"reaction is not resident-prevalidated: {reaction}")
        state = self.reader.get(max_age=.15)
        try:
            validate_state(
                state, initial=True,
                hackathon_suspended_mode=self.hackathon_suspended_mode)
        except RecoverablePreCommandStationarityError as exc:
            if not self.hackathon_runtime or self.writer.weight != 0.:
                raise
            deadline = time.monotonic() + 2.0
            recovery_status = None
            while time.monotonic() < deadline:
                try:
                    self.runtime.sample("PRECOMMAND_RECOVERY", self.runtime.q0)
                    self.preflight()
                    recovery_status = self.status()
                    break
                except RuntimeError:
                    time.sleep(.05)
            if not (recovery_status
                    and recovery_status.get("ownership_safe") is True
                    and recovery_status.get("external_writers") == 0
                    and recovery_status.get("weight") == 0.
                    and "status_error" not in recovery_status):
                raise
            return {
                "schema": "motiondecode-test.resident-result.v1",
                "reaction": reaction, "status": "recoverable_abort",
                "classification": "RECOVERABLE_ABORT", "executed": False,
                "motion_completed": False, "released": True,
                "weight_zero": True, "returned_to_q0": False,
                "hard_fault": None, "recoverable_reason": str(exc),
                "recovery_status": recovery_status, "commands_sent": 0,
                "reaction_engine_trigger_monotonic_s": request.get(
                    "trigger_monotonic_s") if request else None,
                "worker_receive_monotonic_s": received,
            }
        q0 = np.asarray(state_summary(state)["all_q"]); validate_upper(q0)
        paths, _, durations, metadata = align_precomputed_reaction(self.assets[reaction], q0)
        # Trigger-time checks are bounded vector operations only.
        merged = np.concatenate(list(paths.values()))
        if not np.isfinite(merged).all() or merged.shape[1] != 17:
            raise RuntimeError("resident trajectory integrity failure")

        # Keep the cached fast path, but refresh stale SAFE state immediately
        # before motion. Latched unsafe/error/stopped states never refresh and
        # remain fail-closed. This also closes the preflight/execute TTL gap.
        ownership = self.watchdog.fresh_snapshot(max_age_s=.75,timeout_s=2.)
        if not ownership["ownership_safe"]:
            raise RuntimeError("ownership is not safe: " +
                               "; ".join(ownership["reasons"]))

        runtime = self.runtime; runtime.q0 = q0.copy(); runtime.previous_target = q0.copy()
        runtime.rows.clear(); runtime.command_rows.clear(); runtime.fast_loop_timings.clear()
        runtime.phase_terminations.clear(); runtime.velocity_records.clear()
        sent_before = self.writer.sent
        acquire_start = time.monotonic(); clip_start = None; completed = False
        execution_exception = None
        hard_fault = None
        controlled_return = {}
        gc.collect(); gc.disable()
        try:
            runtime.phase("ACQUIRE_RAMP", FAST_ACQUIRE_S, lambda u: q0, lambda u: u)
            runtime.phase("HOLD", FAST_HOLD_S, lambda u: q0, lambda u: 1.)
            for phase, path in paths.items():
                if phase == "clip": clip_start = time.monotonic()
                times = np.linspace(0, durations[phase], len(path))
                def pose(u, path=path, times=times):
                    target = q0.copy()
                    target[12:] = interpolate(times, path, u * times[-1])
                    return target
                runtime.phase(phase, durations[phase], pose, lambda u: 1.)
            completed = True
            controlled_return = controlled_return_then_release(runtime, q0)
            runtime.observe("POST-RELEASE", FAST_POST_RELEASE_S, q0)
        except BaseException as exc:
            execution_exception = exc
            hard_fault = str(exc)
            runtime.abort_release()
        finally:
            gc.enable()
            # A persistent DDS writer is not persistent ownership: every request
            # ends at weight zero even when the motion raised an exception.
            if self.writer.weight != 0.:
                runtime.abort_release()

        finished = time.monotonic(); final = state_summary(self.reader.get())
        rows = list(runtime.rows)
        tracking = max((r["tracking_error_rad"] for r in rows), default=0.)
        clip_tracking = max((r["tracking_error_rad"] for r in rows
                             if r["phase"] == "clip"), default=0.)
        leg = max((r["leg_q0_deviation_rad"] for r in rows), default=0.)
        return_diagnostics = q0_return_diagnostics(
            rows, q0, np.asarray(final["all_q"]), controlled_return
        )
        recoverable_abort = bool(
            classify_runtime_exception(execution_exception, self.hackathon_runtime)
            == "RECOVERABLE_ABORT"
            and self.writer.weight == 0.
        )
        recovery_status = None
        if recoverable_abort:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                try:
                    self.runtime.sample("RECOVERY_POSTFLIGHT", q0)
                    self.preflight()
                    recovery_status = self.status()
                    break
                except RuntimeError:
                    time.sleep(.05)
            recoverable_abort = bool(
                recovery_status
                and recovery_status.get("ownership_safe") is True
                and recovery_status.get("external_writers") == 0
                and recovery_status.get("weight") == 0.
                and "status_error" not in recovery_status
            )
        classification = (
            "RECOVERABLE_ABORT" if recoverable_abort else
            "HARD_FAULT" if hard_fault else
            "DEGRADED_SUCCESS" if not return_diagnostics["returned_to_q0"] else
            "SUCCESS"
        )
        self.execution_index += 1
        result = {
            "schema": "motiondecode-test.resident-result.v1", "reaction": reaction,
            "status": ("recoverable_abort" if recoverable_abort else
                       "pass" if completed and hard_fault is None else "fail"),
            "classification": classification,
            "executed": self.writer.sent > sent_before, "motion_completed": completed,
            "released": self.writer.weight == 0.,
            "returned_to_q0": return_diagnostics["returned_to_q0"],
            "hard_fault": hard_fault, "metadata": metadata,
            "recoverable_reason": hard_fault if recoverable_abort else None,
            "recovery_status": recovery_status,
            "reaction_engine_trigger_monotonic_s": request.get("trigger_monotonic_s") if request else None,
            "worker_receive_monotonic_s": received,
            "acquire_start_monotonic_s": acquire_start,
            "clip_start_monotonic_s": clip_start,
            "execution_completed_monotonic_s": finished,
            "worker_to_acquire_s": acquire_start - received,
            "trigger_to_clip_s": None if clip_start is None else clip_start - received,
            "max_tracking_error_rad": tracking,
            "max_tracking_error_during_clip_rad": clip_tracking,
            "max_leg_q0_deviation_rad": leg,
            "commands_sent": self.writer.sent - sent_before,
            "publisher_persisted": True, "weight_zero": self.writer.weight == 0.,
            # Backward-compatible field now follows the controlled-return gate.
            "q0_return_error_rad": return_diagnostics["controlled_q0_return_error_rad"],
            **return_diagnostics,
        }
        write_json(self.folder / f"result_{self.execution_index:03d}_{reaction}.json", result)
        if hard_fault and not recoverable_abort:
            raise RuntimeError(hard_fault)
        return result

    def stop(self) -> None:
        if self.writer is not None:
            try:
                if self.writer.weight != 0. and self.runtime is not None:
                    self.runtime.abort_release()
            finally:
                self.writer.close(); self.writer = None
        if self.watchdog is not None: self.watchdog.stop(); self.watchdog = None
        if self.runtime is not None: self.runtime.close_logs(); self.runtime = None
        if self.reader is not None: self.reader.close(); self.reader = None
        if self.lock_file is not None: self.lock_file.close(); self.lock_file = None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--real", action="store_true")
    parser.add_argument("--confirm-site-ready", action="store_true")
    parser.add_argument("--hackathon-runtime", action="store_true")
    parser.add_argument("--socket", type=Path, default=Path("/tmp/motiondecode-reaction.sock"))
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--expected-arm-pid", type=int)
    parser.add_argument("--lowstate-backend", choices=("unitree", "cyclonedds"),
                        default="unitree")
    parser.add_argument("--discovery-peer", action="append", default=[])
    parser.add_argument("--output-root", type=Path, default=ROOT / "output" / "resident_worker")
    parser.add_argument("--reaction", action="append", choices=sorted(REACTION_SPECS),
                        help="Preload only this reaction; repeat as needed")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.real and not args.confirm_site_ready:
        raise SystemExit("real resident worker requires --confirm-site-ready")
    if args.real and args.expected_arm_pid is None:
        raise SystemExit("real resident worker requires PID from the startup identity probe")
    reactions = set(args.reaction or REACTION_SPECS)
    backend = (DryRunBackend() if args.dry_run else
               RealResidentBackend(args.network_interface, args.expected_arm_pid,
                                   args.discovery_peer, args.output_root, reactions,
                                   args.lowstate_backend, args.hackathon_runtime))
    controller = ResidentController(backend, reactions)
    controller.start()
    server = JsonUnixServer(args.socket, controller)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: server.stopping.set())
    print(json.dumps({"state": "READY", "socket": str(args.socket),
                      **controller.status()}, sort_keys=True), flush=True)
    try:
        server.serve()
    finally:
        if controller.state != "STOPPING": backend.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
