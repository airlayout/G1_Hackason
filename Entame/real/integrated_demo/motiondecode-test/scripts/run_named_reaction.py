#!/usr/bin/env python3
"""Run an allowlisted MotionDecode reaction through the validated G1 runner."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / "config" / "named_reactions.json"
RESULT_PREFIX = "MOTIONDECODE_NAMED_RESULT="
MANUAL_GATE = "RUN_NAMED_FRUSTRATION"


def load_profile(name: str) -> dict[str, Any]:
    data = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    profiles = data.get("reactions", {})
    if name not in profiles:
        raise ValueError(f"Unknown named reaction: {name}")
    profile = profiles[name]
    if not profile.get("real_g1_validated"):
        raise ValueError(f"Named reaction is not validated for real G1: {name}")
    return profile


def public_profile(name: str, profile: dict[str, Any]) -> dict[str, Any]:
    return {
        "reaction": name,
        "source": profile["source"],
        "arms_amplitude": profile["arms_amplitude"],
        "waist_amplitude": profile["waist_amplitude"],
        "legs": profile["legs"],
        "requires_stop": profile["requires_stop"],
        "real_g1_validated": profile["real_g1_validated"],
    }


def runner_command(args: argparse.Namespace, profile: dict[str, Any], run_dir: Path) -> list[str]:
    command = [
        sys.executable,
        str(ROOT / "scripts" / "run_real_reaction.py"),
        "--stage", profile["stage"],
        "--motion-layout", profile["motion_layout"],
        "--waist-scale", str(profile["waist_amplitude"]),
        "--network-interface", args.network_interface,
        "--expected-arm-pid", str(args.expected_arm_pid),
        "--run-dir", str(run_dir),
        "--execute",
        "--confirm-site-ready",
        "--previous-stage-result", str(ROOT / profile["previous_stage_result"]),
        "--confirm-previous-stage-observed",
        "--acquire-ramp-s", str(profile["acquire_ramp_seconds"]),
    ]
    if profile.get("prevalidated_runtime"):
        command.append("--prevalidated-runtime")
    for peer in args.discovery_peer:
        command.extend(["--discovery-peer", peer])
    if not args.engine_authorized:
        command.extend(["--final-gate-token", MANUAL_GATE])
    return command


def remote_command(args: argparse.Namespace) -> list[str]:
    nested = [
        "python3", "scripts/run_named_reaction.py", args.reaction,
        "--real", "--transport", "local",
        "--confirm-site-ready",
        "--network-interface", args.network_interface,
        "--expected-arm-pid", str(args.expected_arm_pid),
        "--timeout", str(args.timeout),
    ]
    for peer in args.discovery_peer:
        nested.extend(["--discovery-peer", peer])
    if args.engine_authorized:
        nested.extend(["--engine-authorized", "--json"])
    env = (
        f"PYTHONPATH={shlex.quote(args.remote_pythonpath)} "
        f"LD_LIBRARY_PATH={shlex.quote(args.remote_library_path)}"
    )
    remote = f"cd {shlex.quote(args.remote_root)} && {env} {shlex.join(nested)}"
    command = [
        "ssh", "-T", "-o", "StrictHostKeyChecking=yes", "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=2",
        "-o", "ServerAliveCountMax=2",
    ]
    if args.ssh_control:
        command.extend(["-S", args.ssh_control])
    command.extend(["--", args.ssh_target, remote])
    return command


def parse_remote_result(stdout: str) -> dict[str, Any]:
    candidates = [line for line in stdout.splitlines() if line.strip().startswith("{")]
    if len(candidates) != 1:
        raise RuntimeError(f"Expected one named-reaction JSON result, found {len(candidates)}")
    result = json.loads(candidates[0])
    if not isinstance(result, dict):
        raise RuntimeError("Named-reaction result must be an object")
    return result


def run_local(args: argparse.Namespace, profile: dict[str, Any]) -> dict[str, Any]:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = ROOT / "output" / f"named_{args.reaction}_{stamp}"
    command = runner_command(args, profile, run_dir)
    if args.json:
        completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, timeout=args.timeout)
    else:
        print("NAMED MOTIONDECODE REACTION", flush=True)
        print(json.dumps(public_profile(args.reaction, profile), ensure_ascii=False, indent=2), flush=True)
        completed = subprocess.run(command, cwd=ROOT, text=True, timeout=args.timeout)
    result_path = run_dir / "stage_result.json"
    stage = json.loads(result_path.read_text(encoding="utf-8")) if result_path.exists() else {}
    passed = completed.returncode == 0 and stage.get("status") == "PASS"
    return {
        "reaction": args.reaction,
        "status": "pass" if passed else "fail",
        "executed": bool(stage.get("executed", False)),
        "released": bool(stage.get("ownership_released", False)),
        "returned_to_q0": bool(stage.get("returned_to_q0", False)),
        "run_dir": str(run_dir),
        "runner_returncode": completed.returncode,
        "reason": stage.get("reason"),
    }


def run_remote(args: argparse.Namespace) -> dict[str, Any]:
    capture = args.engine_authorized
    completed = subprocess.run(
        remote_command(args), text=True, capture_output=capture, timeout=args.timeout + 30.0
    )
    if completed.returncode != 0:
        detail = ((completed.stderr or completed.stdout) if capture else "remote named reaction failed")[-2000:].strip()
        return {
            "reaction": args.reaction,
            "status": "fail",
            "executed": False,
            "released": False,
            "runner_returncode": completed.returncode,
            "reason": detail,
        }
    if not capture:
        return {
            "reaction": args.reaction,
            "status": "pass",
            "executed": True,
            "released": True,
            "returned_to_q0": True,
            "runner_returncode": 0,
        }
    return parse_remote_result(completed.stdout)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reaction")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--real", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--transport", choices=("local", "ssh"), default="local")
    parser.add_argument("--confirm-site-ready", action="store_true")
    parser.add_argument("--engine-authorized", action="store_true")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--discovery-peer", action="append", default=[])
    parser.add_argument("--expected-arm-pid", type=int, default=2899)
    parser.add_argument("--timeout", type=float, default=420.0)
    parser.add_argument("--ssh-target", default="unitree@10.42.0.76")
    parser.add_argument("--ssh-control")
    parser.add_argument("--remote-root", default="/tmp/motiondecode-current")
    parser.add_argument("--remote-pythonpath", default="/tmp/motiondecode-hold-deps")
    parser.add_argument("--remote-library-path", default="/home/unitree/work/unitree_sdk2/thirdparty/lib/aarch64")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        profile = load_profile(args.reaction)
        if args.dry_run:
            result = {
                **public_profile(args.reaction, profile),
                "status": "pass",
                "executed": False,
                "released": False,
                "transport": args.transport,
            }
        else:
            if not args.confirm_site_ready:
                raise ValueError("Real execution requires --confirm-site-ready")
            if args.json and not args.engine_authorized:
                raise ValueError("Attended real execution uses the interactive final gate; omit --json")
            if args.timeout <= 0:
                raise ValueError("--timeout must be positive")
            result = run_local(args, profile) if args.transport == "local" else run_remote(args)
    except Exception as exc:
        result = {
            "reaction": args.reaction,
            "status": "fail",
            "executed": False,
            "released": False,
            "reason": str(exc),
        }
    payload = json.dumps(result, ensure_ascii=False, sort_keys=True)
    print(payload if args.json else RESULT_PREFIX + payload, flush=True)
    return 0 if result["status"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
