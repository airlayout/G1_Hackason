#!/usr/bin/env python3
"""Offline comparison of trimmed named reactions at 1.00/1.25/1.50x."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from common import write_json
from reaction_profiles import build_named_paths, load_trajectory_profile
from run_real_reaction import evaluate_paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--q0", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    snapshot = json.loads(args.q0.read_text(encoding="utf-8"))
    report = {"schema": "motiondecode-test.fast-game-reactions.v1", "receive_only": True,
              "command_publishers_created": 0, "reactions": {}}
    for name in ("surprise", "found", "joy"):
        profile = load_trajectory_profile(name)
        variants = []
        for speed in (1.0, 1.25, 1.5):
            paths, _, durations, metadata = build_named_paths(
                snapshot, name, 0.5, 0.25, playback_speed=speed
            )
            safety = evaluate_paths(paths, durations, snapshot, motion_layout="upper-body")
            variants.append({"speed": speed, "duration_s": durations["clip"],
                             "metadata": metadata, "safety": safety})
        selected = next(
            item for item in variants if item["speed"] == float(profile["playback_speed"])
        )
        report["reactions"][name] = {
            "source": profile["source"], "variants": variants,
            "selected_speed": selected["speed"],
            "selected_duration_s": selected["duration_s"],
            "selected_passed": selected["safety"]["passed"],
        }
        print(name, selected["speed"], selected["duration_s"], selected["safety"]["passed"], flush=True)
    write_json(args.output, report)
    return 0 if all(row["selected_passed"] for row in report["reactions"].values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
