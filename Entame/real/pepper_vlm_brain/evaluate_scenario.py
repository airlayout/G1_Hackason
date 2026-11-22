"""Read-only post-run checks. Scenario expectations never control inference."""
import argparse
import json
from pathlib import Path
from run_video import write_report
from brain.decision import parse_decision
from brain.memory import Memory
from brain.schemas import Direction, wait_decision


def evaluate(report, scenario):
    checks = []
    integrity_errors = []
    if report.get("status") != "pass": integrity_errors.append("Pipeline did not pass")
    if report.get("timestamps") != scenario["timestamps"]: integrity_errors.append("Timestamp schedule mismatch")
    if [f.get("timestamp") for f in report["frames"]] != scenario["timestamps"]:
        integrity_errors.append("Evaluated frame schedule mismatch")
    if scenario.get("require_observation") and not report.get("combined_output"):
        integrity_errors.append("Explicit Observation protocol required")
    previous = None
    for frame in report["frames"]:
        if report.get("temporal_enabled"):
            expected_previous = previous["timestamp"] if previous else None
            if frame.get("previous_selected_timestamp") != expected_previous:
                integrity_errors.append(f"Previous selected frame mismatch at {frame['timestamp']}")
            if frame.get("metrics", {}).get("image_count") != (2 if previous else 1):
                integrity_errors.append(f"Temporal image count mismatch at {frame['timestamp']}")
        parsed = parse_decision(frame["raw_response"], combined=report.get("combined_output", False))
        rejected = parsed.fallback or frame.get("fallback", False)
        if rejected:
            integrity_errors.append(f"Rejected model output/fallback at {frame['timestamp']}: {frame.get('error') or parsed.error}")
            if frame["decision"] != wait_decision().model_dump(mode="json", exclude_none=True):
                integrity_errors.append(f"Fallback Action mismatch at {frame['timestamp']}")
            if frame.get("observation") is not None:
                integrity_errors.append(f"Rejected Observation was accepted at {frame['timestamp']}")
        elif parsed.decision.model_dump(mode="json", exclude_none=True) != frame["decision"]:
            integrity_errors.append(f"Raw/Decision mismatch at {frame['timestamp']}")
        if report.get("combined_output"):
            if not rejected and (parsed.observation is None or parsed.observation.model_dump(mode="json", exclude_none=True) != frame.get("observation")):
                integrity_errors.append(f"Raw/Observation mismatch at {frame['timestamp']}")
            if frame.get("memory_before") and frame.get("memory_after"):
                before = frame["memory_before"]
                reconstructed = Memory(last_seen_person_direction=Direction(before["last_seen_person_direction"]),
                                       person_visible_last_frame=before["person_visible_last_frame"],
                                       frames_since_person_seen=before["frames_since_person_seen"])
                reconstructed.update(wait_decision() if rejected else parsed.decision,
                                     None if rejected else parsed.observation)
                if reconstructed.context() != frame["memory_after"]:
                    integrity_errors.append(f"Observation/Memory mismatch at {frame['timestamp']}")
        if previous is not None and frame.get("memory_before") != previous.get("memory_after"):
            integrity_errors.append(f"Memory continuity mismatch at {frame['timestamp']}")
        previous = frame
    for rule in scenario["checks"]:
        matches = [f for f in report["frames"] if abs(f["timestamp"] - rule["timestamp"]) < 1e-6]
        errors = []
        if len(matches) != 1:
            errors.append("Expected one evaluated frame")
        else:
            frame = matches[0]
            decision = frame["decision"]
            if frame["fallback"]: errors.append("Inference/parser fallback")
            if decision["action"] not in rule["allowed_actions"]: errors.append("Action mismatch")
            if decision["target"] != rule["target"]: errors.append("Target mismatch")
            direction = rule.get("direction")
            if "direction_from_memory" in rule:
                direction = (frame.get("memory_before") or {}).get(rule["direction_from_memory"])
                if direction in (None, "UNKNOWN"): errors.append("No previous known direction")
            if decision["direction"] != direction: errors.append("Direction mismatch")
            if report.get("combined_output"):
                obs = frame.get("observation") or {}
                if obs.get("person_visible") != rule["person_visible_after"]:
                    errors.append("Observation person_visible mismatch")
                if rule["person_visible_after"] and "direction" in rule and obs.get("person_direction") != rule["direction"]:
                    errors.append("Observation person_direction mismatch")
                if "person_transition" in rule and obs.get("person_transition") != rule["person_transition"]:
                    errors.append("Observation transition mismatch")
            elif scenario.get("require_observation"):
                errors.append("Explicit Observation required")
            memory = frame.get("memory_after") or {}
            if memory.get("person_visible_last_frame") != rule["person_visible_after"]:
                errors.append("Memory visible state mismatch")
            if rule["person_visible_after"] and memory.get("frames_since_person_seen") != 0:
                errors.append("Reappearance did not reset missed-frame count")
            if not rule["person_visible_after"]:
                before = frame.get("memory_before") or {}
                if memory.get("last_seen_person_direction") != before.get("last_seen_person_direction"):
                    errors.append("Previous direction was not retained after disappearance")
                if memory.get("frames_since_person_seen") != before.get("frames_since_person_seen", 0) + 1:
                    errors.append("Missed-frame count did not advance")
            if not frame.get("memory_before"): errors.append("Memory not enabled")
            if rule.get("require_previous_absence"):
                before = frame.get("memory_before") or {}
                if before.get("person_visible_last_frame") is not False or before.get("frames_since_person_seen", 0) < 1:
                    errors.append("Reappearance transition cannot pass without previous detected absence")
        checks.append({"timestamp": rule["timestamp"], "observation": rule["observation"],
                       "pass": not errors, "errors": errors})
    return {"scenario": scenario["name"], "pass": not integrity_errors and all(c["pass"] for c in checks),
            "integrity_errors": integrity_errors, "checks": checks}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("scenario", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = evaluate(json.loads(args.report.read_text(encoding="utf-8")),
                      json.loads(args.scenario.read_text(encoding="utf-8")))
    print(json.dumps(result, indent=2))
    if args.output: write_report(args.output, result)
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
