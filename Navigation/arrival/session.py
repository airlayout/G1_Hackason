"""Append-only acquisition files and a fixed 3 x 10 attempt inventory."""
from __future__ import annotations

import json
import math
import fcntl
from pathlib import Path
import time


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+"\n")


def read_lines(path):
    p = Path(path)
    if not p.exists():
        return []
    rows = []
    for line_no, line in enumerate(p.read_text().splitlines(), 1):
        if line.strip():
            try:
                rows.append(json.loads(line))
            except ValueError as exc:
                raise ValueError(f"{p}:{line_no}: invalid JSON; preserve/recover the incomplete recording") from exc
    return rows


class Writer:
    def __init__(self, root, stream):
        path = Path(root)/"streams"/f"{stream}.jsonl"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("a")
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.file.close()
            raise ValueError(f"another recorder owns stream {stream}") from exc

    def append(self, value):
        self.file.write(json.dumps(value, ensure_ascii=False, allow_nan=False)+"\n")
        self.file.flush()

    def close(self):
        self.file.close()


def initialize(root, data_kind="real"):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    if (root/"session.json").exists():
        raise ValueError("session already exists; use another output directory")
    manifest = {
        "schema_version": 1, "data_kind": data_kind,
        "clock": {"id": "unix", "synchronized": False, "max_offset_s": None},
        "target": {"id": "panel_1", "button": "up", "nav_goal": None},
        "reference_trial_id": "reference",
        "trials": [{"id": f"route_{route}_{attempt:02d}", "route": f"route_{route}", "attempt": attempt}
                   for attempt in range(1, 11) for route in range(1, 4)],
        "calibration": "calibration.json",
        "analysis": {"body_max_gap_s": .1, "pose_max_gap_s": .3,
                     "post_arrival_s": 3., "stop_dwell_s": .5,
                     "leg_speed_rad_s": .05, "odom_speed_m_s": .02,
                     "odom_yaw_speed_rad_s": .05},
    }
    write_json(root/"session.json", manifest)
    # Deliberately unusable until measured; no virtual camera mount is imported.
    write_json(root/"calibration.json", {
        "id": "UNMEASURED", "camera": "head_camera", "optical_frame": "head_optical",
        "measured": False, "joints": [], "T_last_optical": None,
        "translation_uncertainty_m": None, "rotation_uncertainty_rad": None,
        "T_pelvis_imu": None, "T_base_pelvis": None, "navigation_base_frame": "base_link",
    })
    write_json(root/"annotations.json", {"frames": []})
    return manifest


def trial_windows(root, manifest):
    ids = {manifest["reference_trial_id"], *(t["id"] for t in manifest["trials"])}
    windows, active, previous_time = {}, None, -float("inf")
    for event in read_lines(Path(root)/"streams/operator.jsonl"):
        if not math.isfinite(event["t"]) or event["t"] < previous_time:
            raise ValueError("operator clock moved backwards across attempts")
        previous_time = event["t"]
        trial_id = event["trial_id"]
        if trial_id not in ids:
            raise ValueError(f"unknown trial {trial_id}")
        if event["event"] == "start":
            if active is not None or trial_id in windows:
                raise ValueError("overlapping/reused trial markers; start a new session for repeats")
            windows[trial_id] = {"start": event["t"], "end": None, "goal_id": event.get("goal_id"), "manual": False, "manual_reasons": []}
            active = trial_id
        else:
            if active != trial_id:
                raise ValueError("operator event does not belong to active trial")
            w = windows[trial_id]
            if event["t"] < w["start"]:
                raise ValueError("operator clock moved backwards")
            if event["event"] == "bind_goal":
                w["goal_id"] = event["goal_id"]
            elif event["event"] == "manual_intervention":
                w["manual"] = True
                w["manual_reasons"].append(event.get("reason", ""))
            elif event["event"] == "end":
                w.update(end=event["t"], end_reason=event.get("reason", ""))
                active = None
            else:
                raise ValueError("unknown operator event")
    return windows


def mark(root, trial_id, event, goal_id=None, reason="", stamp=None):
    root = Path(root)
    manifest = read_json(root/"session.json")
    writer = Writer(root, "operator")
    try:
        windows = trial_windows(root, manifest)
        ids = {manifest["reference_trial_id"], *(t["id"] for t in manifest["trials"])}
        if trial_id not in ids:
            raise ValueError("trial is not in the session inventory")
        active = next((i for i, w in windows.items() if w["end"] is None), None)
        if event == "start" and (active is not None or trial_id in windows):
            raise ValueError("another trial is active or this attempt has already been recorded")
        if event != "start" and active != trial_id:
            raise ValueError("mark requires the active trial")
        if event == "bind_goal" and not goal_id:
            raise ValueError("bind_goal requires --goal-id")
        now = time.time() if stamp is None else float(stamp)
        previous_marks = read_lines(root/"streams/operator.jsonl")
        if not math.isfinite(now) or (previous_marks and now < previous_marks[-1]["t"]):
            raise ValueError("invalid/backwards marker clock")
        writer.append({"trial_id": trial_id, "event": event, "t": now, "goal_id": goal_id, "reason": reason})
    finally:
        writer.close()
