"""Offline measurements and reports. Missing evidence stays null, never zero."""
from __future__ import annotations

import csv
import hashlib
import math
from pathlib import Path
from collections import Counter

import numpy as np

from .geometry import (Unmeasured, camera_transform, interpolate_state, kinematic_chain, panel_pose,
                       pose_matrix, quaternion, transform, wrap)
from .session import read_json, read_lines, trial_windows, write_json


def synchronized(manifest):
    clock = manifest["clock"]
    offset = clock.get("max_offset_s")
    return (clock.get("synchronized") is True and isinstance(offset, (float, int))
            and math.isfinite(offset) and 0 <= offset <= manifest["analysis"]["body_max_gap_s"]/2)


def measure_frame(root, manifest, calibration, frame, annotation, body):
    if not synchronized(manifest):
        raise Unmeasured("clock synchronization is unverified/outside interpolation budget")
    if frame.get("stamp_kind") != "exposure" or frame.get("clock_id") != manifest["clock"]["id"]:
        raise Unmeasured("camera exposure clock is unverified")
    if frame["t"] > frame["received_at"] + manifest["clock"]["max_offset_s"]:
        raise Unmeasured("camera exposure is in the future")
    if not frame.get("depth_aligned"):
        raise Unmeasured("depth is not aligned to RGB")
    if frame.get("camera") != calibration.get("camera") or frame.get("optical_frame") != calibration.get("optical_frame"):
        raise Unmeasured("camera/frame does not match calibration")
    gap = manifest["analysis"]["body_max_gap_s"]
    nearby = [s for s in body if abs(s["t"]-frame["t"]) <= gap]
    if not nearby or any(s.get("clock_id") != manifest["clock"]["id"] for s in nearby):
        raise Unmeasured("body clock mismatch/missing")
    if any(s.get("time_uncertainty_s") is None or not math.isfinite(s["time_uncertainty_s"])
           or not 0 <= s["time_uncertainty_s"] <= gap/2 for s in nearby):
        raise Unmeasured("body timestamp uncertainty is unmeasured/too large")
    state = interpolate_state(nearby, frame["t"], gap)
    T = camera_transform(calibration, state["q"])
    if calibration.get("T_pelvis_imu") is None and calibration.get("T_last_imu") is None:
        raise Unmeasured("IMU mounting link/transform is unmeasured")
    imu_mount = (kinematic_chain(calibration.get("imu_joints", []), calibration["T_last_imu"], state["q"])
                 if calibration.get("T_last_imu") is not None else transform(calibration["T_pelvis_imu"]))
    up = imu_mount[:3, :3] @ quaternion(state["imu_wxyz"]).T @ [0., 0., 1.]
    path = Path(root)/frame["path"]
    try:
        with np.load(path, allow_pickle=False) as data:
            if data["rgb"].shape[:2] != data["depth_m"].shape:
                raise Unmeasured("RGB/depth shapes differ")
            result = panel_pose(data["depth_m"], data["K"], annotation, T, up)
    except (OSError, KeyError) as exc:
        raise Unmeasured(f"missing frame arrays: {exc}") from exc
    result.update(t=frame["t"], frame_id=frame["id"], waist_rad=state["q"][12:15].tolist(),
                  body_stamp_kinds=state["stamp_kinds"], calibration_id=calibration.get("id"),
                  translation_uncertainty_m=calibration.get("translation_uncertainty_m"),
                  rotation_uncertainty_rad=calibration.get("rotation_uncertainty_rad"))
    return result


def pose_at(events, t, gap, kind="nav_pose", frame_id=None):
    rows = sorted((s for s in events if s["kind"] == kind and
                   (frame_id is None or s.get("frame_id") == frame_id)), key=lambda s: s["t"])
    if not rows:
        raise Unmeasured(f"missing {kind}")
    times = np.array([s["t"] for s in rows])
    i = int(np.searchsorted(times, t))
    if i < len(rows) and times[i] == t:
        a = b = rows[i]
        ratio = 0.
    else:
        if i == 0 or i == len(rows) or times[i]-times[i-1] > gap:
            raise Unmeasured(f"{kind} does not bracket timestamp")
        a, b = rows[i-1:i+1]
        if a["frame_id"] != b["frame_id"] or a["child_frame_id"] != b["child_frame_id"]:
            raise Unmeasured("navigation frame changed during interpolation")
        ratio = (t-times[i-1])/(times[i]-times[i-1])
    A, B = pose_matrix(a["xyz"], a["xyzw"]), pose_matrix(b["xyz"], b["xyzw"])
    # Navigation is explicitly floor-frame SE(2); do not quietly use it as a pelvis pose.
    yaw_a, yaw_b = math.atan2(A[1, 0], A[0, 0]), math.atan2(B[1, 0], B[0, 0])
    yaw = yaw_a + ratio*wrap(yaw_b-yaw_a)
    T = np.eye(4)
    T[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    T[:3, 3] = A[:3, 3]+ratio*(B[:3, 3]-A[:3, 3])
    return T, a["frame_id"], a["child_frame_id"]


def navigation_result(events, window):
    statuses = [e for e in events if e["kind"] == "nav_status"]
    goal_id = window.get("goal_id")
    if not goal_id:
        candidates = {e["goal_id"] for e in statuses
                      if window["start"] <= e.get("goal_time", -1) <= window["end"]}
        if len(candidates) != 1:
            return "unmeasured", None, "goal UUID is missing/ambiguous"
        goal_id = next(iter(candidates))
    terminal = sorted((e for e in statuses if e["goal_id"] == goal_id and
                       window["start"] <= e.get("goal_time", -1) <= window["end"] and
                       e["status"] in ("succeeded", "aborted", "canceled")), key=lambda e: e["t"])
    if not terminal:
        return "unmeasured", None, "no terminal status for bound goal"
    last = terminal[0]
    return last["status"], last["t"] if last["status"] == "succeeded" else None, ""


def stop_time(events, body, arrival, settings):
    """Observed settling metric, not a runtime motion permission or safety guarantee."""
    start = None
    previous = None
    for sample in sorted((s for s in body if s["t"] >= arrival), key=lambda s: s["t"]):
        t = sample["t"]
        dq = np.asarray(sample.get("dq", []), float)
        try:
            pose, _, _ = pose_at(events, t, settings["pose_max_gap_s"], "odom")
            before, _, _ = pose_at(events, t-.2, settings["pose_max_gap_s"], "odom")
            delta = np.linalg.inv(before) @ pose
            still = (dq.shape == (29,) and np.isfinite(dq).all()
                     and np.max(np.abs(dq[:12])) <= settings["leg_speed_rad_s"]
                     and np.linalg.norm(delta[:2, 3])/.2 <= settings["odom_speed_m_s"]
                     and abs(math.atan2(delta[1, 0], delta[0, 0]))/.2 <= settings["odom_yaw_speed_rad_s"])
        except Unmeasured:
            still = False
        if previous is not None and t-previous > settings["body_max_gap_s"]+1e-8:
            start = None
        if not still:
            start = None
        elif start is None:
            start = t
        if start is not None and t-start >= settings["stop_dwell_s"]-1e-8:
            return t  # Confirmation time includes the entire dwell interval.
        previous = t
    return None


def post_arrival(events, body, arrival, end, settings):
    result = {"post_motion_max_m": None, "post_yaw_max_rad": None, "imu_sway_max_rad": None}
    finish = arrival+settings["post_arrival_s"]
    if finish > end:
        return result
    try:
        initial, frame, child = pose_at(events, arrival, settings["pose_max_gap_s"], "odom")
        samples = sorted((s for s in events if s["kind"] == "odom" and arrival <= s["t"] <= finish), key=lambda s: s["t"])
        bounds = [arrival, *(s["t"] for s in samples), finish]
        if np.max(np.diff(bounds)) > settings["pose_max_gap_s"] or not samples:
            raise Unmeasured("incomplete post-arrival odometry window")
        final, _, _ = pose_at(events, finish, settings["pose_max_gap_s"], "odom", frame)
        if any(s["frame_id"] != frame or s["child_frame_id"] != child for s in samples):
            raise Unmeasured("post-arrival odometry frame changed")
        deltas = [np.linalg.inv(initial) @ pose_matrix(s["xyz"], s["xyzw"]) for s in samples]
        deltas.append(np.linalg.inv(initial) @ final)
        result["post_motion_max_m"] = max(float(np.linalg.norm(d[:3, 3])) for d in deltas)
        result["post_yaw_max_rad"] = max(abs(math.atan2(d[1, 0], d[0, 0])) for d in deltas)
    except Unmeasured:
        pass
    samples = sorted((s for s in body if arrival <= s["t"] <= finish), key=lambda s: s["t"])
    if len(samples) > 1 and max(np.diff([arrival, *(s["t"] for s in samples), finish])) <= settings["body_max_gap_s"]+1e-8:
        try:
            R = quaternion(samples[0]["imu_wxyz"])
            result["imu_sway_max_rad"] = max(math.acos(float(np.clip((np.trace(R.T @ quaternion(s["imu_wxyz"]))-1)/2, -1, 1))) for s in samples)
        except Unmeasured:
            pass
    return result


def statistics(values, circular=False):
    a = np.asarray([v for v in values if v is not None], float)
    if not len(a):
        return {"n": 0, "mean": None, "std": None, "abs_p95": None, "abs_max": None}
    mean, std = float(a.mean()), float(a.std())
    if circular:
        sine, cosine = float(np.sin(a).mean()), float(np.cos(a).mean())
        resultant = min(1., math.hypot(sine, cosine))
        mean = math.atan2(sine, cosine) if resultant > 1e-8 else None
        std = math.sqrt(max(0., -2*math.log(resultant))) if resultant > 1e-8 else None
    return {"n": len(a), "mean": mean, "std": std,
            "abs_p95": float(np.percentile(np.abs(a), 95)), "abs_max": float(np.max(np.abs(a)))}


def analyze(root, output=None):
    root = Path(root)
    manifest = read_json(root/"session.json")
    if manifest["schema_version"] != 1 or manifest["data_kind"] not in ("real", "synthetic"):
        raise ValueError("unsupported session version/data provenance")
    ids = [t["id"] for t in manifest["trials"]]
    if len(ids) != len(set(ids)) or manifest["reference_trial_id"] in ids:
        raise ValueError("duplicate/reference attempt IDs")
    calibration = read_json(root/manifest["calibration"])
    if calibration.get("data_kind", manifest["data_kind"]) != manifest["data_kind"]:
        raise ValueError("calibration/session provenance mismatch")
    windows = trial_windows(root, manifest)
    events = read_lines(root/"streams/ros.jsonl")
    body = read_lines(root/"streams/body.jsonl")
    timed_body = [s for s in body if s.get("clock_id") == manifest["clock"]["id"]
                  and isinstance(s.get("time_uncertainty_s"), (int, float))
                  and math.isfinite(s["time_uncertainty_s"])
                  and 0 <= s["time_uncertainty_s"] <= manifest["analysis"]["body_max_gap_s"]/2]
    frames = read_lines(root/"streams/frames.jsonl")
    annotations = read_json(root/"annotations.json")["frames"]
    if len({a["frame_id"] for a in annotations}) != len(annotations) or len({f["id"] for f in frames}) != len(frames):
        raise ValueError("duplicate frame/annotation identifiers")
    annotation_map = {a["frame_id"]: a for a in annotations}
    frame_map = {f["id"]: f for f in frames}
    if set(annotation_map)-set(frame_map):
        raise ValueError("annotation references an unknown frame")
    measurements, issues, exposures = {}, {}, set()
    duplicate_exposures = set()
    for frame in frames:
        key = (frame.get("camera"), frame["t"])
        if key in exposures:
            duplicate_exposures.add(frame["id"])
        exposures.add(key)
    for frame_id, annotation in annotation_map.items():
        try:
            if frame_id in duplicate_exposures:
                raise Unmeasured("repeated camera exposure")
            measurements[frame_id] = measure_frame(root, manifest, calibration, frame_map[frame_id], annotation, body)
        except (Unmeasured, KeyError, TypeError, IndexError) as exc:
            issues[frame_id] = str(exc)

    def in_window(rows, window):
        # Half-open windows avoid counting frames twice when attempts touch.
        return [r for r in rows if window["start"] <= r["t"] < window["end"]]

    reference = None
    ref_window = windows.get(manifest["reference_trial_id"])
    if ref_window and ref_window["end"] is not None and not ref_window["manual"]:
        reference_stopped = (stop_time(in_window(events, ref_window), in_window(timed_body, ref_window),
                                      ref_window["start"], manifest["analysis"])
                             if synchronized(manifest) else None)
        candidates = sorted((m for m in in_window(list(measurements.values()), ref_window)
                             if reference_stopped is not None and m["t"] >= reference_stopped), key=lambda m: m["t"])
        if candidates:
            reference = candidates[0]
    results = []
    settings = manifest["analysis"]
    for trial in manifest["trials"]:
        row = dict(trial, data_kind=manifest["data_kind"], record_status="not_recorded", nav_outcome="unmeasured",
                   manual_intervention=False, arrival_t=None, stop_confirmed_t=None, stop_delay_s=None,
                   stand_off_m=None, lateral_m=None, height_m=None, heading_rad=None,
                   delta_normal_m=None, delta_lateral_m=None, delta_height_m=None, delta_heading_rad=None,
                   nav_goal_xy_error_m=None, nav_goal_yaw_error_rad=None,
                   nav_local_discrepancy_m=None, nav_local_heading_discrepancy_rad=None,
                   post_motion_max_m=None, post_yaw_max_rad=None, imu_sway_max_rad=None,
                   measured_frames=0, annotated_frames=0, captured_frames=0,
                   waist_yaw_rad=None, waist_roll_rad=None, waist_pitch_rad=None,
                   plane_rms_m=None, measurement_t=None, settled_measurement=False, quality=[])
        window = windows.get(trial["id"])
        if window is None:
            row["quality"].append("attempt has not been recorded")
            results.append(row)
            continue
        if window["end"] is None:
            row["record_status"] = "incomplete"
            row["quality"].append("trial has no end marker")
            results.append(row)
            continue
        e, b = in_window(events, window), in_window(body, window)
        timed_b = in_window(timed_body, window)
        fs = in_window(frames, window)
        row.update(record_status="recorded", manual_intervention=window["manual"],
                   captured_frames=len(fs), annotated_frames=sum(f["id"] in annotation_map for f in fs))
        row["nav_outcome"], arrival, reason = navigation_result(e, window)
        row["arrival_t"] = arrival
        if reason:
            row["quality"].append(reason)
        if window["manual"]:
            row["quality"].append("manual intervention: excluded from automatic-arrival statistics")
            row["quality"].extend("intervention: "+reason for reason in window["manual_reasons"] if reason)
        if window.get("end_reason"):
            row["quality"].append("operator: "+window["end_reason"])
        if not b:
            row["quality"].append("body stream missing")
        elif len(timed_b) != len(b):
            row["quality"].append("some body timestamps have unverified clock/uncertainty")
        if not synchronized(manifest):
            row["quality"].append("clock synchronization unverified: cross-stream measurements unavailable")
        if not fs:
            row["quality"].append("camera stream missing")
        for f in fs:
            if f["id"] in issues:
                row["quality"].append(issues[f["id"]])
        candidates = sorted(in_window(list(measurements.values()), window), key=lambda m: m["t"])
        row["measured_frames"] = len(candidates)
        if arrival is not None:
            confirmed = stop_time(e, timed_b, arrival, settings) if synchronized(manifest) else None
            row["stop_confirmed_t"] = confirmed
            row["stop_delay_s"] = None if confirmed is None else confirmed-arrival
            if confirmed is None:
                row["quality"].append("settling not confirmed in recorded window")
            if synchronized(manifest):
                row.update(post_arrival(e, timed_b, arrival, window["end"], settings))
            settled = [m for m in candidates if confirmed is not None and m["t"] >= confirmed]
        else:
            settled = []
        measured = (settled or candidates or [None])[0]
        if measured:
            row.update({k: measured[k] for k in ("stand_off_m", "lateral_m", "height_m", "heading_rad", "plane_rms_m")})
            row.update(measurement_t=measured["t"], settled_measurement=bool(settled),
                       waist_yaw_rad=measured["waist_rad"][0], waist_roll_rad=measured["waist_rad"][1], waist_pitch_rad=measured["waist_rad"][2])
            if reference:
                for dest, source in [("delta_normal_m", "stand_off_m"), ("delta_lateral_m", "lateral_m"), ("delta_height_m", "height_m")]:
                    row[dest] = measured[source]-reference[source]
                row["delta_heading_rad"] = wrap(measured["heading_rad"]-reference["heading_rad"])
                try:
                    current, frame, child = pose_at(e, measured["t"], settings["pose_max_gap_s"], frame_id="map")
                    ref_pose, _, ref_child = pose_at(events, reference["t"], settings["pose_max_gap_s"], frame_id=frame)
                    if child != ref_child or child != calibration["navigation_base_frame"]:
                        raise Unmeasured("navigation base frame does not match calibration")
                    if calibration.get("T_base_pelvis") is None:
                        raise Unmeasured("floor-base to pelvis transform is unmeasured")
                    BP = transform(calibration["T_base_pelvis"])
                    panel_from_map = transform(reference["T_panel_pelvis"]) @ np.linalg.inv(ref_pose @ BP)
                    predicted = panel_from_map @ current @ BP
                    observed = transform(measured["T_panel_pelvis"])
                    row["nav_local_discrepancy_m"] = float(np.linalg.norm(predicted[:3, 3]-observed[:3, 3]))
                    row["nav_local_heading_discrepancy_rad"] = abs(wrap(math.atan2(predicted[1, 0], predicted[0, 0])-math.atan2(observed[1, 0], observed[0, 0])))
                except Unmeasured as exc:
                    row["quality"].append(str(exc))
            else:
                row["quality"].append("reference relative pose is unmeasured")
        else:
            row["quality"].append("relative pose is unmeasured")
        goal = manifest["target"].get("nav_goal")
        if arrival is not None and goal:
            try:
                current, _, child = pose_at(e, arrival, settings["pose_max_gap_s"], frame_id=goal["frame_id"])
                if child != goal["child_frame_id"]:
                    raise Unmeasured("goal/observed navigation base frames differ")
                delta = np.linalg.inv(pose_matrix(goal["xyz"], goal["xyzw"])) @ current
                row["nav_goal_xy_error_m"] = float(np.linalg.norm(delta[:2, 3]))
                row["nav_goal_yaw_error_rad"] = math.atan2(delta[1, 0], delta[0, 0])
            except Unmeasured as exc:
                row["quality"].append(str(exc))
        row["quality"] = sorted(set(row["quality"]))
        results.append(row)
    eligible = [r for r in results if r["nav_outcome"] == "succeeded" and r["settled_measurement"] and not r["manual_intervention"]]
    summary = {key: statistics([r[key] for r in eligible], circular=key == "delta_heading_rad") for key in
               ("delta_normal_m", "delta_lateral_m", "delta_height_m", "delta_heading_rad", "stop_delay_s", "nav_local_discrepancy_m")}
    report = {"data_kind": manifest["data_kind"], "expected_attempts": len(results),
              "target": manifest["target"], "reference_trial_id": manifest["reference_trial_id"],
              "record_counts": dict(Counter(r["record_status"] for r in results)),
              "outcomes": dict(Counter(r["nav_outcome"] for r in results)),
              "automatic_settled_measurements": len(eligible), "reference": reference,
              "calibration": calibration, "clock": manifest["clock"], "summary": summary,
             "trials": results, "frame_issues": issues,
             "arrival_time_basis": "first observed terminal status of the bound goal UUID",
             "analysis_settings": settings,
             "input_sha256": {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in [root/"session.json", root/manifest["calibration"], root/"annotations.json",
                                         *sorted((root/"streams").glob("*.jsonl")),
                                         *(root/f["path"] for f in frames if (root/f["path"]).is_file())]}}
    report["implementation_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                       for p in sorted(Path(__file__).parent.glob("*.py"))}
    destination = Path(output) if output else root/"analysis"
    destination.mkdir(parents=True, exist_ok=True)
    write_json(destination/"results.json", report)
    with (destination/"trials.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]) if results else ["id"])
        writer.writeheader()
        for row in results:
            writer.writerow(dict(row, quality="; ".join(row["quality"])))
    (destination/"report.md").write_text(markdown(report))
    return report


def markdown(report):
    synthetic = report["data_kind"] == "synthetic"
    lines = ["# 到着位置の計測レポート", "",
             "**合成データによる解析検証。実機・歩行・押下の性能を示さない。**" if synthetic else
             "実機データの記録状況と計測値。未取得・未校正の値は未測定。", "",
             f"予定試行: {report['expected_attempts']}、記録状況: {report['record_counts']}。",
             f"対象: `{report['target']['id']}` / `{report['target']['button']}`。",
             f"navigation結果: {report['outcomes']}。自動到達・停止確認・相対姿勢計測あり: {report['automatic_settled_measurements']}。", "",
             "## 基準と計測品質", "",
             f"基準立位の相対姿勢: {'取得済み' if report['reference'] else '未測定'}。最適な押下位置という意味ではない。",
             f"時計: `{report['clock']}`。校正ID: `{report['calibration'].get('id')}`。",
             "パネル原点はボタン中心の視線とパネル面の交点。ボタンの突出量・押込量は測っていない。",
             "パネル+xはロボット側への法線、+zは重力に基づく上向き、+yはz×x。yawは壁を向いた姿勢からの差。",
             "平面残差・校正の不確かさはresults.jsonに保持する。カメラ推定を真値と扱わず、navとの差は不一致量として扱う。",
             "T_base_pelvisを固定値で与えた場合は立位近似。その誤差と身体の揺れがnavとの差に含まれる。", "",
             "## 偏り・ばらつき", "",
             "自動到達し、停止後の計測がある試行のみ。欠測・手動介入は下の全試行表に残す。位置は母標準偏差、向きは円平均・円標準偏差。",
             "角度はrad、距離はm。heading差は[-π,π)で集計する。95百分位は初期標本の要約であり運用保証ではない。", "",
             "| 項目 | n | 平均 | 標準偏差 | 絶対値p95 | 絶対値最大 |", "|---|---:|---:|---:|---:|---:|"]
    fmt = lambda v: "未測定" if v is None else f"{v:.5f}"
    for key, s in report["summary"].items():
        lines.append(f"| {key} | {s['n']} | {fmt(s['mean'])} | {fmt(s['std'])} | {fmt(s['abs_p95'])} | {fmt(s['abs_max'])} |")
    lines += ["", "## 全試行", "", "| 試行 | 記録 | 到達結果 | 手動介入 | 前後差m | 左右差m | 向き差rad | 停止確認s | 品質・欠測理由 |",
              "|---|---|---|---|---:|---:|---:|---:|---|"]
    for r in report["trials"]:
        quality = "; ".join(r["quality"]).replace("|", "/").replace("\n", " ") or "—"
        lines.append(f"| {r['id']} | {r['record_status']} | {r['nav_outcome']} | {r['manual_intervention']} | {fmt(r['delta_normal_m'])} | {fmt(r['delta_lateral_m'])} | {fmt(r['delta_heading_rad'])} | {fmt(r['stop_delay_s'])} | {quality} |")
    lines += ["", "## 改善候補の整理", "",
              "- 押下側: 停止後のパネル相対位置・向きの分布を、認識範囲や補正範囲の要求候補として渡す。腰・腕の方式や許容範囲は未決定。",
              "- navigation側: 未到達、停止の遅れ、接近経路による偏り、navと局所観測の不一致を調べる。計測誤差を確認してから原因を分ける。",
              "- この記録だけでは押下可能性を判定しない。実機30試行は初期評価であり、成功率の運用保証ではない。", ""]
    return "\n".join(lines)
