"""Physics-independent evidence tests; fixtures are never field measurements."""
from __future__ import annotations

import ast
import math
from pathlib import Path

import numpy as np
import pytest

from Navigation.arrival.analysis import (analyze, measure_frame, navigation_result, post_arrival,
                                         statistics, stop_time)
from Navigation.arrival.geometry import (Unmeasured, camera_transform, interpolate_state,
                                         panel_pose, quaternion, transform)
from Navigation.arrival.session import Writer, initialize, mark, read_json, read_lines, trial_windows
from Navigation.arrival.synthetic import generate


@pytest.fixture(scope="module")
def dataset(tmp_path_factory):
    root = tmp_path_factory.mktemp("arrival")/"synthetic"
    generate(root)
    report = analyze(root)
    return root, report


def test_known_arrival_offsets_and_separate_navigation_drift(dataset):
    root, report = dataset
    expected = {r["id"]: r for r in read_json(root/"expected.json")["trials"]}
    assert report["data_kind"] == "synthetic"
    assert len(report["trials"]) == 30
    assert report["automatic_settled_measurements"] == 24
    assert report["reference"] is not None
    for row in report["trials"][:24]:
        for key in ("delta_normal_m", "delta_lateral_m", "delta_heading_rad"):
            assert row[key] == pytest.approx(expected[row["id"]][key], abs=.001)
        assert row["nav_local_discrepancy_m"] == pytest.approx(math.hypot(.01, .005), abs=.001)
        assert row["stop_delay_s"] == pytest.approx(.9, abs=.03)
        assert row["plane_rms_m"] < .001
        assert row["post_motion_max_m"] == pytest.approx(0)


def test_failures_and_missing_attempt_stay_in_inventory(dataset):
    _, report = dataset
    rows = report["trials"]
    assert rows[24]["measured_frames"] == 0  # missing body
    assert rows[25]["nav_outcome"] == "aborted"
    assert rows[26]["manual_intervention"] is True
    assert rows[27]["measured_frames"] == 0  # missing depth
    assert rows[28]["measured_frames"] == 0  # wrong clock
    assert rows[29]["record_status"] == "not_recorded"
    assert rows[29]["delta_normal_m"] is None
    assert sum(report["record_counts"].values()) == 30
    assert all(report["summary"][key]["n"] == 24 for key in report["summary"])


def test_real_session_has_thirty_unmeasured_rows(tmp_path):
    initialize(tmp_path)
    report = analyze(tmp_path)
    assert report["data_kind"] == "real"
    assert report["record_counts"] == {"not_recorded": 30}
    assert report["reference"] is None
    assert report["summary"]["delta_normal_m"]["mean"] is None
    assert len((tmp_path/"analysis/trials.csv").read_text().splitlines()) == 31


def test_session_rejects_overlapping_attempts_and_preserves_early_end(tmp_path):
    manifest = initialize(tmp_path)
    mark(tmp_path, "route_1_01", "start", stamp=1.)
    with pytest.raises(ValueError, match="active"):
        mark(tmp_path, "route_2_01", "start", stamp=2.)
    mark(tmp_path, "route_1_01", "manual_intervention", stamp=2.)
    mark(tmp_path, "route_1_01", "end", reason="timeout", stamp=3.)
    windows = trial_windows(tmp_path, manifest)
    assert windows["route_1_01"]["manual"]
    assert windows["route_1_01"]["end_reason"] == "timeout"
    with pytest.raises(ValueError):
        mark(tmp_path, "route_1_01", "start", stamp=4.)


def test_unfinished_attempt_is_not_reported_as_complete(tmp_path):
    initialize(tmp_path)
    mark(tmp_path, "route_1_01", "start", stamp=1.)
    report = analyze(tmp_path)
    assert report["trials"][0]["record_status"] == "incomplete"
    assert report["trials"][0]["nav_outcome"] == "unmeasured"


def state(t, angle=0.):
    q = np.zeros(29)
    q[12] = angle
    return {"t": t, "q": q, "dq": np.zeros(29), "imu_wxyz": [math.cos(angle/2), 0., 0., math.sin(angle/2)]}


def test_exposure_interpolation_and_quaternion_wrap():
    samples = [state(0., math.radians(170)), state(1., math.radians(-170))]
    result = interpolate_state(samples, .5, 1.1)
    assert result["q"][12] == pytest.approx(0.)  # bounded encoders interpolate linearly
    assert quaternion(result["imu_wxyz"])[0, 0] == pytest.approx(-1.)  # IMU follows short arc


@pytest.mark.parametrize("samples,t,gap", [([state(0), state(1)], -.1, 2),
    ([state(0), state(1)], 1.1, 2), ([state(0), state(1)], .5, .1),
    ([state(0), state(0)], 0., .1)])
def test_no_extrapolation_duplicate_time_or_gap(samples, t, gap):
    with pytest.raises(Unmeasured):
        interpolate_state(samples, t, gap)


def test_waist_mount_uses_measured_angle_before_camera_offset():
    tail = np.eye(4)
    tail[0, 3] = .2
    origin = np.eye(4)
    origin[2, 3] = .4
    profile = {"measured": True, "joints": [{"T_parent_joint": origin,
                "axis": [0, 0, 1], "q_index": 12}], "T_last_optical": tail}
    q = np.zeros(29)
    q[12] = math.pi/2
    T = camera_transform(profile, q)
    assert T[:3, 3] == pytest.approx([0., .2, .4])
    assert T[:3, 0] == pytest.approx([0., 1., 0.])


@pytest.mark.parametrize("bad", [np.zeros((4, 4)), np.diag([-1, 1, 1, 1]), np.eye(3)])
def test_reject_nonrigid_or_reflected_calibration(bad):
    with pytest.raises(Unmeasured):
        transform(bad)


def test_simple_panel_signed_distance_height_and_heading():
    T = np.eye(4)
    T[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    T[2, 3] = .4
    K = np.array([[60., 0., 40.], [0., 60., 30.], [0., 0., 1.]])
    result = panel_pose(np.full((60, 80), .7), K,
        {"panel_polygon": [[2, 2], [77, 2], [77, 57], [2, 57]], "button_uv": [40, 30]}, T, [0., 0., 1.])
    assert result["stand_off_m"] == pytest.approx(.7)
    assert result["lateral_m"] == pytest.approx(0.)
    assert result["height_m"] == pytest.approx(-.4)
    assert result["heading_rad"] == pytest.approx(0.)


@pytest.mark.parametrize("change,reason", [("calibration", "extrinsics"), ("clock", "synchronization"),
    ("exposure", "exposure clock"), ("depth", "aligned"), ("future", "future"),
    ("imu", "IMU mounting"), ("body_uncertainty", "uncertainty")])
def test_missing_measurement_prerequisites_never_become_zero(dataset, change, reason):
    root, _ = dataset
    manifest = read_json(root/"session.json")
    calibration = read_json(root/"calibration.json")
    frame = dict(read_lines(root/"streams/frames.jsonl")[0])
    annotation = read_json(root/"annotations.json")["frames"][0]
    body = read_lines(root/"streams/body.jsonl")
    if change == "calibration":
        calibration["measured"] = False
    elif change == "clock":
        manifest["clock"]["synchronized"] = False
    elif change == "exposure":
        frame["stamp_kind"] = "receive"
    elif change == "depth":
        frame["depth_aligned"] = False
    elif change == "future":
        frame["received_at"] = frame["t"]-10
    elif change == "imu":
        calibration["T_pelvis_imu"] = None
    else:
        for sample in body:
            sample["time_uncertainty_s"] = None
    with pytest.raises(Unmeasured, match=reason):
        measure_frame(root, manifest, calibration, frame, annotation, body)


def test_stale_and_ambiguous_goal_status_are_not_arrivals():
    window = {"start": 10., "end": 20., "goal_id": None}
    stale = {"kind": "nav_status", "t": 11., "goal_time": 5., "goal_id": "old", "status": "succeeded"}
    assert navigation_result([stale], window)[0] == "unmeasured"
    current = dict(stale, goal_time=10.5, goal_id="new", t=15.)
    assert navigation_result([stale, current], window)[:2] == ("succeeded", 15.)
    assert navigation_result([current, dict(current, goal_id="other")], window)[0] == "unmeasured"
    assert navigation_result([stale], dict(window, goal_id="old"))[0] == "unmeasured"


def test_missing_samples_do_not_prove_settling_or_zero_motion():
    settings = {"body_max_gap_s": .1, "pose_max_gap_s": .3, "post_arrival_s": 3.,
                "stop_dwell_s": .5, "leg_speed_rad_s": .05, "odom_speed_m_s": .02, "odom_yaw_speed_rad_s": .05}
    odom = [{"kind": "odom", "t": float(t), "frame_id": "odom", "child_frame_id": "base_link",
             "xyz": [0., 0., 0.], "xyzw": [0., 0., 0., 1.]} for t in np.arange(0., 4.1, .1)]
    body = [state(t) for t in [.5, .55, .6, 1.8, 1.85, 1.9]]
    assert stop_time(odom, body, .5, settings) is None
    missing = post_arrival([], [], .5, 4., settings)
    assert all(v is None for v in missing.values())


def test_empty_statistics_are_missing_not_zero():
    assert statistics([None, None])["mean"] is None
    assert statistics([0., None])["n"] == 1
    assert statistics([0., None])["mean"] == 0.


def test_heading_statistics_do_not_average_opposite_wraps_to_zero():
    stats = statistics([math.radians(179), math.radians(-179)], circular=True)
    assert abs(stats["mean"]) == pytest.approx(math.pi)
    assert stats["std"] == pytest.approx(math.radians(1), abs=1e-5)


def test_stream_cannot_have_two_recorders(tmp_path):
    first = Writer(tmp_path, "body")
    try:
        with pytest.raises(ValueError, match="another recorder"):
            Writer(tmp_path, "body")
    finally:
        first.close()


def test_clock_rollback_between_attempts_is_rejected(tmp_path):
    initialize(tmp_path)
    mark(tmp_path, "route_1_01", "start", stamp=10.)
    mark(tmp_path, "route_1_01", "end", stamp=20.)
    with pytest.raises(ValueError, match="backwards"):
        mark(tmp_path, "route_2_01", "start", stamp=15.)


@pytest.mark.parametrize("missing", ["base_transform", "clock"])
def test_missing_frame_transform_or_clock_keeps_dependent_metrics_null(dataset, tmp_path, monkeypatch, missing):
    from Navigation.arrival import analysis

    root, _ = dataset
    original = analysis.read_json

    def read(path):
        value = original(path)
        if missing == "base_transform" and Path(path).name == "calibration.json":
            value["T_base_pelvis"] = None
        if missing == "clock" and Path(path).name == "session.json":
            value["clock"]["synchronized"] = False
        return value

    monkeypatch.setattr(analysis, "read_json", read)
    report = analysis.analyze(root, tmp_path)
    for row in report["trials"][:24]:
        if missing == "base_transform":
            assert row["stand_off_m"] is not None
            assert row["nav_local_discrepancy_m"] is None
            assert any("floor-base" in issue for issue in row["quality"])
        else:
            assert row["stop_delay_s"] is None
            assert row["stand_off_m"] is None
            assert row["post_motion_max_m"] is None


def test_collectors_cannot_issue_robot_control_commands():
    source = Path(__file__).parents[1]/"collect.py"
    tree = ast.parse(source.read_text())
    forbidden = {"ChannelPublisher", "LocoClient", "ActionClient", "create_publisher", "create_client", "command", "StopMove", "SetVelocity", "Move"}
    calls = {getattr(n.func, "attr", getattr(n.func, "id", "")) for n in ast.walk(tree) if isinstance(n, ast.Call)}
    assert not calls & forbidden
    assert "Manipulation" not in source.read_text()
