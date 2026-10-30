"""teleop_episode_check.py の試験（2026-10-27）。Mac の Navigation/.venv で:

    G1_Hackason/Navigation/.venv/bin/python -m pytest G1_Hackason/Teleop/scripts/pc2/tests -q
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import teleop_episode_check as C  # noqa: E402

DOOR_REAL = HERE.parents[3] / "Door" / "real"
sys.path.insert(0, str(DOOR_REAL))
sys.path.insert(0, str(DOOR_REAL / "tests"))
import synthetic_door as S  # noqa: E402

START = (-1.2, 0.0, 0.0)   # 扉の正面 1.2 m・正対（計画書の T3 の最初の 10 本の開始位置）


def arm_state(k, tau_scale, effort=True):
    q = [0.1 * math.sin(0.05 * k + j) for j in range(7)]
    if not effort:
        return {"qpos": q, "qvel": [], "torque": []}
    return {"qpos": q, "qvel": [0.005 * math.cos(0.05 * k + j) * 30 for j in range(7)],
            "torque": [tau_scale * (j + 1) / 7 for j in range(7)]}


def make_frames(n=60, t0=1_793_000_000_000_000_000, dt_ms=33.3, walk=True, with_time=True, body=True,
                effort=True, tau_left=3.0, tau_right=3.0):
    frames = []
    for k in range(n):
        f = {"idx": k, "actions": {"left_arm": {"qpos": [0.0] * 7}, "right_arm": {"qpos": [0.0] * 7}},
             "states": {"left_arm": arm_state(k, tau_left, effort), "right_arm": arm_state(k, tau_right, effort)}}
        if with_time:
            f["time_ns"] = t0 + int(k * dt_ms * 1e6)
        if body:
            f["actions"]["body"] = {"qpos": [0.15 if (walk and k > 20) else 0.0, 0.0, 0.0]}
        else:
            f["actions"]["body"] = {"qpos": []}
        frames.append(f)
    return frames


def write_episode(tmp_path, frames, closed=True):
    ep = tmp_path / "episode_0001"
    ep.mkdir()
    head = '{\n"info": {},\n"text": {},\n"data": [\n'
    items = ",\n".join(json.dumps(f) for f in frames)
    (ep / "data.json").write_text(head + items + ("\n]\n}" if closed else ""), encoding="utf-8")
    return ep


# ---------------------------------------------------------------- 遠隔操作の記録
def test_load_frames_reads_unclosed_json(tmp_path):
    # Arrange: 遠隔操作を強制終了すると data.json は閉じないまま残る
    ep = write_episode(tmp_path, make_frames(40), closed=False)

    # Act
    frames = C.load_frames(ep / "data.json")

    # Assert
    assert len(frames) == 40


def test_check_frames_passes_good_record():
    rep = C.Report()
    span = C.check_frames(make_frames(90), rep)
    assert span is not None and rep.bad == 0
    assert rep.row["frames"] == 90 and rep.row["vx_max"] == pytest.approx(0.15)
    assert rep.row["dt_median_ms"] == pytest.approx(33.3, abs=0.1)


def test_check_frames_flags_missing_time():
    rep = C.Report()
    assert C.check_frames(make_frames(90, with_time=False), rep) is None
    assert rep.bad == 1 and "record-timestamp" in "\n".join(rep.lines)


def test_check_frames_flags_missing_walk_command():
    # Arrange: 手なしで record-loco-cmd が当たっていないと body が空になる（10-26 の不具合）
    rep = C.Report()

    # Act
    C.check_frames(make_frames(90, body=False), rep)

    # Assert
    assert rep.bad == 1 and "record-loco-cmd" in "\n".join(rep.lines)


def test_check_frames_warns_but_passes_when_not_walking():
    rep = C.Report()
    C.check_frames(make_frames(90, walk=False), rep)
    assert rep.bad == 0 and any("全部 0" in ln for ln in rep.lines)


# ---------------------------------------------------------------- 左右の腕
def test_arm_effort_summary_per_side():
    rep = C.Report()
    C.check_frames(make_frames(90, tau_left=4.0, tau_right=3.5), rep)
    assert rep.bad == 0
    assert rep.row["L_tau_max"] == pytest.approx(4.0) and rep.row["R_tau_max"] == pytest.approx(3.5)
    assert rep.row["L_tau_joint"] == "手首 yaw" and rep.row["L_travel_rad"] > 0
    assert not any("倍違う" in ln for ln in rep.lines)


def test_arm_effort_missing_is_flagged():
    # Arrange: 修正 record-arm-effort が当たっていないと、上流のまま qvel・torque が空になる
    rep = C.Report()

    # Act
    C.check_frames(make_frames(90, effort=False), rep)

    # Assert
    assert rep.bad == 1 and "record-arm-effort" in "\n".join(rep.lines)


def test_arm_effort_warns_left_right_gap_and_near_limit():
    # Arrange: 左だけ強く押した（手首 yaw の上限 5 N·m の 8 割＝4 N·m を超える）
    rep = C.Report()

    # Act
    C.check_frames(make_frames(90, tau_left=4.5, tau_right=1.0), rep)

    # Assert
    text = "\n".join(rep.lines)
    assert rep.bad == 0
    assert "倍違う" in text and "左が大きい" in text
    assert "左腕が上限の 80% を超えた" in text and rep.row["L_near_limit_frac"] > 0


# ---------------------------------------------------------------- 段階
@pytest.mark.parametrize("angle,x,stage", [
    (float("nan"), float("nan"), "S0"), (1.0, -1.2, "S0"), (5.0, -1.2, "S1"), (40.0, -0.5, "S2"),
    (78.0, 0.2, "S3"), (78.0, 1.6, "S4"), (88.0, 1.6, "S2"),   # 85° を超えて押し切ったら S3・S4 にしない
])
def test_stage_of(angle, x, stage):
    assert C.stage_of(angle, x) == stage


# ---------------------------------------------------------------- 扉の角度（作り物の点群）
def test_track_door_follows_opening_door():
    # Arrange: 開始位置で止まった 5 回で扉と地図を作り、扉が 0 → 80° と開いていく間を追う
    import lidar_door_est as est
    rng = np.random.default_rng(3)
    world0 = S.scene(0.0)
    ref = [S.lidar_scan(world0, *START, pitch_deg=5.0, noise=0.003, rng=rng) for _ in range(5)]
    scans = [(0.1 * (k + 1), S.lidar_scan(S.scene(float(a), lever=False), *START, pitch_deg=5.0, noise=0.003, rng=rng))
             for k, a in enumerate(range(0, 81, 5))]

    # Act
    r = C.track_door(est, ref, scans)

    # Assert
    assert r["max_angle"] == pytest.approx(80.0, abs=2.0)
    assert r["start_x"] == pytest.approx(START[0], abs=0.02)
    assert r["stage"] == "S3"            # 止まったまま 80° まで開けた（通り抜けていない）
    assert r["accepted_frac"] > 0.9


# ---------------------------------------------------------------- 実物の記録で通す（手元に記録があるときだけ）
RUNS = HERE.parents[3] / "Mapping" / "real" / "runs"
REAL_LI = RUNS / "20261021T070217_odom_1021_02_li"   # 10-21 の扉の前で立っている記録（ライダー 75 秒・/dog_odom あり）


@pytest.mark.skipif(not REAL_LI.exists(), reason="10-21 の記録が手元に無い")
def test_main_on_real_recording(tmp_path, capsys):
    # Arrange: 10-21 の実物の記録（扉の前で立っている・扉は閉じている）に、偽の 1 本ぶんの時間を重ねる
    t_start, _ = C.db3_span(REAL_LI)
    t0 = t_start + 20_000_000_000
    ep = write_episode(tmp_path, make_frames(150, t0=t0))

    # Act
    code = C.main(["--episode-dir", str(ep), "--runs", str(RUNS), "--sensors", "odom_1021_02",
                   "--door-real", str(DOOR_REAL)])
    out = capsys.readouterr().out

    # Assert: ライダー・odometry が入っていて、扉と地図が作れ、扉は閉じたまま（S0）
    row = list((ep.parent / "episode_checks.csv").read_text(encoding="utf-8").splitlines())
    assert "ライダー" in out and "/dog_odom" in out
    assert "扉と地図を作れた" in out, out
    assert len(row) == 2
    assert code == 0, out            # 赤外線の画像の記録は手元に無い（⚠️ だけで ✗ にはしない）
    fields = dict(zip(row[0].split(","), row[1].split(",")))
    assert float(fields["door_width"]) == pytest.approx(0.95, abs=0.02)
    assert abs(float(fields["max_angle_deg"])) < 2.0 and fields["stage"] == "S0"
