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


def test_track_door_keeps_per_scan_track():
    # Arrange: 押す力の計算（door_push_force.py）が 1 本ごとに扉の角度と頭の位置を引けるよう、1 回ずつの値も返す
    import lidar_door_est as est
    rng = np.random.default_rng(4)
    ref = [S.lidar_scan(S.scene(0.0), *START, pitch_deg=5.0, noise=0.003, rng=rng) for _ in range(5)]
    scans = [(0.1 * (k + 1), S.lidar_scan(S.scene(float(a), lever=False), *START, pitch_deg=5.0, noise=0.003, rng=rng))
             for k, a in enumerate(range(0, 41, 10))]

    # Act
    tr = C.track_door(est, ref, scans)["track"]

    # Assert
    assert set(tr) == {"t", "angle", "accepted", "x", "y", "yaw"}
    assert all(len(v) == len(scans) for v in tr.values())
    assert tr["t"][0] == pytest.approx(0.1) and tr["angle"][-1] == pytest.approx(40.0, abs=2.0)
    assert tr["x"][0] == pytest.approx(START[0], abs=0.02)


# ---------------------------------------------------------------- 指令どおりに進んだ割合（10-30 の改善策 1 の関門）
T0_NS = 1_793_000_000_000_000_000


def odom_track(seconds, rate_hz, vel_body, yaw0=0.0):
    """機体の座標の速さ vel_body(t) -> (vx, vy) で動いた /dog_odom（時刻 [s]・世界の xy・yaw）"""
    t = np.arange(0.0, seconds, 1.0 / rate_hz)
    xy = np.zeros((len(t), 2))
    for k in range(1, len(t)):
        vx, vy = vel_body(t[k - 1])
        c, s = math.cos(yaw0), math.sin(yaw0)
        xy[k] = xy[k - 1] + np.array([c * vx - s * vy, s * vx + c * vy]) / rate_hz
    return t, xy, np.full(len(t), yaw0)


def cmd_track(seconds, cmd_of_t, rate_hz=30.0):
    t = np.arange(0.0, seconds, 1.0 / rate_hz)
    return t, np.array([cmd_of_t(x) for x in t], dtype=float)


def test_command_segments_keep_only_pushes_longer_than_threshold():
    # Arrange: 1.0 秒の押し（数える）と 0.3 秒の押し（短すぎて数えない）
    t, v = cmd_track(4.0, lambda x: 0.2 if 0.5 <= x < 1.5 or 2.5 <= x < 2.8 else 0.0)

    # Act
    segs = C.command_segments(t, v)

    # Assert
    assert len(segs) == 1
    a, b = segs[0]
    assert t[a] == pytest.approx(0.5, abs=0.04) and t[b] == pytest.approx(1.5, abs=0.04)


def test_compliance_is_full_when_odometry_follows_the_command():
    # Arrange: 前へ 0.2 m/s を 3 秒（機体は世界で 90° を向いている＝機体の座標に直せているかも見る）
    t, cmd1 = cmd_track(6.0, lambda x: 0.2 if 1.0 <= x < 4.0 else 0.0)
    cmd = np.stack([cmd1, np.zeros_like(cmd1), np.zeros_like(cmd1)], axis=1)
    ot, oxy, oyaw = odom_track(6.0, 500, lambda x: (0.2 if 1.0 <= x < 4.0 else 0.0, 0.0), yaw0=math.pi / 2)

    # Act
    r = C.cmd_compliance(t, cmd, ot, oxy, oyaw)

    # Assert
    fwd = r["前"]
    assert fwd["segments"] == 1
    assert fwd["expected_m"] == pytest.approx(0.6, abs=0.01)
    assert fwd["got_m"] == pytest.approx(0.6, abs=0.02)
    assert r["後ろ"]["segments"] == 0 and r["左"]["segments"] == 0


def test_compliance_separates_blocked_forward_from_free_backward_and_sideways():
    # Arrange: 1 本目（10-29）の型。扉の前で前への指令には 12% しか進まず、後ろ・左には指令どおり進む
    def cmd_of(x):
        if 1.0 <= x < 3.0:
            return (0.25, 0.0, 0.0)
        if 4.0 <= x < 6.0:
            return (-0.2, 0.0, 0.0)
        if 7.0 <= x < 9.0:
            return (0.0, 0.15, 0.0)
        return (0.0, 0.0, 0.0)

    t = np.arange(0.0, 11.0, 1 / 30)
    cmd = np.array([cmd_of(x) for x in t])
    ot, oxy, oyaw = odom_track(11.0, 500, lambda x: (0.12 * cmd_of(x)[0] if cmd_of(x)[0] > 0 else cmd_of(x)[0],
                                                      cmd_of(x)[1]))

    # Act
    r = C.cmd_compliance(t, cmd, ot, oxy, oyaw)

    # Assert
    assert r["前"]["got_m"] / r["前"]["expected_m"] == pytest.approx(0.12, abs=0.02)
    assert r["後ろ"]["got_m"] / r["後ろ"]["expected_m"] == pytest.approx(1.0, abs=0.03)
    assert r["左"]["got_m"] / r["左"]["expected_m"] == pytest.approx(1.0, abs=0.03)
    assert r["右"]["segments"] == 0


def odom_cdr(sec, nanosec, x, y, yaw):
    """nav_msgs/Odometry の CDR（リトルエンディアン）の先頭から姿勢まで。dog_odom_bag.parse_odom_cdr が読む形"""
    import struct

    def string(buf, s):
        b = s.encode() + b"\0"
        return buf + struct.pack("<I", len(b)) + b

    def align(buf, n):
        return buf + b"\0" * ((-(len(buf) - 4)) % n)

    buf = b"\x00\x01\x00\x00" + struct.pack("<iI", sec, nanosec)
    buf = string(buf, "odom")
    buf = align(buf, 4)
    buf = string(buf, "base_link")
    buf = align(buf, 8)
    return buf + struct.pack("<7d", x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def write_odom_db3(session, ot_ns, oxy, oyaw):
    import sqlite3
    raw = session / "raw" / "rosbag2"
    raw.mkdir(parents=True)
    con = sqlite3.connect(raw / "s_0.db3")
    con.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT)")
    con.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    con.execute("INSERT INTO topics VALUES (1, '/dog_odom', 'nav_msgs/msg/Odometry', 'cdr')")
    con.executemany("INSERT INTO messages (topic_id, timestamp, data) VALUES (1, ?, ?)",
                    [(int(tn), odom_cdr(int(tn // 1_000_000_000), int(tn % 1_000_000_000), float(p[0]), float(p[1]), float(y)))
                     for tn, p, y in zip(ot_ns, oxy, oyaw)])
    con.commit()
    con.close()


def test_check_compliance_reads_dog_odom_from_the_recording(tmp_path):
    # Arrange: 記録の時刻（PC2 の time_ns）で前へ 0.2 m/s を 2 秒。/dog_odom は db3 に 500 回/秒（その半分しか進まない）
    frames = make_frames(150, t0=T0_NS, walk=False)
    for f in frames:
        x = (f["time_ns"] - T0_NS) / 1e9
        f["actions"]["body"]["qpos"] = [0.2 if 1.0 <= x < 3.0 else 0.0, 0.0, 0.0]
    # odom の時刻は記録の 1 秒前から始まるので、記録の 1〜3 秒は odom の 2〜4 秒
    ot, oxy, oyaw = odom_track(7.0, 500, lambda x: (0.1 if 2.0 <= x < 4.0 else 0.0, 0.0), yaw0=0.3)
    od = tmp_path / "20261031T010000_gate_odom"
    write_odom_db3(od, T0_NS - 1_000_000_000 + (ot * 1e9).astype(np.int64), oxy, oyaw)
    rep = C.Report()

    # Act: odom の時刻は 1 秒前から始まる（記録の時刻に合わせて切り出せているか）
    C.check_compliance(od, frames, T0_NS, frames[-1]["time_ns"], DOOR_REAL, rep)

    # Assert
    assert rep.bad == 0
    assert rep.row["fwd_pct"] == pytest.approx(50, abs=4)
    assert rep.row["fwd_cmd_m"] == pytest.approx(0.4, abs=0.02)
    assert rep.row["back_pct"] == "" and rep.row["left_pct"] == ""
    assert any("指令どおりに進んだ割合" in ln and "前 " in ln for ln in rep.lines)


def test_check_compliance_without_walking_says_so(tmp_path):
    frames = make_frames(90, t0=T0_NS, walk=False)
    ot, oxy, oyaw = odom_track(5.0, 500, lambda x: (0.0, 0.0))
    od = tmp_path / "20261031T010000_gate_odom"
    write_odom_db3(od, T0_NS - 1_000_000_000 + (ot * 1e9).astype(np.int64), oxy, oyaw)
    rep = C.Report()

    C.check_compliance(od, frames, T0_NS, frames[-1]["time_ns"], DOOR_REAL, rep)

    assert rep.bad == 0 and rep.row["fwd_pct"] == ""
    assert any("0.6 秒以上続いた歩きの指令が無い" in ln for ln in rep.lines)


# ---------------------------------------------------------------- 関門の数え（S2 以上が 5/10 で合格）
def test_gate_tally_counts_latest_check_of_each_episode(tmp_path):
    # Arrange: 同じ 1 本を 2 度点検したら新しい方を使う（ep2 は S2 → 直して S1）
    out = tmp_path / "episode_checks.csv"
    for ep, stage in (("episode_0001", "S0"), ("episode_0002", "S2"), ("episode_0002", "S1"),
                      ("episode_0003", "S3"), ("episode_0004", "S4"), ("episode_0005", "")):
        C.append_csv(out, {"episode": ep, "stage": stage})

    # Act
    reached, total = C.gate_tally(out)

    # Assert: 段階が出なかった 1 本（扉を測れなかった）も分母に入れる
    assert (reached, total) == (2, 5)


def test_append_csv_widens_an_old_header_without_shifting_columns(tmp_path):
    # Arrange: 10-29 の記録表は、指令どおりに進んだ割合の欄が無い古い見出しで書かれている
    import csv
    out = tmp_path / "episode_checks.csv"
    out.write_text("episode,seconds,stage,problems\nepisode_0001,578.0,S1,0\n", encoding="utf-8")

    # Act
    C.append_csv(out, {"episode": "episode_0002", "seconds": 30.0, "fwd_pct": 45, "stage": "S2", "problems": 0})

    # Assert: 古い行も新しい行も、欄の名前どおりに読める
    rows = list(csv.DictReader(out.open(encoding="utf-8")))
    assert [r["episode"] for r in rows] == ["episode_0001", "episode_0002"]
    assert rows[0]["stage"] == "S1" and rows[0]["fwd_pct"] == ""
    assert rows[1]["stage"] == "S2" and rows[1]["fwd_pct"] == "45"


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
