"""11-12 夜: 当たりの模型（door_clearance.py）・門番（script_gate.py）・巻き尺の形（door_staged.tape_check）・
台本のつなぎ（door_run_staged.ScriptArm の順運動学のまとめ・手を離す動き）。機体は要らない。

門番の線は 11-12 の機体の結果（どの版の定数が当たり、どれが通ったか）で答え合わせする（test_gate_history_matches_robot_outcomes）。
"""
from __future__ import annotations

import io
import contextlib
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import door_clearance as dcl  # noqa: E402
import door_frame as dfr  # noqa: E402
import door_run_staged as drst  # noqa: E402
import door_script as dscr  # noqa: E402
import door_staged as ds  # noqa: E402
import g1_arm_fk as fk  # noqa: E402
import g1_arm_ik as aik  # noqa: E402
import hand_door_angle as hda  # noqa: E402
import script_gate as sg  # noqa: E402

DOOR = (0.94, 2.05, 0.99, 0.064)
DOOR_S = "0.94,2.05,0.99,0.064"


@pytest.fixture(scope="module")
def chain():
    return fk.load_chain()


# ── 当たりの模型 ──────────────────────────────────────────
def test_obstacles_follow_the_tape_measurements():
    g = dcl.load_geom(None, DOOR)
    ob = dcl.obstacles(g)
    bar = ob["棒"]
    assert abs(bar.a[0] - (-0.02 - 0.055)) < 1e-9 and abs(bar.a[2] + bar.r - 1.008) < 1e-9  # 棒の中心は板の表の 5.5 cm 手前・上面 100.8 cm
    assert abs(bar.a[1] - (0.47 - 0.064)) < 1e-9 and abs(bar.a[1] - bar.b[1] - 0.13) < 1e-9  # 付け根から蝶番の側へ 13 cm
    assert abs(ob["枠"].lo[0] - (-0.05)) < 1e-9 and abs(ob["壁"].lo[0] - (-0.031)) < 1e-9
    ref = ds.lever_ref_point(0.94, 0.064, 0.99, True)
    assert bar.b[1] < ref[1] < bar.a[1]  # 当てる点は棒の上


def test_obstacles_mirror_for_lever_on_the_right():
    gl, gr = dcl.load_geom(None, DOOR, True), dcl.load_geom(None, DOOR, False)
    a, b = dcl.obstacles(gl), dcl.obstacles(gr)
    assert np.allclose(a["棒"].a * [1, -1, 1], b["棒"].a) and np.allclose(a["枠"].lo[1], -b["枠"].hi[1])


def test_seg_gap_box_and_rod():
    box = dcl.Box(np.array([0.0, -1.0, 0.0]), np.array([0.1, 1.0, 1.0]))
    s = dcl.Seg("t", np.array([-0.2, 0.0, 0.5]), np.array([-0.05, 0.0, 0.5]), 0.02)
    assert abs(dcl.seg_gap(s, box) - 0.03) < 1e-6           # 端から 5 cm − 太さ 2 cm
    s_in = dcl.Seg("t", np.array([-0.2, 0.0, 0.5]), np.array([0.05, 0.0, 0.5]), 0.02)
    assert dcl.seg_gap(s_in, box) < 0                          # 食い込みは負
    rod = dcl.Rod(np.array([0.0, -0.1, 1.0]), np.array([0.0, 0.1, 1.0]), 0.01)
    up = dcl.Seg("t", np.array([0.0, 0.0, 1.1]), np.array([0.0, 0.0, 1.2]), 0.02)
    assert abs(dcl.seg_gap(up, rod) - 0.07) < 1e-6


def test_load_geom_rejects_unknown_names(tmp_path):
    f = tmp_path / "g.json"
    f.write_text(json.dumps({"bar_lenn": 0.1}), encoding="utf-8")
    with pytest.raises(ValueError, match="知らない名前"):
        dcl.load_geom(str(f), DOOR)
    f.write_text(json.dumps({"bar_len": 0.15, "hand_tip": 0.13}), encoding="utf-8")
    g = dcl.load_geom(str(f), DOOR)
    assert g.bar_len == 0.15 and g.hand_tip == 0.13 and g.width == 0.94


# ── 門番 ─────────────────────────────────────────────────
def test_gate_history_matches_robot_outcomes(capsys):
    """11-12 の各回の定数で、門番が実機の結果（当たった版は落とし、通った版は通す）と 6/6 合う"""
    assert sg.run_history(None, DOOR_S, 0.73) == 0
    assert "6/6" in capsys.readouterr().out


def test_gate_passes_shipped_params_from_all_stance_corners():
    p = dscr.load_params(str(HERE / "script_params.json"))
    ok, warn, res = sg.check(p, dcl.load_geom(None, DOOR), 0.73)
    assert ok, sg.report(res)
    assert len(res) == 5 and all(r.min_gap["棒"][0] >= sg.WARN_M for r in res)  # 上げる間に棒から 2 cm 以上


def test_gate_fails_robot_day_params_at_back_right_corner():
    """11-12 の機体の値（最初の経由点が低い）は、構えの許容の隅（後ろ・右・左回り）で手が届かない＝11-12 夜に直した理由"""
    p = dscr.load_params(str(HERE / "script_params_1112_robot.json"))
    ok, _, res = sg.check(p, dcl.load_geom(None, DOOR), 0.73)
    assert not ok and [r.name for r in res if not r.ok] == ["後ろ・右・左回り"]


def test_gate_stamp_and_verify(tmp_path):
    src = json.loads((HERE / "script_params.json").read_text(encoding="utf-8"))
    f = tmp_path / "p.json"
    f.write_text(json.dumps(src), encoding="utf-8")
    assert "無い" in sg.verify_ok(str(f), None, DOOR_S, 0.73)
    with contextlib.redirect_stdout(io.StringIO()):
        assert sg.main([str(f), "--door", DOOR_S, "--pelvis-z", "0.73"]) == 0
    assert sg.verify_ok(str(f), None, DOOR_S, 0.73) == ""
    assert "DOOR" in sg.verify_ok(str(f), None, "0.95,2.05,0.99,0.064", 0.73)       # 違う扉
    assert "DOOR" in sg.verify_ok(str(f), None, DOOR_S, 0.75)                       # 違う pelvis の高さ
    src["press_depth"] = 0.06
    f.write_text(json.dumps(src), encoding="utf-8")
    assert "変わった" in sg.verify_ok(str(f), None, DOOR_S, 0.73)                  # 通った後に直した


def test_gate_failure_removes_stale_ok(tmp_path):
    src = json.loads((HERE / "script_params.json").read_text(encoding="utf-8"))
    f = tmp_path / "p.json"
    f.write_text(json.dumps(src), encoding="utf-8")
    with contextlib.redirect_stdout(io.StringIO()):
        assert sg.main([str(f), "--door", DOOR_S]) == 0
        src["above"] = [0.0, 0.0, 0.10]  # 2 回目の形: 手の先が板に届く
        f.write_text(json.dumps(src), encoding="utf-8")
        assert sg.main([str(f), "--door", DOOR_S]) == 1
    assert not sg.ok_path(str(f)).exists()


# ── 巻き尺の形 ───────────────────────────────────────────
def test_tape_check_catches_the_1112_typo():
    assert ds.tape_check(0.73, 0.72, 0.17, 0.38) == ""
    assert "両足の間 11 cm" in ds.tape_check(0.73, 0.72, 0.17, 0.28)   # 11-12: 38 を 28 と打ち間違えた
    assert "差" in ds.tape_check(0.73, 0.55, 0.17, 0.38)
    assert ds.tape_check(0.73, float("nan"), 0.17, 0.38) != ""


# ── 台本のつなぎ ─────────────────────────────────────────
def _arm(chain, **kw):
    p = replace(dscr.ScriptParams(), **kw)
    ik = aik.ArmIK(chain, "left")
    sd = dscr.ScriptDoor(p, 0.94, 0.064, 0.99, True)
    return drst.ScriptArm(sd, ik, drst.make_palm_fn(chain, 0.73, True), 0.73), ik


def test_script_arm_palm_matches_old_palm_fn(chain):
    arm, ik = _arm(chain)
    q = np.zeros(29)
    q[ik.idx] = dscr.ScriptParams().posture
    quat = np.array([math.cos(0.03), 0.0, math.sin(0.03), 0.0])  # 少し前へ傾いている
    pose = dfr.PlanarPose(-0.40, 0.08, 0.1)
    with contextlib.redirect_stdout(io.StringIO()):
        arm.step(0.0, q, quat, pose, np.zeros(29))
    old = hda.palm_in_door_frame(chain, q, pose, quat, 0.73, "left")
    assert np.allclose(arm.script.palm, old, atol=1e-9)  # 1 周期 1 回の順運動学に替えても同じ手のひら


def test_ik_direction_from_shared_frames_matches_separate_call(chain):
    ik = aik.ArmIK(chain, "left")
    q = np.zeros(29)
    q7 = np.array(dscr.ScriptParams().posture)
    u1, J1 = ik.direction(q, q7)
    p, _ = ik.jacobian(q, q7)
    u2, J2 = ik._direction_from(p, ik._frames)
    assert np.allclose(u1, u2) and np.allclose(J1, J2)


def test_release_moves_hand_back_and_up_then_holds(chain):
    arm, ik = _arm(chain, release_s=1.0)
    q = np.zeros(29)
    q[ik.idx] = dscr.ScriptParams().posture
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    pose = dfr.PlanarPose(-0.40, 0.08, 0.0)
    with contextlib.redirect_stdout(io.StringIO()):
        arm.step(0.0, q, quat, pose, np.zeros(29))
    assert not arm.begin_release(0.1, q, quat)          # 2a（触れていない）は離さない
    arm.script.sub = "2d"
    assert arm.begin_release(0.1, q, quat)
    p0 = hda.palm_in_door_frame(chain, q, pose, quat, 0.73, "left")
    for k in range(60):                                  # 行き先どおりに動く腕で 1.2 秒
        q7 = arm.release(0.1 + k * 0.02, q, quat)
        if q7 is None:
            break
        q[ik.idx] = q7
    p1 = hda.palm_in_door_frame(chain, q, pose, quat, 0.73, "left")
    d = p1 - p0
    assert d[0] < -0.03 and d[2] > 0.01                   # 後ろへ 4 cm・上へ 2 cm の向きに動いた
    assert arm.release(5.0, q, quat) is None              # release_s を過ぎたら保つ
