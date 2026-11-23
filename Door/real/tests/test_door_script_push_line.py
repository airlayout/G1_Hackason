"""11-16 押し返しの力の線を体の中心に通す（計画 docs/plan/2026-11-16-g1-door-push-line-two-hands-sim.md）: walk_face_door（3c・4a の
向きの目標を扉の角度だけ蝶番の側へ）・walk_center（体に固めた手の横を体の中心へ寄せる）・helper_push_press（添えた手の押し込み）。
偽の世界（test_door_script.World）で、定数の確かめと指令・手の行き先の向きだけを確かめる（⚠️ 機体では未試験）。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_frame as dfr  # noqa: E402
import door_script as dscr  # noqa: E402
from test_door_script import World, make  # noqa: E402


def _record(s):
    """step の出力を (小段, 歩きの指令, 手の行き先) で残す"""
    rows = []
    orig = s.step

    def step(*a, **k):
        o = orig(*a, **k)
        rows.append((o.sub, o.walk, None if o.goal is None else np.asarray(o.goal, dtype=float).copy()))
        return o

    s.step = step
    return rows


def test_defaults_keep_the_old_behavior():
    p = dscr.ScriptParams()
    assert p.walk_face_door == 0.0 and not p.walk_center and p.helper_push_press < 0.0
    _, s = make()
    assert s.face_rad() == 0.0


@pytest.mark.parametrize("kw", [
    {"walk_face_door": -0.1}, {"walk_face_door": 1.6}, {"walk_face_max_deg": 61.0},
    {"walk_center": True, "walk_center_y": 0.31}, {"walk_center": True, "walk_center_rate": 0.0},
    {"walk_center": True, "walk_hand": "arc"}, {"helper_push_press": 0.21},
])
def test_bad_values_are_refused(kw):
    with pytest.raises(ValueError):
        make(**kw)


def test_face_door_turns_the_yaw_target_toward_the_hinge_as_the_door_opens():
    """把手が左（lr +1）なら蝶番は右＝向きの目標は負へ。扉の角度の中央値 × 倍率、上限つき"""
    p, s = make(walk_face_door=1.0, walk_face_max_deg=20.0, walk_yaw_ff=0.0)
    s.sub, s.yaw_3c0 = "3c", 0.0
    s.angles = [(0.0, math.radians(10.0))] * 5
    s.t_progress = 0.0
    assert s.angle_median_deg() == pytest.approx(10.0, abs=1e-6)
    assert s.face_rad() == pytest.approx(math.radians(10.0))
    wz = s._hold_yaw_wz(dfr.PlanarPose(0.0, 0.0, 0.0))
    assert wz < 0.0  # 右（蝶番の側）へ回す指令
    s.angles = [(0.0, math.radians(40.0))] * 5
    assert s.face_rad() == pytest.approx(math.radians(20.0))  # 上限
    s.sub = "3a"
    assert s.face_rad() == 0.0  # 3c・4a の外では回さない


def test_face_door_is_off_when_an_explicit_reference_is_given():
    """walk_from の足踏み（3a の向きを保つ）は ref を渡すので、扉の角度で回さない"""
    p, s = make(walk_face_door=1.0, walk_yaw_ff=0.0)
    s.sub, s.yaw_3c0 = "3c", 0.0
    s.angles = [(0.0, math.radians(15.0))] * 5
    assert s._hold_yaw_wz(dfr.PlanarPose(0.0, 0.0, 0.0), ref=0.0, ff=False) == pytest.approx(0.0)


def test_center_moves_the_body_fixed_hand_toward_the_body_center_at_the_rate():
    p, s = make(walk_center=True, walk_center_y=0.0, walk_center_rate=0.05)
    pose = dfr.PlanarPose(-0.40, -0.30, 0.0)
    s.palm_body = np.array([0.35, 0.25])
    s.palm_ref = np.array([0.0, 0.0, 1.0])
    g0 = s._walk_hand_goal(pose, np.array([-0.05, -0.05, 1.0]), 0.0)
    assert s.palm_body[1] == pytest.approx(0.25 - 0.05 * dscr.DT)
    for _ in range(400):
        g = s._walk_hand_goal(pose, np.array([-0.05, -0.05, 1.0]), 0.0)
    assert s.palm_body[1] == pytest.approx(0.0, abs=1e-9)  # 行き過ぎない
    assert g[1] < g0[1]  # 扉の座標でも手の行き先は右（体の中心の前）へ
    assert s.palm_body[0] == pytest.approx(0.35)  # 前後は変えない


def test_center_off_keeps_the_hand_fixed_to_the_body():
    p, s = make()
    s.palm_body = np.array([0.35, 0.25])
    s.palm_ref = np.array([0.0, 0.0, 1.0])
    for _ in range(10):
        s._walk_hand_goal(dfr.PlanarPose(-0.40, -0.30, 0.0), np.array([-0.05, -0.05, 1.0]), 0.0)
    assert s.palm_body[1] == pytest.approx(0.25)


def test_face_door_and_center_still_finish_in_the_fake_world():
    p, s = make(walk_face_door=1.0, walk_center=True, pass_through=True, pass_x=0.2)
    w = World(p)
    rows = _record(s)
    out = w.run(s, seconds=90.0)
    assert out.success, out
    in3c = [wk for sub, wk, _ in rows if sub == "3c" and wk[0]]
    assert in3c and all(abs(wk[3]) <= p.walk_wz_max + 1e-9 for wk in in3c)


# ---- 11-16 n1 の後: 外の扉の角度・左手を引く・右手を力の線の上に ----
@pytest.mark.parametrize("kw", [
    {"angle_src_3c": "lidar"}, {"helper_line": True}, {"helper": True, "helper_line": True, "helper_line_max_deg": 0.0},
    {"walk_hand_back": 0.05}, {"helper": True, "walk_hand_back": 0.16},
])
def test_bad_new_values_are_refused(kw):
    with pytest.raises(ValueError):
        make(**kw)


def test_ext_angle_is_used_only_in_3c_and_4a():
    p, s = make(angle_src_3c="ext")
    s.palm_ref = np.array([0.0, 0.0, 1.0])
    s.sub, s.goal = "3c", None
    palm = np.array([0.0, 0.0, 1.0])
    s.step(0.0, dfr.PlanarPose(-0.4, -0.3, 0.0), palm, 0.0, 0.3, door_angle_ext=math.radians(20.0))
    assert s.angle == pytest.approx(math.radians(20.0))
    s.step(0.02, dfr.PlanarPose(-0.4, -0.3, 0.0), palm, 0.0, 0.3, door_angle_ext=float("nan"))
    assert s.angle == pytest.approx(0.0, abs=1e-6)  # nan は手のひらの読み（動いていないので 0）
    p2, s2 = make()  # 既定の palm は外の角度を見ない
    s2.palm_ref, s2.sub, s2.goal = np.array([0.0, 0.0, 1.0]), "3c", None
    s2.step(0.0, dfr.PlanarPose(-0.4, -0.3, 0.0), palm, 0.0, 0.3, door_angle_ext=math.radians(20.0))
    assert s2.angle == pytest.approx(0.0, abs=1e-6)


def test_walk_hand_back_pulls_the_lever_hand_back_and_skips_the_push_in_guard():
    p, s = make(helper=True, walk_hand_back=0.08, push_press=0.10)
    s.palm_body, s.palm_ref = np.array([0.30, 0.30]), np.array([0.0, 0.0, 1.0])
    g = s._walk_hand_goal(dfr.PlanarPose(-0.40, -0.30, 0.0), np.array([-0.05, 0.0, 1.0]), 0.0)
    # 行き先 x −0.18 は手のひら（−0.05）から lead_max 5 cm までに切られる＝手のひらより後ろ
    assert g[0] == pytest.approx(-0.05 - p.lead_max)
    s.sub, s.t_sub, s.t_progress = "3c", 0.0, 0.0
    s.yaw_3c0, s.y_3c0 = 0.0, -0.30
    walk, why = s._push_walk(0.1, dfr.PlanarPose(-0.40, -0.30, 0.0), np.array([-0.30, 0.0, 1.0]), 0.05)
    assert why == "" and walk[0]  # 手の前への出 0.05 m でも止めない
