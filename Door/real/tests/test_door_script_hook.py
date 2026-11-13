"""11-13 夜 手首を下げてレバーに掛ける形（door_script.ScriptParams の hook_*）: 偽の手のひらで状態の移りと形を確かめる。機体は要らない。

形: 2d で押し下げた後、手の先の手前の棒の縁（手のひらの原点から hook_pivot 先）を支点に、手首を下げる（手の先を hook_deg 上へ）。
同時に手を hook_slide だけ手の向きに滑らせ、棒の縁を手の先から遠ざける。前へ押す力が「前と下」の向きで棒に入る（押すほど食い込む）。
扉の角度の基準（palm_ref）は掛け終えた手のひらで取る（掛ける間の手の動きを扉の角度と読まない）。
⚠️ 偽の世界は状態の移りの確かめだけ。棒と手の当たりの形はシム（s0_script_1113.sh）と机の上の門番（script_gate.py）で見る
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import door_script as dscr  # noqa: E402
from test_door_script import DOOR, World, make  # noqa: E402

HOOK = dict(hook_deg=35.0, hook_pivot=0.09, hook_slide=0.02, hook_rate=0.6)


def run_logged(p, s, w=None, seconds=60.0):
    """（時刻, 小段, 手の向きの上下の成分）を周期ごとに残して走らせる"""
    w = w or World(p)
    log = []
    orig = s.step

    def step(t, *a, **k):
        out = orig(t, *a, **k)
        log.append((t, out.sub, None if out.direction is None else float(out.direction[2])))
        return out

    s.step = step
    out = w.run(s, seconds)
    return out, w, log


def test_hook_off_keeps_hand_level_in_every_substage():
    p, s = make()
    out, w, log = run_logged(p, s)
    assert out.success, out
    assert all(dz == 0.0 for _, _, dz in log)


def test_hook_tilts_tip_up_after_press_then_unlatches_and_opens():
    p, s = make(**HOOK)
    out, w, log = run_logged(p, s)
    assert out.success, out
    assert w.subs == ["2a", "2b", "2c", "2d", "3a", "3c", "done"], w.subs
    up = math.sin(math.radians(HOOK["hook_deg"]))
    assert all(dz == 0.0 for _, sub, dz in log if sub in ("2a", "2b", "2c"))
    in2d = [dz for _, sub, dz in log if sub == "2d"]
    assert in2d[0] == 0.0 and in2d[-1] == pytest.approx(up, abs=1e-6)   # 2d の押し下げの後に上げていく
    assert all(b >= a - 1e-12 for a, b in zip(in2d, in2d[1:]))          # 上げるだけ（戻らない）
    assert all(dz == pytest.approx(up, abs=1e-6) for _, sub, dz in log if sub in ("3a", "3c"))


def test_hook_takes_door_angle_reference_after_tilting():
    p, s = make(**HOOK)
    out, w, log = run_logged(p, s)
    t_start = next(t for t, sub, dz in log if sub == "2d" and dz and dz > 0)
    t_3a = next(t for t, sub, _ in log if sub == "3a")
    assert t_3a - t_start >= math.radians(HOOK["hook_deg"]) / HOOK["hook_rate"] - 0.05   # 掛け終えてから 3a
    assert out.success


def test_hook_geometry_rotates_about_bar_edge_and_slides_hand_forward():
    p, s = make(**HOOK)
    palm0 = np.array([-0.18, 0.20, 0.92])
    g0, d0 = s.hook_geometry(palm0, 0.0)
    assert np.allclose(g0, palm0) and np.allclose(d0, [1.0, 0.0, 0.0])
    g1, d1 = s.hook_geometry(palm0, 1.0)
    h = math.radians(HOOK["hook_deg"])
    assert np.allclose(d1, [math.cos(h), 0.0, math.sin(h)])
    pivot = palm0 + HOOK["hook_pivot"] * d0
    reach = HOOK["hook_pivot"] - HOOK["hook_slide"]
    assert np.allclose(pivot - g1, reach * d1)              # 支点は手の向きの上、手のひらの原点から reach 先
    assert g1[0] > palm0[0] and g1[2] < palm0[2]            # 手のひらは前へ・下へ（手首を下げる）
    assert g1[2] == pytest.approx(palm0[2] - reach * math.sin(h))


def test_hook_geometry_follows_hand_yaw_toward_hinge():
    p, s = make(hand_dir_deg=30.0, **HOOK)
    _, d1 = s.hook_geometry(np.array([-0.18, 0.20, 0.92]), 1.0)
    h, a = math.radians(HOOK["hook_deg"]), math.radians(30.0)
    assert np.allclose(d1, [math.cos(h) * math.cos(a), -math.cos(h) * math.sin(a), math.sin(h)])  # 把手が左: 蝶番は −y


def test_hook_is_undone_when_retreating_and_redone_on_retry():
    p, s = make(**HOOK)
    w = World(p)
    w.yaw_kicks = {"3a": math.radians(p.yaw_abort_deg + 2.0)}  # 3a で体が回る → 手を上げてやり直す
    out, w, log = run_logged(p, s, w)
    assert "retreat" in w.subs and out.success, (w.subs, out)
    i = w.subs.index("retreat")
    assert w.subs[i:i + 2] == ["retreat", "2a"]
    t_2a = next(t for t, sub, _ in log if sub == "2a" and t > 1.0 and any(x[1] == "retreat" for x in log if x[0] < t))
    assert all(dz == 0.0 for t, sub, dz in log if t >= t_2a and sub in ("2a", "2b", "2c"))  # やり直しの構えは手を水平に
    assert sum(1 for k in range(1, len(log)) if log[k][1] == "3a" and log[k - 1][1] == "2d") == 2  # 2 回とも掛けてから 3a


def test_stop_after_2d_stops_after_the_hook_not_before():
    p, s = make(stop_after="2d", **HOOK)
    out, w, log = run_logged(p, s)
    assert "2d が終わった" in out.stop
    assert log[-1][2] == pytest.approx(math.sin(math.radians(HOOK["hook_deg"])), abs=1e-6)


@pytest.mark.parametrize("bad", [
    dict(hook_deg=-5.0), dict(hook_deg=70.0), dict(hook_deg=35.0, hook_pivot=0.0), dict(hook_deg=35.0, hook_pivot=0.15),
    dict(hook_deg=35.0, hook_slide=0.08), dict(hook_deg=35.0, hook_slide=-0.01), dict(hook_deg=35.0, hook_rate=0.0),
    dict(hook_deg=35.0, hook_rate=3.0), dict(hook_deg=35.0, hand_dir_deg=-1.0), dict(hook_deg=35.0, hook_settle_s=-0.1),
])
def test_bad_hook_values_are_rejected(bad):
    with pytest.raises(ValueError):
        dscr.load_params(None, **bad)


def test_shipped_params_keep_hook_off():
    here = Path(__file__).resolve().parents[1]
    for name in ("script_params.json", "script_params_run1.json"):
        assert dscr.load_params(str(here / name)).hook_deg == 0.0
    assert dscr.ScriptDoor(dscr.load_params(None), *DOOR, True).p.hook_deg == 0.0
