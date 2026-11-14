"""11-14 段 4 の入り（4a・計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の 13.3）: 3c で扉が goal_deg に
届いたら、手は 3c の形のまま前へ歩き続け、pelvis が戸口の面（x ≥ pass_x）を越えたら成功で止まる。偽の世界（test_door_script.World）で
状態の移りだけを確かめる（⚠️ 推測で作った形・機体の当たり方は見ていない）。"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_frame as dfr  # noqa: E402
import door_script as dscr  # noqa: E402
from test_door_script import DT, World, make  # noqa: E402


class FrozenWorld(World):
    """sub の小段の間は脚の指令が出ても pelvis が進まない（扉や枠に体が止められた形）"""

    def __init__(self, p, frozen=("4a",)):
        super().__init__(p)
        self.frozen = frozen

    def run(self, s, seconds=60.0):
        out = None
        for k in range(int(seconds / DT)):
            t = k * DT
            out = s.step(t, self.pose, self.palm, -2.0, float(self.palm[0] - self.pose.x))
            if not self.subs or self.subs[-1] != out.sub:
                self.subs.append(out.sub)
            if out.stop:
                return out
            if out.goal is not None:
                self.follow(out.goal)
            if out.walk[0] and out.sub not in self.frozen:
                self.walked += 0.5 * out.walk[1] * DT
                self.pose = dfr.PlanarPose(self.pose.x + 0.5 * out.walk[1] * DT, self.pose.y, self.pose.yaw)
        return out


def test_old_substage_numbers_are_unchanged_and_4a_is_last():
    assert dscr.SUBS[:9] == ("2a", "2b", "2c", "2d", "3a", "3c", "done", "retreat", "failed")
    assert dscr.SUBS.index("4a") == 9


def test_pass_through_walks_past_the_doorway_plane_and_succeeds():
    p, s = make(pass_through=True, pass_x=0.2)
    w = World(p)
    out = w.run(s, seconds=90.0)
    assert out.success and "成功" in out.stop, out
    assert w.subs[-3:] == ["3c", "4a", "done"], w.subs
    assert w.pose.x >= p.pass_x
    assert s.touching()


def test_without_pass_through_it_still_stops_at_goal_deg():
    p, s = make()
    w = World(p)
    out = w.run(s)
    assert out.success and "4a" not in w.subs and w.subs[-2:] == ["3c", "done"]


def test_4a_keeps_walking_until_the_plane():
    p, s = make(pass_through=True, pass_x=0.2)
    w = World(p)
    walks = []
    orig = s.step

    def step(*a, **k):
        o = orig(*a, **k)
        walks.append((o.sub, o.walk))
        return o

    s.step = step
    w.run(s, seconds=90.0)
    in4a = [wk for sub, wk in walks if sub == "4a"]
    assert in4a and all(wk[0] and wk[1] == p.pass_vx for wk in in4a)
    assert walks[-1][1] == dscr.STAND


def test_4a_fails_when_the_pelvis_does_not_advance():
    # 偽の世界は 3c の間に体が 0.4 m 進む（手のひらの動きで扉の角度を読むので）。線を奥に置いて 4a で歩かせる
    p, s = make(pass_through=True, pass_stall_s=2.0, pass_x=0.2)
    w = FrozenWorld(p)
    out = w.run(s, seconds=90.0)
    assert not out.success and "前へ出ない" in out.stop and w.subs[-2:] == ["4a", "failed"], (out.stop, w.subs)


def test_4a_fails_when_the_door_comes_back():
    p, s = make(pass_through=True, pass_x=0.2)
    w = World(p)
    orig = s.angle_median_deg
    s.angle_median_deg = lambda: 10.0 if s.sub == "4a" else orig()
    out = w.run(s, seconds=90.0)
    assert not out.success and "戻ってきた" in out.stop, out.stop


@pytest.mark.parametrize("bad", [{"pass_x": 0.6}, {"pass_vx": 0.05}, {"pass_min_deg": 35.0}, {"pass_stall_s": 0.0},
                                 {"pass_arc_max_deg": 20.0}])
def test_bad_pass_params_are_rejected(bad):
    with pytest.raises(ValueError):
        dscr.load_params(None, pass_through=True, **bad)


# ── 11-14 walk_from（足踏みを 3a より前から） ────────────────
def test_walk_from_2d_steps_while_pressing_and_skips_the_still_legs_guard():
    p, s = make(walk_in_3a=True, walk_3a_vx=0.2, walk_from="2d")
    w = World(p)
    walks = []
    orig = s.step

    def step(*a, **k):
        o = orig(*a, **k)
        walks.append((o.sub, o.walk[0]))
        return o

    s.step = step
    out = w.run(s, seconds=90.0)
    # 偽の世界は扉に押し返されないので、2d の間に体が 0.1 m/s で手を追い越し、3c で「手が体へ押し込まれた」で止まることがある
    # （実機でも 2d から足踏みすると 2d の 2〜3 秒で 10〜30 cm 寄る＝気を付ける所）。ここでは 2d で歩き、3a を抜けて 3c まで進むことを見る
    assert "3c" in w.subs, (out.stop, w.subs)
    assert any(wk for sub, wk in walks if sub == "2d")
    assert not any(wk for sub, wk in walks if sub in ("2a", "2b", "2c"))
    assert s.x_walk0 is not None  # 3a の空振りの進みは 3a に入った所から数えた


def test_walk_from_default_keeps_legs_still_until_3a():
    p, s = make(walk_in_3a=True, walk_3a_vx=0.2)
    w = World(p)
    walks = []
    orig = s.step

    def step(*a, **k):
        o = orig(*a, **k)
        walks.append((o.sub, o.walk[0]))
        return o

    s.step = step
    w.run(s, seconds=90.0)
    assert not any(wk for sub, wk in walks if sub in ("2a", "2b", "2c", "2d"))


def test_walk_from_rejects_unknown_substage():
    with pytest.raises(ValueError):
        dscr.load_params(None, walk_from="2b")
