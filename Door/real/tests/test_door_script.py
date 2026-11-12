"""11-12 台本（S2・S3・S5）: door_script.ScriptDoor の状態の移りを、偽の手のひら（扉の座標の点）で確かめる。機体は要らない。

偽の世界: 手のひらは行き先へ付いていく。レバーに当たるのは手の先（手のひらの原点の −above[0] 前。11-12 の形は手の先で押す）。
レバーは当てる点の高さでばねのように受け（行き先の半分だけ下がる・4.5 cm で止まる）、3 cm 下がると掛け金が外れる。
外れるまでは手の先は前へ出られない。脚の指令が出ると pelvis が前へ進む（0.5 倍）。
⚠️ 偽の世界は台本の状態の移りを確かめるだけで、機体の当たり方・純正の脚の癖は入っていない（作業ログ 2026-11-12 の 4 章）
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import door_frame as dfr  # noqa: E402
import door_script as dscr  # noqa: E402
import door_staged as ds  # noqa: E402

DOOR = (0.94, 0.064, 0.99)  # 幅・バックセット・レバーの高さ
DT = 0.02


class World:
    def __init__(self, p: dscr.ScriptParams, lever: bool = True, latch_free: bool = True, tau_gain: float = 0.0) -> None:
        self.ref = ds.lever_ref_point(*DOOR, True)
        st = dscr.script_stance(p, *DOOR, True).target
        self.pose = st
        self.palm = np.array([st.x + 0.01, st.y + 0.23, 0.63])  # 腕を下ろした手のひら
        self.reach = -p.above[0]  # 手のひらの原点から、レバーに当たる所（手の先）まで
        self.lever, self.latch_free, self.tau_gain = lever, latch_free, tau_gain
        self.stuck_x: float = math.inf  # 手の先がこれより前へ出られない（扉が動かない形）
        self.depth = 0.0
        self.opened = False
        self.yaw_kicks = {}   # 小段の名前 → 回す角度 [rad]（1 回だけ）
        self.subs, self.walked = [], 0.0

    def follow(self, goal: np.ndarray) -> None:
        d = goal - self.palm
        n = float(np.linalg.norm(d))
        new = goal.copy() if n <= 0.3 * DT else self.palm + d * (0.3 * DT / n)
        tip = new + np.array([self.reach, 0.0, 0.0])
        over = self.lever and np.linalg.norm(tip[:2] - self.ref[:2]) < 0.06 or self.depth > 0
        if self.lever and over and goal[2] < self.ref[2]:
            self.depth = min(0.045, 0.5 * (self.ref[2] - goal[2]))
            new[2] = self.ref[2] - self.depth
        released = self.latch_free and self.depth >= 0.03
        self.opened = self.opened or (released and tip[0] > self.ref[0] + 0.02)  # 開いた扉は掛け金に掛からない
        if self.lever and self.depth > 0 and not released and not self.opened:
            new[0] = min(new[0], self.ref[0] + 0.003 - self.reach)
        new[0] = min(new[0], self.stuck_x - self.reach)
        self.palm = new

    def run(self, s: dscr.ScriptDoor, seconds: float = 60.0) -> dscr.ScriptOut:
        out = None
        for k in range(int(seconds / DT)):
            t = k * DT
            tau = -2.0 - self.tau_gain * self.depth
            out = s.step(t, self.pose, self.palm, tau, float(self.palm[0] - self.pose.x))
            if not self.subs or self.subs[-1] != out.sub:
                self.subs.append(out.sub)
                if out.sub in self.yaw_kicks:
                    self.pose = dfr.PlanarPose(self.pose.x, self.pose.y, self.pose.yaw + self.yaw_kicks.pop(out.sub))
            if out.stop:
                return out
            if out.goal is not None:
                self.follow(out.goal)
            if out.walk[0]:
                self.walked += 0.5 * out.walk[1] * DT
                self.pose = dfr.PlanarPose(self.pose.x + 0.5 * out.walk[1] * DT, self.pose.y, self.pose.yaw)
        return out


def make(**kw):
    p = dscr.load_params(None, **kw)
    return p, dscr.ScriptDoor(p, *DOOR, True)


def test_full_run_presses_lever_unlatches_and_walks_door_to_goal():
    p, s = make()  # 既定＝段 3 は walk（腕は体に対して固め、歩いて押す）
    w = World(p)
    out = w.run(s)
    assert out.success and "成功" in out.stop, out
    assert w.subs == ["2a", "2b", "2c", "2d", "3a", "3c", "done"], w.subs
    assert s.angle_median_deg() >= p.goal_deg
    assert w.walked > 0.10  # 体が前へ出て押している
    assert out.angle_obs[1] == 1.0 and math.degrees(out.angle_obs[0]) >= p.goal_deg


def test_pulse_mode_still_reaches_goal_with_short_walks():
    p, s = make(push_mode="pulse")  # 11-12 の機体の形
    w = World(p)
    out = w.run(s)
    assert out.success, out
    assert w.walked > 0.05  # 手が前へ出すぎた所で脚が前へ出ている


def test_walk_mode_keeps_walking_through_3c_and_stands_at_the_end():
    p, s = make()
    w = World(p)
    walks = []
    orig = s.step

    def step(*a, **k):
        out = orig(*a, **k)
        walks.append((out.sub, out.walk[0]))
        return out

    s.step = step
    out = w.run(s)
    assert out.success
    in3c = [wk for sub, wk in walks if sub == "3c"]
    assert len(in3c) > 2 and all(in3c[1:])                     # 3c の間は歩き続ける（入った周期は 3a の続きで立つ）
    assert not any(wk for sub, wk in walks if sub in ("2a", "2b", "2c", "2d", "3a"))  # 3a までは脚を止める
    assert not walks[-1][1]                                    # 終わりは立つ


def test_walk_mode_fails_when_hand_is_pushed_into_the_body():
    # 扉が動かず、歩くほど手が体へ押し込まれる形（手の前への出が walk_min_fwd を下回る）→ 立ったまま止まる
    p, s = make()
    w = World(p)
    orig = s._enter_3c

    def enter(t, pose, palm):
        w.stuck_x = float(palm[0] + w.reach) + 0.005
        orig(t, pose, palm)

    s._enter_3c = enter
    out = w.run(s)
    assert ("押し込まれた" in out.stop or "開いていかない" in out.stop) and not out.success, out


def test_walk_in_3a_walks_while_unlatching_and_does_not_trip_pelvis_guard():
    p, s = make(walk_in_3a=True)
    w = World(p)
    out = w.run(s)
    assert out.success, out


def test_goal_uses_median_angle_so_a_single_spike_does_not_end_the_run():
    p, s = make(goal_deg=20.0)
    s.palm_ref = np.array([-0.18, 0.30, 0.95])
    s.sub, s.t_sub, s.palm_body = "3c", 0.0, np.array([0.2, 0.2])
    s.angle_best, s.t_progress = 0.0, 0.0
    pose = dfr.PlanarPose(-0.4, 0.08, 0.0)
    line = s.line
    v = s.palm_ref[:2] - line.hinge_xy

    def palm_at(deg):
        a = line.open_sign * math.radians(deg)
        c, sn = math.cos(a), math.sin(a)
        xy = line.hinge_xy + np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1]])
        return np.array([xy[0], xy[1], 0.95])

    for k, deg in enumerate([5, 5, 5, 25, 5, 5]):  # 1 周期だけ 25° に跳ぶ
        out = s.step(k * 0.02, pose, palm_at(5), -2.0, 0.3, palm_angle=palm_at(deg))
        assert out.sub == "3c" and not out.success
    assert s.angle_median_deg() < 10


def test_angle_is_read_from_palm_angle_when_given():
    p, s = make()
    s.palm_ref = np.array([-0.18, 0.30, 0.95])
    s.sub, s.t_sub, s.palm_body = "3c", 0.0, np.array([0.2, 0.2])
    pose = dfr.PlanarPose(-0.4, 0.08, 0.0)
    s.step(0.0, pose, s.palm_ref, -2.0, 0.3, palm_angle=s.palm_ref + np.array([0.05, 0.0, 0.0]))
    assert math.degrees(s.angle) > 2.0  # 行き先の基準（palm）は動いていなくても、角度は palm_angle で読む


def test_release_goal_moves_back_and_up_and_touching_follows_contact():
    p, s = make()
    palm = np.array([-0.18, 0.30, 0.95])
    g = s.release_goal(palm)
    assert np.allclose(g - palm, [-p.release_back, 0.0, p.release_up])
    assert not s.touching()        # 2a・触れていない
    s.sub = "2b"
    assert not s.touching()
    s.sub = "2d"
    assert s.touching()


def test_defaults_match_shipped_params_file_and_robot_file_differs_only_in_first_via():
    here = Path(__file__).resolve().parents[1]
    shipped = dscr.load_params(str(here / "script_params.json"))
    robot = dscr.load_params(str(here / "script_params_1112_robot.json"))
    from dataclasses import replace
    assert replace(shipped, note="") == replace(dscr.ScriptParams(), note="")
    diff = {f: (getattr(robot, f), getattr(shipped, f)) for f in dscr.ScriptParams.__dataclass_fields__
            if f != "note" and getattr(robot, f) != getattr(shipped, f)}
    assert set(diff) == {"via"} and robot.via[1:] == shipped.via[1:], diff  # 11-12 夜に変えたのは最初の経由点の高さだけ


def test_arm_stays_down_and_legs_stand_while_waiting():
    p, s = make()
    w = World(p)
    out = s.step(0.0, w.pose, w.palm, -2.0, 0.3)
    assert out.sub == "2a" and out.goal is None and out.walk == dscr.STAND and not out.stop
    assert np.array_equal(out.angle_obs, dfr.door_closed_angle_obs())


def test_miss_when_no_lever_under_the_hand():
    # 触れたを高さで決めない時（touch_by_height=False）は、レバーが無ければ当てる点の下まで下ろして空振りで止まる
    p, s = make(touch_by_height=False)
    w = World(p, lever=False)
    out = w.run(s)
    assert "空振り" in out.stop and w.subs[-1] == "failed" and "3a" not in w.subs


def test_touch_is_detected_by_shoulder_torque_too():
    # 遅れ・高さでは立たないようにして、力だけで触れたと分かるか（偽のレバーは 4.5 cm しか下がらないので押し下げの線も合わせる）
    p, s = make(lag_line=1.0, touch_by_height=False, press_depth=0.04, press_min=0.02)
    w = World(p, tau_gain=400.0)
    out = w.run(s)
    assert out.success, out


@pytest.mark.parametrize("sub", ["2a", "2b", "2c", "2d", "3a"])
def test_stop_after_stops_right_after_that_substage(sub):
    p, s = make(stop_after=sub)
    w = World(p)
    out = w.run(s)
    assert "止める所" in out.stop and not out.success
    assert w.subs[-1] == sub and w.walked == 0.0


def test_latch_stuck_nudges_legs_once_then_fails():
    p, s = make()
    w = World(p, latch_free=False)
    out = w.run(s)
    assert "掛け金が外れない" in out.stop
    assert s.nudged and abs(w.walked - 0.5 * p.nudge_vx * p.nudge_s) < 0.02  # 脚で押すのは 1 回だけ


def test_body_yaw_during_press_retreats_once_and_retries():
    p, s = make()
    w = World(p)
    w.yaw_kicks = {"2d": math.radians(12.0)}
    out = w.run(s)
    assert w.subs.count("retreat") == 1 and w.subs.count("2a") == 2
    assert out.success, out  # やり直しで通る


def test_body_yaw_twice_fails_after_retries_are_used_up():
    p, s = make(retries=0)
    w = World(p)
    w.yaw_kicks = {"2d": math.radians(12.0)}
    out = w.run(s)
    assert "体が" in out.stop and "retreat" not in w.subs


def test_shoulder_torque_over_limit_stops():
    p, s = make(tau_max=5.0, tau_max_s=0.5, lag_line=1.0, tau_line=100.0, touch_by_height=False)  # 触れたと分からないまま押し続ける形
    w = World(p, tau_gain=400.0)
    out = w.run(s)
    assert "上限" in out.stop


def test_non_finite_palm_fails_instead_of_sending_goal():
    p, s = make()
    w = World(p)
    out = s.step(0.0, w.pose, np.array([np.nan, 0.0, 0.0]), -2.0, 0.3)
    assert "有限でない" in out.stop


def test_goal_never_leads_palm_by_more_than_lead_max_once_touching():
    p, s = make()
    w = World(p)
    lead = []
    orig = w.follow

    def follow(goal):
        if s.sub in ("2c", "2d", "3a", "3c"):
            lead.append(max(abs(float(goal[2] - w.palm[2])), float(np.linalg.norm(goal[:2] - w.palm[:2]))))
        orig(goal)

    w.follow = follow
    w.run(s)
    assert lead and max(lead) <= p.lead_max + 1e-6


def test_row_has_fixed_length_and_substage_index():
    p, s = make()
    w = World(p)
    s.step(0.0, w.pose, w.palm, -2.0, 0.3)
    r = s.row()
    assert r.shape == (dscr.ROW_LEN,) and r[0] == dscr.SUBS.index("2a")


def test_load_params_reads_json_and_rejects_unknown_or_bad_values(tmp_path):
    f = tmp_path / "p.json"
    f.write_text(json.dumps({"press_depth": 0.05, "via": [[-0.1, 0, -0.2]], "stop_after": "2b"}), encoding="utf-8")
    p = dscr.load_params(str(f), stop_after="2c")  # 引数が定数ファイルより優先
    assert p.press_depth == 0.05 and p.via == ((-0.1, 0.0, -0.2),) and p.stop_after == "2c"
    f.write_text(json.dumps({"press_dept": 0.05}), encoding="utf-8")
    with pytest.raises(ValueError, match="知らない名前"):
        dscr.load_params(str(f))
    for bad in ({"walk_vx": 0.1}, {"stop_after": "3c"}, {"press_min": 0.1}, {"stance_x": -0.2}, {"posture": [0, 0, 0]},
                {"push_mode": "run"}, {"push_mode": "walk", "walk_vx": 0.0}, {"release_back": 0.2}):
        with pytest.raises(ValueError):
            dscr.load_params(None, **bad)


def test_shipped_params_file_loads_and_matches_defaults():
    p = dscr.load_params(str(Path(__file__).resolve().parents[1] / "script_params.json"))
    assert dscr.script_stance(p, *DOOR, True).target.x == p.stance_x


def test_script_stance_sits_beside_lever_facing_the_door():
    p = dscr.ScriptParams()
    st = dscr.script_stance(p, *DOOR, True)
    ref = ds.lever_ref_point(*DOOR, True)
    assert abs(st.target.y - (ref[1] + p.stance_y_off)) < 1e-9 and st.target.yaw == 0.0
    mirrored = dscr.script_stance(p, *DOOR, False)
    assert abs(mirrored.target.y + st.target.y) < 1e-9


def test_pose_filter_smooths_steps_and_wraps_yaw():
    f = dscr.PoseFilter(0.2)
    f.step(dfr.PlanarPose(0.0, 0.0, math.radians(179)))
    out = f.step(dfr.PlanarPose(0.1, 0.0, math.radians(-179)))
    assert 0.0 < out.x < 0.1
    assert abs(math.degrees(ds.wrap(out.yaw - math.radians(179)))) < 1.0  # 360° を跨いでも近い方へ


def test_blocked_hand_while_raising_fails_fast_instead_of_pushing():
    # 11-12 実機 1 本目: 手の先がレバーの棒の下に掛かったまま 5 秒押し上げ、肩 pitch が 53→76 °C になった
    p, s = make()
    w = World(p)
    orig = w.follow

    def follow(goal):  # 当てる点の 5 cm 下に天井がある（棒の下に掛かった形）
        orig(goal)
        w.palm[2] = min(w.palm[2], w.ref[2] - 0.05)

    w.follow = follow
    t_block = [None]
    out = None
    for k in range(int(20 / DT)):
        t = k * DT
        out = s.step(t, w.pose, w.palm, -17.0, 0.2)
        if out.stop:
            break
        if out.goal is not None:
            w.follow(out.goal)
            assert np.abs(out.goal - w.palm).max() <= p.lead_max + 0.01  # 行き先は手のひらの 5 cm 先で止まる
        if t_block[0] is None and w.palm[2] >= w.ref[2] - 0.05 - 1e-9 and s.sub == "2b" and s.wp_i == 1:
            t_block[0] = t
    assert "進まない" in out.stop and t_block[0] is not None
    assert k * DT - t_block[0] < p.raise_stall_s + 0.5


def test_pelvis_pushed_back_while_legs_are_stopped_fails():
    p, s = make()
    w = World(p)
    out = None
    for k in range(int(20 / DT)):
        out = s.step(k * DT, w.pose, w.palm, -2.0, 0.2)
        if out.stop:
            break
        if out.goal is not None:
            w.follow(out.goal)
        if s.sub == "2b":
            w.pose = dfr.PlanarPose(w.pose.x - 0.002, w.pose.y, w.pose.yaw)  # 毎周期 2 mm ずつ後ろへ
    assert "pelvis が" in out.stop
