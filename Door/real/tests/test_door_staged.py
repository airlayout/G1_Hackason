"""door_staged（11-10 手順 L4 の本体）の試験: 段 0 の測りと見張り・構えの位置・段 1 の寄せ・段 2 の脚の放しと引き継ぎ・持ち替え。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import door_frame as dfr  # noqa: E402
import door_staged as ds  # noqa: E402

W, BS, LH = 0.94, 0.064, 0.99


# ── 網の扉の値 → pelvis の姿勢 ─────────────────────────────────
@pytest.mark.parametrize("roll, pitch", [(0.0, 0.0), (0.03, -0.05)])
def test_pose_from_frame_loc_inverts_door_frame_loc(roll: float, pitch: float) -> None:
    pose = dfr.PlanarPose(-0.9, 0.12, math.radians(-20))
    # IMU の四元数（wxyz）を roll・pitch・yaw から（yaw は測位に入れるので何でもよい）
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    quat = np.array([cr * cp, sr * cp, cr * sp, -sr * sp])
    loc = dfr.door_frame_loc(pose, quat)
    r, p = dfr.roll_pitch_from_quat_wxyz(quat)
    got = ds.pose_from_frame_loc(loc, r, p)
    assert (got.x, got.y, got.yaw) == pytest.approx((pose.x, pose.y, pose.yaw), abs=1e-9)


def test_median_pose_handles_yaw_near_pi() -> None:
    poses = [dfr.PlanarPose(0.0, 0.0, math.radians(a)) for a in (179, -179, 178, -178, 180)]
    got = ds.median_pose(poses)
    assert abs(math.degrees(math.atan2(math.sin(got.yaw), math.cos(got.yaw)))) == pytest.approx(180, abs=1.01)


# ── 段 0 ───────────────────────────────────────────────────
def test_pose_diff_and_average() -> None:
    a = dfr.PlanarPose(-1.0, 0.0, math.radians(2))
    b = dfr.PlanarPose(-0.94, 0.08, math.radians(-2))
    dpos, dyaw = ds.pose_diff(a, b)
    assert dpos == pytest.approx(0.1) and dyaw == pytest.approx(4.0)
    m = ds.average_pose(a, b)
    assert (m.x, m.y, math.degrees(m.yaw)) == pytest.approx((-0.97, 0.04, 0.0), abs=1e-9)


def run_stage0(net_pose_fn, carried_fn, ready_after_s: float = 3.5, t_end: float = 30.0, dt: float = 0.02):
    """段 0 を回す。net_pose_fn(t)・carried_fn(t) は網・FAST-LIO の口の pelvis の姿勢（無ければ None）"""
    st = ds.Stage0()
    ref_t = None
    outs = []
    t = 0.0
    while t < t_end and not (st.done or st.failed):
        ready = ref_t is not None and t - ref_t >= ready_after_s
        out = st.step(t, net_pose_fn(t), carried_fn(t) if ready else None, ready)
        if out.ref is not None:
            ref_t = t
        outs.append(out)
        t += dt
    return st, outs


def test_stage0_measures_twice_and_sends_average() -> None:
    # Arrange: 網は 2 回とも同じ扉を見ている。2 回目の前に 0.2 m 前へ出た（FAST-LIO で運んだ値も同じ）
    moved = {"x": 0.0}

    def net(t):
        return dfr.PlanarPose(-1.0 + moved["x"], 0.05, math.radians(1.0))

    def carried(t):
        return dfr.PlanarPose(-1.0 + moved["x"] + 0.01, 0.05, math.radians(1.0))

    st = ds.Stage0()
    refs, t, ref_t = [], 0.0, None
    while t < 40 and not (st.done or st.failed):
        ready = ref_t is not None and t - ref_t >= 3.5
        out = st.step(t, net(t), carried(t) if ready else None, ready)
        if out.cmd[0] > 0:
            moved["x"] = moved["x"] + 0.8 * out.cmd[0] * 0.02  # 純正は 0.2 m/s に 0.8 倍で応える（11-11）
        if out.ref is not None:
            refs.append(out.ref)
            ref_t = t
        t += 0.02
    # Assert: REF は 2 回（1 回目の網の値・2 回目は網と運んだ値の平均）
    assert st.done and not st.failed
    assert len(refs) == 2
    assert refs[0].x == pytest.approx(-1.0, abs=1e-9)
    assert refs[1].x == pytest.approx(-1.0 + moved["x"] + 0.005, abs=1e-6)
    assert moved["x"] == pytest.approx(0.2, abs=0.03)


def test_stage0_fails_when_two_measurements_disagree() -> None:
    # 2 回目の網が 10 cm 外れた（線 8 cm）
    phase = {"second": False}

    def net(t):  # 0.2 m 前へ出た後の 2 回目だけ 10 cm 外れる
        return dfr.PlanarPose(-0.7 if phase["second"] else -1.0, 0.0, 0.0)

    st = ds.Stage0()
    t, ref_t = 0.0, None
    while t < 40 and not (st.done or st.failed):
        ready = ref_t is not None and t - ref_t >= 3.5
        out = st.step(t, net(t), dfr.PlanarPose(-0.8, 0.0, 0.0) if ready else None, ready)
        if out.ref is not None:
            ref_t = t
        phase["second"] = st.phase == "measure2"
        t += 0.02
    assert st.failed and "食い違" in st.reason


def test_stage0_fails_when_robot_did_not_step_forward() -> None:
    # 純正が前への指令に応えなかった（FAST-LIO で運んだ値が動いていない）: 運びの確かめが空振りなので止まる
    st, _ = run_stage0(lambda t: dfr.PlanarPose(-1.0, 0.0, 0.0), lambda t: dfr.PlanarPose(-1.0, 0.0, 0.0))
    assert st.failed and "前へ出ていない" in st.reason


def test_stage0_fails_without_net_samples() -> None:
    st, _ = run_stage0(lambda t: None, lambda t: None)
    assert st.failed and "網" in st.reason


def test_stage0_rejects_too_far_or_angled_start() -> None:
    # 段 0 の立ち位置の条件（扉の正面 0.8〜1.3 m・向き ±10°）の外
    st, _ = run_stage0(lambda t: dfr.PlanarPose(-1.6, 0.0, 0.0), lambda t: None)
    assert st.failed and "立ち位置" in st.reason


# ── 構えの位置・レバーの点 ────────────────────────────────────
@pytest.mark.parametrize("lever_left", [True, False])
def test_stance_target_inside_s2f_start_range(lever_left: bool) -> None:
    st = ds.stance(W, BS, lever_left)
    lr = 1.0 if lever_left else -1.0
    pivot = lr * (W / 2 - BS)
    x0, x1, y0, y1, w0, w1 = ds.S2F_START_RANGE
    # 目標と許す幅の角が、s2f が学んだ範囲の内側
    for dx in (-st.tol_x, st.tol_x):
        assert x0 <= st.target.x + dx <= x1
    for dy in (-st.tol_y, st.tol_y):
        off = lr * (st.target.y + dy - pivot)
        assert y0 <= off <= y1
    for dw in (-st.tol_yaw, st.tol_yaw):
        yaw = lr * math.degrees(st.target.yaw + dw)
        assert w0 <= yaw <= w1
    assert st.contains(st.target)
    assert not st.contains(dfr.PlanarPose(st.target.x + 2 * st.tol_x, st.target.y, st.target.yaw))


@pytest.mark.parametrize("lever_left", [True, False])
def test_lever_ref_point(lever_left: bool) -> None:
    p = ds.lever_ref_point(W, BS, LH, lever_left)
    lr = 1.0 if lever_left else -1.0
    assert p == pytest.approx([-0.078, lr * (W / 2 - BS - 0.105), LH + 0.03], abs=1e-9)


# ── 段 1 の寄せ ───────────────────────────────────────────────
GAIN = np.array([0.8, 0.8, 0.61])  # 純正が指令に応える倍率（前後・左右・旋回・11-06）
DEAD = np.array([0.15, 0.15, 0.2])  # これより小さい指令では立ったまま


def simulate_approach(start: dfr.PlanarPose, t_end: float = 60.0, dt: float = 0.02, noise=None):
    """純正の鈍さと効かない帯を真似た機体で段 1 を回す（体の座標の指令 → 扉の座標で動く）"""
    st = ds.stance(W, BS, True)
    ap = ds.Approach(st)
    pose = start
    t = 0.0
    rng = np.random.default_rng(0)
    while t < t_end and not (ap.done or ap.failed):
        seen = pose if noise is None else dfr.PlanarPose(pose.x + noise * rng.standard_normal(),
                                                         pose.y + noise * rng.standard_normal(), pose.yaw)
        cmd = np.array(ap.step(t, seen))
        assert all(c == 0.0 or abs(c) >= m - 1e-12 for c, m in zip(cmd, (ds.V_MIN, ds.V_MIN, ds.WZ_MIN)))
        v = np.where(np.abs(cmd) >= DEAD, cmd * GAIN, 0.0)
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        pose = dfr.PlanarPose(pose.x + (c * v[0] - s * v[1]) * dt, pose.y + (s * v[0] + c * v[1]) * dt, pose.yaw + v[2] * dt)
        t += dt
    return ap, pose, t


@pytest.mark.parametrize("start", [dfr.PlanarPose(-0.9, 0.0, 0.0), dfr.PlanarPose(-1.1, 0.2, math.radians(5)),
                                   dfr.PlanarPose(-0.8, -0.15, math.radians(-8))])
def test_approach_reaches_stance(start: dfr.PlanarPose) -> None:
    ap, pose, t = simulate_approach(start)
    assert ap.done and not ap.failed, ap.reason
    assert ds.stance(W, BS, True).contains(pose)
    assert t < 60.0


def test_approach_turns_before_moving_sideways() -> None:
    # 止まって測った後、先に回る（向きが目標の 5° 以内に入るまで並進しない）
    st = ds.stance(W, BS, True)
    ap = ds.Approach(st)
    t = 0.0
    while ap.phase == "settle":
        cmd = ap.step(t, dfr.PlanarPose(-0.9, 0.0, 0.0))
        assert cmd == (0.0, 0.0, 0.0)
        t += 0.02
    cmd = ap.step(t, dfr.PlanarPose(-0.9, 0.0, 0.0))
    assert cmd[0] == 0.0 and cmd[1] == 0.0 and cmd[2] < 0.0


def test_approach_uses_average_while_standing_not_noisy_instant() -> None:
    # 構えの位置で ±5 cm 揺れる姿勢でも、止まった間の中央値で「入った」と判定する
    st = ds.stance(W, BS, True)
    ap = ds.Approach(st)
    for k in range(200):
        n = 0.05 * math.sin(k * 0.9)
        ap.step(0.02 * k, dfr.PlanarPose(st.target.x + n, st.target.y - n, st.target.yaw))
        if ap.done:
            break
    assert ap.done


def test_approach_stops_when_too_close_to_door() -> None:
    st = ds.stance(W, BS, True)
    ap = ds.Approach(st)
    assert ap.step(0.0, dfr.PlanarPose(-0.25, 0.3, st.target.yaw)) == (0.0, 0.0, 0.0)
    assert ap.failed and "近" in ap.reason


def test_approach_fails_when_pose_stops_coming() -> None:
    st = ds.stance(W, BS, True)
    ap = ds.Approach(st)
    ap.step(0.0, dfr.PlanarPose(-0.9, 0.0, 0.0))
    assert ap.step(0.3, None) == (0.0, 0.0, 0.0) and not ap.failed
    ap.step(0.7, None)
    assert ap.failed and "位置" in ap.reason


def test_approach_times_out() -> None:
    st = ds.stance(W, BS, True)
    ap = ds.Approach(st)
    for k in range(0, int(61 / 0.1)):
        ap.step(0.1 * k, dfr.PlanarPose(-0.9, 0.0, 0.0))  # 何を出しても動かない機体
        if ap.failed:
            break
    assert ap.failed and "秒" in ap.reason


# ── 段 2 ───────────────────────────────────────────────────
def test_arm_stage_releases_legs_and_hands_over_at_3deg() -> None:
    # Arrange
    arm = ds.ArmStage(W, BS, LH, lever_left=True)
    ref = arm.lever_ref
    # Act 1: 手がレバーに触れて押し下げる（2 cm 下・力 8 N を 5 歩）→ 脚を放す
    for k in range(5):
        out = arm.step(0.02 * k, ref + np.array([0.0, 0.0, -0.03]), 8.0)
    assert out.legs_free and not out.handover
    # Act 2: 扉と一緒に手が 3.2° 回る → 引き継ぎ
    line = arm.est.line
    v = (ref + np.array([0.0, 0.0, -0.03]))[:2] - line.hinge_xy
    a = line.open_sign * math.radians(3.2)
    c, s = math.cos(a), math.sin(a)
    moved = np.array([*(line.hinge_xy + [c * v[0] - s * v[1], s * v[0] + c * v[1]]), ref[2] - 0.03])
    out = arm.step(0.2, moved, 8.0)
    # Assert
    assert out.handover and math.degrees(out.angle_obs[0]) == pytest.approx(3.2, abs=0.05)


def test_arm_stage_keeps_legs_fixed_before_press() -> None:
    arm = ds.ArmStage(W, BS, LH, lever_left=True)
    out = arm.step(0.0, arm.lever_ref + np.array([0.0, 0.0, 0.05]), 8.0)  # 上から触れただけ
    assert not out.legs_free
    np.testing.assert_allclose(out.angle_obs, dfr.door_closed_angle_obs())


def test_arm_stage_times_out() -> None:
    arm = ds.ArmStage(W, BS, LH, lever_left=True)
    arm.step(0.0, arm.lever_ref + np.array([0.3, 0.0, 0.0]), 0.0)
    out = arm.step(ds.STAGE2_MAX_S + 0.1, arm.lever_ref + np.array([0.3, 0.0, 0.0]), 0.0)
    assert out.failed and "秒" in arm.reason


def test_arm_stage_without_force_uses_geometry_only() -> None:
    # 力がまだ測れない（L2c の前）: 力の線を 0 にして、位置と下がりだけで放す。ただし手のひらが上から来てから
    arm = ds.ArmStage(W, BS, LH, lever_left=True, force_known=False)
    arm.step(0.0, arm.lever_ref + np.array([0.0, 0.0, 0.01]), None)  # 棒の上
    for k in range(1, 6):
        out = arm.step(0.02 * k, arm.lever_ref + np.array([0.0, 0.0, -0.03]), None)
    assert out.legs_free


def test_arm_stage_without_force_ignores_hand_resting_below_lever() -> None:
    # 構えた腕の手のひらが初めから当てる点の近く・下にある（学習の初めの姿勢で水平 8.8 cm・17.5 cm 下）: 押していないので放さない
    arm = ds.ArmStage(W, BS, LH, lever_left=True, force_known=False)
    low = arm.lever_ref + np.array([-0.079, -0.038, -0.175])
    for k in range(50):
        out = arm.step(0.02 * k, low, None)
    assert not out.legs_free and not arm.armed
    np.testing.assert_allclose(out.angle_obs, dfr.door_closed_angle_obs())


# ── 持ち替え・運び ────────────────────────────────────────────
def test_handover_state_keeps_accumulator_and_zeros_memory() -> None:
    import door_core as dc
    s = dc.CoreState(h=np.ones((2, 256)), cell=np.ones((2, 256)), D=np.arange(17.0), a_real_prev=np.ones(17),
                     whole_prev=np.ones(29), cmd_prev=np.ones(7), homie_hist=np.ones((5, 86)), homie_a_prev=np.ones(15))
    z = ds.handover_state(s, "zero")
    assert not z.h.any() and not z.cell.any()
    np.testing.assert_allclose(z.D, s.D)
    np.testing.assert_allclose(z.homie_hist, s.homie_hist)
    with pytest.raises(ValueError):
        ds.handover_state(s, "shadow")  # 影で回すのは走らせ役が f3t3 の状態を別に持つ時（ここでは扱わない）


def test_zero_walk_accumulator() -> None:
    import door_core as dc
    s = dc.CoreState(h=np.zeros((2, 256)), cell=np.zeros((2, 256)), D=np.arange(17.0), a_real_prev=np.ones(17),
                     whole_prev=np.ones(29), cmd_prev=np.ones(7), homie_hist=np.ones((5, 86)), homie_a_prev=np.ones(15))
    z = ds.zero_walk_accumulator(s)
    np.testing.assert_allclose(z.D[:3], 0.0)
    np.testing.assert_allclose(z.D[3:], s.D[3:])


def test_odom_carry_from_handover_pose() -> None:
    carry = ds.OdomCarry(dfr.PlanarPose(-0.4, 0.3, math.radians(-45)), dfr.PlanarPose(10.0, 5.0, math.radians(90)))
    # odom の上で、その時の向き（90°）に 0.2 m 前へ出た → 扉の座標では −45° の向きに 0.2 m
    got = carry.pose(dfr.PlanarPose(10.0, 5.2, math.radians(90)))
    assert (got.x, got.y) == pytest.approx((-0.4 + 0.2 * math.cos(math.radians(-45)), 0.3 + 0.2 * math.sin(math.radians(-45))))
