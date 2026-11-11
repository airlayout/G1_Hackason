"""door_run_staged（11-10 手順 L4 の差し込み口）の試験: 偽の FAST-LIO の受け口・偽の網で、段 0 → 1 → 2 → 引き継ぎを回す。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import door_frame as dfr  # noqa: E402
import door_run_staged as drst  # noqa: E402
import door_staged as ds  # noqa: E402

W, H, LH, BS = 0.94, 2.05, 0.99, 0.064


class World:
    """機体の本当の姿勢（扉の座標）。指令で動く（純正の鈍さを真似る）"""

    def __init__(self, pose: dfr.PlanarPose) -> None:
        self.pose = pose

    def move(self, cmd, dt: float) -> None:
        walk, vx, vy, wz = cmd
        if not walk:
            return
        g = (0.52, 0.36, 0.61)
        v = [c * k if abs(c) >= m else 0.0 for c, k, m in zip((vx, vy, wz), g, (0.15, 0.15, 0.2))]
        c, s = math.cos(self.pose.yaw), math.sin(self.pose.yaw)
        self.pose = dfr.PlanarPose(self.pose.x + (c * v[0] - s * v[1]) * dt, self.pose.y + (s * v[0] + c * v[1]) * dt,
                                   self.pose.yaw + v[2] * dt)


class FakeLidar:
    """FAST-LIO の受け口の代わり: REF を受けてから 3.5 秒で準備ができ、以後は本当の姿勢を返す"""

    def __init__(self, world: World) -> None:
        self.world, self.ref_t, self.refs, self.head_xy = world, None, [], (0.0, 0.0)

    def request_ref(self, lidar_pose=None) -> None:
        self.refs.append(lidar_pose)
        self.ref_t = self.now

    def note_state(self, now, odom, q_waist, roll, pitch) -> None:
        self.now = now

    def ready(self, now) -> bool:
        return self.ref_t is not None and now - self.ref_t >= 3.5

    def pose(self, now, odom):
        return (self.world.pose if self.ready(now) else None), ""


class FakeNet:
    """網の代わり: 10 Hz で本当の姿勢（＋片寄り）を返す"""

    def __init__(self, world: World, bias=(0.0, 0.0, 0.0)) -> None:
        self.world, self.bias, self.next_t = world, bias, 0.0

    def sample(self, now, q, quat, obs):
        if now < self.next_t:
            return None
        self.next_t = now + 0.1
        p = self.world.pose
        return dfr.PlanarPose(p.x + self.bias[0], p.y + self.bias[1], p.yaw + self.bias[2])


def make(world: World, net_bias=(0.0, 0.0, 0.0), palm_fn=None):
    lidar = FakeLidar(world)
    sd = drst.StagedDoor(width=W, height=H, lever_height=LH, backset=BS, lever_left=True, lidar=lidar,
                         net=FakeNet(world, net_bias), palm_fn=palm_fn or (lambda q, pose, quat: np.array([0.3, 0.0, 0.8])),
                         force_known=False)
    return sd, lidar


def run(sd: drst.StagedDoor, world: World, t_end: float, dt: float = 0.02, until_stage: int = 99):
    q, quat = np.zeros(29), np.array([1.0, 0.0, 0.0, 0.0])
    sd.begin(0.0)
    t, decs = 0.0, []
    while t < t_end:
        d = sd.decide(t, q, quat, odom=(world.pose.x, world.pose.y, world.pose.yaw), odom_t=t, obs=None)
        decs.append(d)
        if d.stop or d.stage >= until_stage:
            break
        world.move(d.walk if d.walk is not None else (False, 0.0, 0.0, 0.0), dt)
        t += dt
    return decs


def test_stage0_then_approach_reaches_stance_and_prepares_arms() -> None:
    # Arrange: 扉の正面 1.0 m に立っている
    world = World(dfr.PlanarPose(-1.0, 0.05, 0.0))
    sd, lidar = make(world)
    # Act
    decs = run(sd, world, 90.0, until_stage=2)
    # Assert: 段 0 で REF を 2 回（ライダーの姿勢に直して）・段 1 で構えの位置へ・段 2 の前に腕を構える
    assert not decs[-1].stop, decs[-1].stop
    assert len(lidar.refs) == 2 and all(r is not None for r in lidar.refs)
    assert ds.stance(W, BS, True).contains(world.pose)
    stages = [d.stage for d in decs]
    assert stages[0] == 0 and 1 in stages and stages[-1] == 2
    assert all(d.arm == "home" for d in decs if d.stage < 2)
    assert all(d.mode == "hold" for d in decs if d.stage < 2)


def test_stage0_disagreement_stops() -> None:
    # 網が 2 回目だけ 10 cm 外れる → 立ったまま止まる
    world = World(dfr.PlanarPose(-1.0, 0.0, 0.0))
    sd, _ = make(world)
    net = sd.net

    class Shifting:
        def sample(self, now, q, quat, obs):
            p = net.sample(now, q, quat, obs)
            if p is None or sd.stage0.phase != "measure2":
                return p
            return dfr.PlanarPose(p.x + 0.1, p.y, p.yaw)

    sd.net = Shifting()
    decs = run(sd, world, 40.0)
    assert decs[-1].stop and "食い違" in decs[-1].stop
    assert decs[-1].walk == (False, 0.0, 0.0, 0.0)


def test_stage2_zeroes_walk_until_release_then_hands_over() -> None:
    # Arrange: 構えの位置に入った所から（段 2 の腕の構えを飛ばす）
    world = World(ds.stance(W, BS, True).target)
    ref = ds.lever_ref_point(W, BS, LH, True)
    palm = {"p": ref + np.array([0.05, 0.0, 0.05])}
    sd, _ = make(world, palm_fn=lambda q, pose, quat: palm["p"])
    sd.begin(0.0)
    sd._enter_stage2(0.0)
    q, quat = np.zeros(29), np.array([1.0, 0.0, 0.0, 0.0])
    sd.lidar.ref_t = -10.0
    sd.lidar.note_state(0.0, None, None, 0, 0)
    t = 0.0
    d = sd.decide(t, q, quat, odom=(0.0, 0.0, 0.0), odom_t=t, obs=None)
    # 脚を放す前: 方策は腕の段・歩きの積み上げは 0・歩かない
    assert d.stage == 2 and d.mode == "door" and d.policy == "arm" and d.zero_walk and d.walk == (False, 0.0, 0.0, 0.0)
    # レバーを押し下げる（2 cm 下を 5 歩）→ 脚を放す
    palm["p"] = ref + np.array([0.0, 0.0, -0.03])
    for k in range(1, 6):
        d = sd.decide(0.02 * k, q, quat, odom=(0.0, 0.0, 0.0), odom_t=0.02 * k, obs=None)
    assert not d.zero_walk and d.walk is None
    # 扉と一緒に手が 3.5° 回る → 引き継ぎ（この周期から f3t3・状態は持ち替え）
    line = sd.arm.est.line
    v = palm["p"][:2] - line.hinge_xy
    a = line.open_sign * math.radians(3.5)
    palm["p"] = np.array([*(line.hinge_xy + [math.cos(a) * v[0] - math.sin(a) * v[1], math.sin(a) * v[0] + math.cos(a) * v[1]]),
                          palm["p"][2]])
    d = sd.decide(0.2, q, quat, odom=(0.0, 0.0, 0.0), odom_t=0.2, obs=None)
    assert d.handover_now and d.stage == 3 and d.policy == "f3t3"
    # 段 3: 枠は引き継いだ時の値を odom で運ぶ（odom で 0.2 m 前へ）
    d = sd.decide(0.4, q, quat, odom=(0.2, 0.0, 0.0), odom_t=0.4, obs=None)
    start = sd.carry.start
    assert d.pose.x == pytest.approx(start.x + 0.2 * math.cos(start.yaw), abs=1e-9)


def test_stage1_watch_reason_stops() -> None:
    world = World(dfr.PlanarPose(-1.0, 0.0, 0.0))
    sd, lidar = make(world)
    decs = run(sd, world, 30.0, until_stage=1)
    assert decs[-1].stage == 1
    lidar.pose = lambda now, odom: (world.pose, "ライダーと odom が食い違った")
    d = sd.decide(30.0, np.zeros(29), np.array([1.0, 0.0, 0.0, 0.0]), odom=(0.0, 0.0, 0.0), odom_t=30.0, obs=None)
    assert d.stop and "食い違" in d.stop


def test_stage3_stops_when_odom_is_lost() -> None:
    world = World(ds.stance(W, BS, True).target)
    sd, _ = make(world)
    sd.begin(0.0)
    sd._enter_stage3(0.0, world.pose, (0.0, 0.0, 0.0))
    q, quat = np.zeros(29), np.array([1.0, 0.0, 0.0, 0.0])
    d = sd.decide(0.1, q, quat, odom=None, odom_t=None, obs=None)
    assert not d.stop
    d = sd.decide(0.1 + drst.ODOM_LOST_S + 0.05, q, quat, odom=None, odom_t=None, obs=None)
    assert d.stop and "odom" in d.stop


def test_handover_waits_for_fresh_odom_then_stops() -> None:
    # 3° に届いた周期に odom が無い: 持ち替えを待ち、ODOM_LOST_S を越えたら立ったまま止まる（段 3 の枠を運べない）
    world = World(ds.stance(W, BS, True).target)
    ref = ds.lever_ref_point(W, BS, LH, True)
    palm = {"p": ref + np.array([0.0, 0.0, 0.01])}
    sd, _ = make(world, palm_fn=lambda q, pose, quat: palm["p"])
    sd.begin(0.0)
    sd._enter_stage2(0.0)
    sd.lidar.ref_t = -10.0
    sd.lidar.note_state(0.0, None, None, 0, 0)
    q, quat = np.zeros(29), np.array([1.0, 0.0, 0.0, 0.0])
    sd.decide(0.0, q, quat, odom=(0.0, 0.0, 0.0), odom_t=0.0, obs=None)
    palm["p"] = ref + np.array([0.0, 0.0, -0.03])
    for k in range(1, 6):
        sd.decide(0.02 * k, q, quat, odom=(0.0, 0.0, 0.0), odom_t=0.02 * k, obs=None)
    line = sd.arm.est.line
    v = palm["p"][:2] - line.hinge_xy
    a = line.open_sign * math.radians(3.5)
    palm["p"] = np.array([*(line.hinge_xy + [math.cos(a) * v[0] - math.sin(a) * v[1], math.sin(a) * v[0] + math.cos(a) * v[1]]),
                          palm["p"][2]])
    d = sd.decide(0.2, q, quat, odom=None, odom_t=None, obs=None)
    assert d.stage == 2 and not d.handover_now and not d.stop
    d = sd.decide(0.3, q, quat, odom=(0.1, 0.0, 0.0), odom_t=0.3, obs=None)  # 新しい odom が来た周期に持ち替える
    assert d.handover_now and d.stage == 3
    assert sd.carry.odom_start == dfr.PlanarPose(0.1, 0.0, 0.0)
    # 別の回: 来ないまま ODOM_LOST_S を越える
    sd2, _ = make(World(ds.stance(W, BS, True).target), palm_fn=lambda q, pose, quat: palm["p"])
    sd2.begin(0.0)
    sd2._enter_stage2(0.0)
    sd2.lidar.ref_t = -10.0
    sd2.lidar.note_state(0.0, None, None, 0, 0)
    sd2.arm.handover = True
    sd2.decide(1.0, q, quat, odom=None, odom_t=None, obs=None)
    d = sd2.decide(1.0 + drst.ODOM_LOST_S + 0.05, q, quat, odom=None, odom_t=None, obs=None)
    assert d.stop and "odom" in d.stop


def test_stage0_stops_on_fastlio_watch() -> None:
    world = World(dfr.PlanarPose(-1.0, 0.0, 0.0))
    sd, lidar = make(world)
    decs = run(sd, world, 3.0)  # REF を送って口の準備ができるまで
    assert lidar.ref_t is not None
    lidar.pose = lambda now, odom: (world.pose, "推定器が見失った")
    d = sd.decide(lidar.ref_t + 4.0, np.zeros(29), np.array([1.0, 0.0, 0.0, 0.0]), odom=(0.0, 0.0, 0.0),
                  odom_t=lidar.ref_t + 4.0, obs=None)
    assert d.stop and "見失" in d.stop


S2F_NPZ = Path(__file__).resolve().parents[1] / "logs_real/policy/door_policy_s2f_it79500.npz"
C2_NPZ = Path(__file__).resolve().parents[1] / "logs_real/policy/door_vision_c2.npz"


@pytest.mark.skipif(not (S2F_NPZ.exists() and C2_NPZ.exists()), reason="書き出した s2f・c2 の npz が無い")
@pytest.mark.parametrize("roll, pitch", [(0.0, 0.0), (0.02, -0.04)])
def test_net_sampler_recovers_pose_through_mirror(monkeypatch, roll: float, pitch: float) -> None:
    # 本物の s2f の欄の並びと鏡写しで: 実の向きの扉の値 → 方策の見る側（鏡写し）→ NetSampler が戻して pelvis の姿勢
    import door_core as dc
    import door_vision_core as dvc
    p = dc.load_params(str(S2F_NPZ))
    vp = dvc.load_vision(str(C2_NPZ), p)
    pose = dfr.PlanarPose(-0.95, 0.07, math.radians(4.0))
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    quat = np.array([cr * cp, sr * cp, cr * sp, -sr * sp])
    x_real = np.zeros(p.obs_dim)
    f, g = vp.door_slices["door_frame_loc"]
    x_real[f:g] = dfr.door_frame_loc(pose, quat)
    x_m = p.obs_mirror_M @ x_real + p.obs_mirror_b
    door17_m = dvc.door_from_obs(vp, x_m)
    monkeypatch.setattr(dvc, "door_values", lambda vp_, obs, feat, hg: door17_m)

    class Feats:
        def latest(self):
            return (1, np.zeros(vp.feat_dim), 0.0, 0.0, 0.0)

        def why_not(self, now):
            return ""

    got = drst.NetSampler(p, vp, Feats(), head_pitch=0.9).sample(0.0, np.zeros(29), quat, x_m)
    assert (got.x, got.y, got.yaw) == pytest.approx((pose.x, pose.y, pose.yaw), abs=1e-6)
