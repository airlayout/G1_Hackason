"""11-10 手順 L4: --staged を付けた door_run_stock.main() を、偽の機体・偽の FAST-LIO の受け口・偽の網で実時間で回す（約 1 分）。

  DOOR_POLICY=<書き出した npz> python -m pytest G1_Hackason/Door/real/tests/test_door_staged_loop.py -q

偽の FAST-LIO の受け口は、純正へ最後に送った速度の指令（偽の LocoClient が覚える）で姿勢を動かす（純正の鈍さと効かない帯を真似る）。
方策は s2f・f3t3 の代わりに同じ npz を 2 回読む（形が同じなので輪の確かめには足りる）。
"""
from __future__ import annotations

import math
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_frame as dfr  # noqa: E402
import door_staged as ds  # noqa: E402
import door_stock_io as dsio  # noqa: E402
from test_door_real import _State, needs_policy  # noqa: E402
from test_door_run_stock import POLICY, FakeArm, FakeLoco  # noqa: E402

GAIN, DEAD = (0.52, 0.36, 0.61), (0.15, 0.15, 0.2)


class FakeLidarSource:
    """REF の 0.3 秒後から姿勢を返す。姿勢は最後に送った指令で動く"""

    def __init__(self, loco: FakeLoco) -> None:
        self.loco, self.head_xy, self.refs = loco, (0.0, 0.0), []
        self.ref_t = self.t_last = None
        self.true = dfr.PlanarPose(-1.0, 0.05, 0.0)
        self.lock = threading.Lock()

    def request_ref(self, lidar_pose=None) -> None:
        self.refs.append(lidar_pose)
        self.ref_t = time.monotonic()

    def note_state(self, now, odom, q_waist, roll, pitch) -> None:
        with self.lock:
            if self.t_last is not None:
                vel = self.loco.vel()
                vx, vy, wz = (vel[-1][2][:3] if vel else (0.0, 0.0, 0.0))
                v = [c * g if abs(c) >= d else 0.0 for c, g, d in zip((vx, vy, wz), GAIN, DEAD)]
                dt = now - self.t_last
                p = self.true
                c, s = math.cos(p.yaw), math.sin(p.yaw)
                self.true = dfr.PlanarPose(p.x + (c * v[0] - s * v[1]) * dt, p.y + (s * v[0] + c * v[1]) * dt, p.yaw + v[2] * dt)
            self.t_last = now

    def ready(self, now) -> bool:
        return self.ref_t is not None and now - self.ref_t >= 0.3

    def pose(self, now, odom):
        return (self.true if self.ready(now) else None), ""

    def snapshot(self):
        return None, 0.0


class FakeFeats:
    def __init__(self, *a, **k) -> None:
        pass

    def why_not(self, now) -> str:
        return ""

    def latest(self):
        return None

    def close(self) -> None:
        pass


def _run_staged(monkeypatch, tmp_path, script, extra=()):
    """偽の機体で door_run_stock.main(--staged) を回す。返り値: (記録, 偽の FAST-LIO の受け口, 段の列, 段の並び)"""
    import door_lidar_source
    import door_run_real as drr
    import door_run_staged as drst
    import door_run_stock as drs
    import door_vision_core as dvc
    import door_vision_source as dvs

    p = dc.load_params(POLICY)
    loco = FakeLoco()
    lidar = FakeLidarSource(loco)
    arms = []
    t0 = [time.monotonic()]

    class FakeIO:
        def __init__(self, iface, send, use_odom):
            self.odom = (0.0, 0.0, 0.0)

        def snapshot(self):
            now = time.monotonic()
            return _State(p.q_default, 0, 0.0, 1.0), now, self.odom, now

    class FakeStdin:
        def __init__(self) -> None:
            self.rows = list(script)

        def readline(self):
            if not self.rows:
                time.sleep(30.0)
                return ""
            at, name = self.rows.pop(0)
            time.sleep(max(0.0, t0[0] + at - time.monotonic()))
            return name + "\n"

    class FakeNet:
        def __init__(self, p_, vp, feats, head_pitch) -> None:
            self.head_pitch, self.next_t = head_pitch, 0.0

        def sample(self, now, q, quat, obs):
            if obs is None or now < self.next_t or not lidar.ready(now) and lidar.ref_t is not None:
                return None
            self.next_t = now + 0.1
            return lidar.true

    monkeypatch.setattr(drr, "RobotIO", FakeIO)
    monkeypatch.setattr(drr, "check_29dof", lambda st: None)
    monkeypatch.setattr(drr, "motion_mode_name", lambda: "ai")
    monkeypatch.setattr(dsio, "ArmSdkWriter", lambda mode_machine: FakeArm(arms))
    monkeypatch.setattr(dsio, "make_loco_client", lambda timeout: loco)
    monkeypatch.setattr(door_lidar_source, "LidarDoorSource", lambda port, mode="track": lidar)
    monkeypatch.setattr(dvc, "load_vision", lambda path, p_: types.SimpleNamespace(feat_dim=128))
    monkeypatch.setattr(dvc, "camera_pitch_deg", lambda vp: 50.0)
    monkeypatch.setattr(dvc, "head_pitch_out_of_range", lambda vp, pitch: None)
    monkeypatch.setattr(dvs, "VisionFeatSource", FakeFeats)
    monkeypatch.setattr(drst, "NetSampler", FakeNet)
    monkeypatch.setattr(sys, "stdin", FakeStdin())
    t0[0] = time.monotonic()
    rc = drs.main(["--policy", POLICY, "--policy-f3t3", POLICY, "--net", "c2.npz", "--staged", "--door-source", "lidar",
                   "--door", "0.94,2.05,0.99,0.064", "--run-seconds", "45", "--log-dir", str(tmp_path), *extra])
    assert rc == 0
    z = dict(np.load(sorted(tmp_path.glob("door_stock_*[0-9].npz"))[0]))
    st = z["staged"]
    stages = st[np.isfinite(st[:, 0]), 0].astype(int)
    seen = [int(v) for k, v in enumerate(stages) if k == 0 or v != stages[k - 1]]
    return z, lidar, st, seen


@needs_policy
def test_staged_loop_goes_through_stage0_approach_and_into_stage2(monkeypatch, tmp_path):
    import door_run_stock as drs
    z, lidar, st, seen = _run_staged(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (52.0, "r"), (56.0, "q")])
    # DOOR の周期はすべて段を通る（d を受けた周期も・11-10 の見直し）。段 0 の初め（測る 1.5 秒）は歩かない
    door = np.array([drs.PHASES[int(v)] == "DOOR" for v in z["phase"]])
    assert np.isfinite(st[door, 0]).all()
    t_door = z["t"][door][0]
    early = door & (z["t"] < t_door + 1.0)
    assert not z["vel_want"][early][:, 0].any()
    # 段 0 → 1 → 2 と進み、REF は 2 回（ライダーの姿勢つき）
    assert seen[:3] == [0, 1, 2], seen
    assert len(lidar.refs) == 2 and all(r is not None for r in lidar.refs)
    # 構えの位置に入っている（段 2 に入った時の本当の姿勢）
    assert ds.stance(0.94, 0.064, True).contains(lidar.true), lidar.true
    # 段 2 の脚を放す前（積み上げを止めた周期）は歩かない
    in2 = np.isfinite(st[:, 0]) & (st[:, 0] == 2) & (st[:, 3] == 1)
    assert in2.any()
    want = z["vel_want"][in2]
    assert not want[:, 0].any() and np.allclose(want[:, 1:], 0.0)
    # 段 2 の方策の周期は歩きの積み上げが 0
    assert np.allclose(z["D"][in2][:, :3], 0.0)


@needs_policy
def test_staged_stop_stage_stands_when_stage1_begins(monkeypatch, tmp_path):
    # --staged-stop-stage 1: 段 0 が終わって段 1 に入った周期に立ったまま止まる（段 1 の歩きは出さない）
    import door_run_stock as drs
    z, lidar, st, seen = _run_staged(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (16.0, "r"), (20.0, "q")],
                                     extra=("--staged-stop-stage", "1"))
    phases = [drs.PHASES[int(v)] for v in z["phase"]]
    assert seen[:2] == [0, 1] and max(seen) == 1
    k_post = next(i for i, ph in enumerate(phases) if ph == "POST")
    assert phases[k_post - 1] == "DOOR" and st[k_post, 0] == 1  # 段 1 に入った周期に POST へ落ちる
    assert not z["vel_want"][k_post:, 0].any()
    assert len(lidar.refs) == 2
