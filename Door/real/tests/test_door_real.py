"""段 5 の実機用コードの試験（機体なしで動く）。

  python -m pytest G1_Hackason/Door/real/tests -q

正解の記録（golden_*.npz）と書き出した重み（door_policy_*.npz）は大きいので repo に置かない。環境変数
DOOR_GOLDEN・DOOR_POLICY で場所を渡すと、それを使う試験も走る（無ければ飛ばす）。
"""
from __future__ import annotations

import math
import os
import sys
import types
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import door_core as dc  # noqa: E402
import door_frame as dfr  # noqa: E402

GOLDEN = os.environ.get("DOOR_GOLDEN")
POLICY = os.environ.get("DOOR_POLICY")
needs_golden = pytest.mark.skipif(not (GOLDEN and Path(GOLDEN).exists()), reason="DOOR_GOLDEN が無い")
needs_policy = pytest.mark.skipif(not (POLICY and Path(POLICY).exists()), reason="DOOR_POLICY が無い")


def _golden():
    with np.load(GOLDEN) as z:
        return {k: z[k] for k in z.files}


@needs_golden
def test_gravity_from_imu_quaternion_matches_simulator():
    """実機の IMU は w, x, y, z の順。シミュレータの姿勢（x, y, z, w）を並べ替えて入れると projected_gravity に戻る"""
    g = _golden()
    q_xyzw = g["base_quat_xyzw"].reshape(-1, 4).astype(float)
    want = g["projected_gravity"].reshape(-1, 3).astype(float)
    got = np.array([dc.quat_wxyz_to_gravity(np.array([w, x, y, z])) for x, y, z, w in q_xyzw])
    assert np.abs(got - want).max() < 1e-5


@needs_golden
def test_door_frame_up_axis_is_minus_gravity():
    g = _golden()
    for x, y, z, w in g["base_quat_xyzw"][::50].reshape(-1, 4).astype(float):
        q = np.array([w, x, y, z])
        loc = dfr.door_frame_loc(dfr.PlanarPose(-1.0, 0.2, dfr.yaw_from_quat_wxyz(q)), q)
        assert np.abs(loc[6:9] + dc.quat_wxyz_to_gravity(q)).max() < 1e-6  # float32 の四元数は長さがちょうど 1 でない


def test_door_frame_loc_facing_door():
    """扉の 1 m 手前・正面を向いて直立: 開口の中心は前 1 m、扉の +x は前、+z は上"""
    loc = dfr.door_frame_loc(dfr.PlanarPose(-1.0, 0.0, 0.0), np.array([1.0, 0.0, 0.0, 0.0]))
    np.testing.assert_allclose(loc, [1, 0, 0, 1, 0, 0, 0, 0, 1], atol=1e-12)


def test_door_frame_loc_turned_left():
    """左に 90° 向いている（扉の +x が機体の右）: 開口の中心は機体の右、tan は −y"""
    loc = dfr.door_frame_loc(dfr.PlanarPose(-1.0, 0.0, math.pi / 2), np.array([math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)]))
    np.testing.assert_allclose(loc[:6], [0, -1, 0, 0, -1, 0], atol=1e-12)


def test_compose_planar_moves_in_start_heading():
    """odom の向きが扉の座標と 90° ずれていても、機体の前へ 0.5 m 進めば扉の座標でも前へ 0.5 m"""
    start = dfr.PlanarPose(-1.0, 0.1, 0.0)
    odom0 = dfr.PlanarPose(3.0, 4.0, math.pi / 2)
    odom1 = dfr.PlanarPose(3.0, 4.5, math.pi / 2 + 0.1)
    now = dfr.compose_planar(start, odom0, odom1)
    assert now.x == pytest.approx(-0.5) and now.y == pytest.approx(0.1) and now.yaw == pytest.approx(0.1)


def test_door_geometry_lever_left():
    np.testing.assert_allclose(dfr.door_geometry(0.94, 2.05, 1.0, 0.064), [0.94, 2.05, 1.0, 0.064, 1, 0, 0])


def test_remote_keys():
    import door_run_real as drr

    wr = [0] * 40
    wr[3] = 1 << 0  # A
    assert drr.pressed(drr.parse_keys(wr), "A") and not drr.pressed(drr.parse_keys(wr), "B")
    wr[2], wr[3] = 1 << 2, 1 << 1  # start, B
    k = drr.parse_keys(wr)
    assert drr.pressed(k, "start") and drr.pressed(k, "B") and not drr.pressed(k, "A")


# ---- 状態の進み方（機体の代わりに、立った姿勢の lowstate を返し、指令を貯めるだけの入れ物） ----

class _Motor:
    def __init__(self, q: float) -> None:
        self.q, self.dq, self.tau_est, self.vol, self.temperature = q, 0.0, 0.0, 30.0, [30, 30]


class _State:
    def __init__(self, q_default: np.ndarray, keys: int) -> None:
        self.motor_state = [_Motor(float(v)) for v in q_default] + [_Motor(0.0) for _ in range(6)]
        self.imu_state = types.SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.0], gyroscope=[0.0, 0.0, 0.0])
        self.mode_machine = 5
        self.wireless_remote = [0, 0, keys & 0xFF, keys >> 8] + [0] * 36


def _run(monkeypatch, tmp_path, script, *, held_at_start=(), freeze_after=None, nan_after=None, run_seconds="0.5"):
    """偽の機体で door_run_real.main() を回す。script = [(秒, ボタン名 | "PC" | "HUP")]。

    held_at_start: 起動から 0.3 秒押しっぱなしのボタン／freeze_after: その秒から lowstate の時刻が止まる／
    nan_after: その秒から関節の角度が NaN。返り値: (段の並び（重複を除く）, 段の列, 送った (q, kp) の列)
    """
    import signal as _signal
    import time as _time

    import door_run_real as drr

    p = dc.load_params(POLICY)
    t0 = [None]
    sent = []
    fired = set()

    class FakeIO:
        def __init__(self, iface, send, use_odom):
            self.odom = (0.0, 0.0, 0.0)

        def snapshot(self):
            now = _time.monotonic()
            if t0[0] is None:
                t0[0] = now
            el = now - t0[0]
            keys = 0
            for name in held_at_start:
                if el < 0.3:
                    keys |= 1 << drr.KEY[name]
            for at, name in script:
                if name in ("PC", "HUP"):
                    if el >= at and name not in fired:
                        fired.add(name)
                        os.kill(os.getpid(), _signal.SIGTERM if name == "PC" else _signal.SIGHUP)
                elif at <= el < at + 0.06:
                    keys |= 1 << drr.KEY[name]
            st = _State(p.q_default, keys)
            if nan_after is not None and el >= nan_after:
                st.motor_state[3].q = float("nan")
            stamp = now if freeze_after is None or el < freeze_after else t0[0] + freeze_after
            return st, stamp, self.odom, now

        def write(self, state, q, kp, kd):
            sent.append((np.array(q), np.array(kp)))

    monkeypatch.setattr(drr, "RobotIO", FakeIO)
    monkeypatch.setattr(drr, "check_29dof", lambda st: None)
    monkeypatch.setattr(drr, "motion_mode_name", lambda: "")
    monkeypatch.setattr(sys, "stdin", open(os.devnull))
    monkeypatch.setattr(sys, "argv", ["door_run_real.py", "--policy", POLICY, "--start-x=-1.0", "--start-y=0.0",
                                      "--start-yaw-deg=0", "--door", "0.94,2.05,1.0,0.064", "--run-seconds", run_seconds,
                                      "--log-dir", str(tmp_path)])
    assert drr.main() == 0
    logs = sorted(tmp_path.glob("door_real_*[0-9].npz"))
    assert len(logs) == 1
    with np.load(logs[0]) as z:
        phases = [drr.PHASES[int(v)] for v in z["phase"]]
    seen = [ph for k, ph in enumerate(phases) if k == 0 or ph != phases[k - 1]]
    return seen, phases, sent


@needs_policy
def test_state_machine_runs_door_for_run_seconds(monkeypatch, tmp_path):
    """start → A → （2 秒後）A → 扉の方策 0.5 秒 → POST → B で脱力。例外なく進み、記録が残る"""
    seen, phases, sent = _run(monkeypatch, tmp_path, [(0.4, "start"), (3.7, "A"), (5.9, "A"), (7.2, "B"), (7.6, "PC")])
    assert seen == ["WAIT", "RAMP", "HOLD", "STAND", "DOOR", "POST", "DAMP"]
    assert 20 <= sum(ph == "DOOR" for ph in phases) <= 30  # 0.5 秒 × 50 Hz
    assert len(sent) > 300
    # 脱力の後は kp が全部 0
    damp_from = phases.index("DAMP")
    assert all(not kp.any() for _, kp in sent[damp_from + 1:])


@needs_policy
def test_button_held_at_launch_does_not_start(monkeypatch, tmp_path):
    """起動時に start を押しっぱなしでも RAMP に進まない。離してから押すと進む"""
    seen, _, _ = _run(monkeypatch, tmp_path, [(1.0, "start"), (1.5, "PC")], held_at_start=("start",))
    assert seen[:2] == ["WAIT", "RAMP"]
    seen2, phases2, _ = _run(monkeypatch, tmp_path / "b", [(0.6, "PC")], held_at_start=("start",))
    assert seen2 == ["WAIT", "DAMP"]


@needs_policy
def test_stale_lowstate_damps(monkeypatch, tmp_path):
    seen, _, _ = _run(monkeypatch, tmp_path, [(0.4, "start"), (5.0, "PC")], freeze_after=1.5)
    assert seen == ["WAIT", "RAMP", "DAMP"]


@needs_policy
def test_nan_joint_damps(monkeypatch, tmp_path):
    seen, _, _ = _run(monkeypatch, tmp_path, [(0.4, "start"), (3.7, "A"), (5.0, "PC")], nan_after=4.0)
    assert seen == ["WAIT", "RAMP", "HOLD", "STAND", "DAMP"]


@needs_policy
def test_sighup_damps_and_exits(monkeypatch, tmp_path):
    """ssh が切れた（SIGHUP）ときも脱力を送ってから終わる"""
    seen, _, sent = _run(monkeypatch, tmp_path, [(0.4, "start"), (1.0, "HUP")])
    assert seen == ["WAIT", "RAMP", "DAMP"]
    assert not sent[-1][1].any()
