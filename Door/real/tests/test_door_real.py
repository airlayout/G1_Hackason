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
    def __init__(self, q_default: np.ndarray, keys: int, pitch_deg: float = 0.0) -> None:
        self.motor_state = [_Motor(float(v)) for v in q_default] + [_Motor(0.0) for _ in range(6)]
        h = math.radians(pitch_deg) / 2  # y 軸まわりに pitch_deg 傾いた姿勢（w, x, y, z）
        self.imu_state = types.SimpleNamespace(quaternion=[math.cos(h), 0.0, math.sin(h), 0.0], gyroscope=[0.0, 0.0, 0.0])
        self.mode_machine = 5
        self.wireless_remote = [0, 0, keys & 0xFF, keys >> 8] + [0] * 36


SIGNALS = {"INT": "SIGINT", "TERM": "SIGTERM", "HUP": "SIGHUP"}


def _run(monkeypatch, tmp_path, script, *, held_at_start=(), freeze_after=None, nan_after=None, run_seconds="0.5",
         pitch_deg=0.0, extra_args=()):
    """偽の機体で door_run_real.main() を回す。script = [(秒, ボタン名 | "L2+B" | "INT" | "TERM" | "HUP")]。

    ボタンはその秒から 0.06 秒押す。INT・TERM・HUP はその秒に 1 回だけシグナルを送る（同じ名前を 2 回書けば 2 回）。
    held_at_start: 起動から 0.3 秒押しっぱなしのボタン／freeze_after: その秒から lowstate の時刻が止まる／
    nan_after: その秒から関節の角度が NaN／pitch_deg: 体の傾き（ずっと同じ）／extra_args: door_run_real.py に足す引数。
    返り値: (段の並び（重複を除く）, 段の列, 送った (q, kp) の列)
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
            for k, (at, name) in enumerate(script):
                if name in SIGNALS:
                    if el >= at and k not in fired:
                        fired.add(k)
                        os.kill(os.getpid(), getattr(_signal, SIGNALS[name]))
                elif at <= el < at + 0.06:
                    for b in name.split("+"):
                        keys |= 1 << drr.KEY[b]
            st = _State(p.q_default, keys, pitch_deg)
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
                                      "--log-dir", str(tmp_path), *extra_args])
    assert drr.main() == 0
    logs = sorted(tmp_path.glob("door_real_*[0-9].npz"))
    assert len(logs) == 1
    with np.load(logs[0]) as z:
        phases = [drr.PHASES[int(v)] for v in z["phase"]]
    seen = [ph for k, ph in enumerate(phases) if k == 0 or ph != phases[k - 1]]
    return seen, phases, sent


# 段の時刻: start 0.4 → RAMP 3 秒 → HOLD 3.4〜 → A 3.7 で STAND → A 5.9 で DOOR


@needs_policy
def test_state_machine_runs_door_for_run_seconds(monkeypatch, tmp_path):
    """start → A → （2 秒後）A → 扉の方策 0.5 秒 → POST → L2+B で脱力 → Ctrl+C で終わる"""
    seen, phases, sent = _run(monkeypatch, tmp_path, [(0.4, "start"), (3.7, "A"), (5.9, "A"), (7.2, "L2+B"), (7.6, "INT")])
    assert seen == ["WAIT", "RAMP", "HOLD", "STAND", "DOOR", "POST", "DAMP"]
    assert 20 <= sum(ph == "DOOR" for ph in phases) <= 30  # 0.5 秒 × 50 Hz
    assert len(sent) > 300
    damp_from = phases.index("DAMP")
    assert all(not kp.any() for _, kp in sent[damp_from + 1:])  # 脱力の後は kp が全部 0


@needs_policy
def test_b_in_door_stands_instead_of_damping(monkeypatch, tmp_path):
    """扉の方策の途中で B → 立ったまま（POST）。kp は 0 にならない。脱力は L2+B だけ"""
    seen, phases, sent = _run(monkeypatch, tmp_path, [(0.4, "start"), (3.7, "A"), (5.9, "A"), (6.1, "B"), (6.8, "L2+B"), (7.2, "INT")],
                              run_seconds="3.0")
    assert seen == ["WAIT", "RAMP", "HOLD", "STAND", "DOOR", "POST", "DAMP"]
    post = [k for k, ph in enumerate(phases) if ph == "POST"]
    assert len(post) > 20 and all(sent[k][1].any() for k in post)  # POST の間は PD で立っている


@needs_policy
def test_b_in_hold_freezes(monkeypatch, tmp_path):
    """HOMIE が動く前（HOLD）に B → 今の姿勢を固く保つ（FREEZE）。脱力しない"""
    seen, phases, sent = _run(monkeypatch, tmp_path, [(0.4, "start"), (3.6, "B"), (4.5, "INT"), (4.8, "INT")])
    assert seen == ["WAIT", "RAMP", "HOLD", "FREEZE", "DAMP"]
    frz = [k for k, ph in enumerate(phases) if ph == "FREEZE"]
    assert len(frz) > 20 and all(sent[k][1].any() for k in frz)
    assert np.abs(sent[frz[-1]][0] - sent[frz[0]][0]).max() < 1e-12  # 同じ目標を保つ


@needs_policy
def test_first_ctrl_c_stands_second_damps(monkeypatch, tmp_path):
    """Ctrl+C の 1 回目は立ったまま（終わらない）、2 回目で脱力して終わる"""
    seen, phases, sent = _run(monkeypatch, tmp_path, [(0.4, "start"), (3.7, "A"), (4.8, "INT"), (5.6, "INT")])
    assert seen == ["WAIT", "RAMP", "HOLD", "STAND", "POST", "DAMP"]
    assert sum(ph == "POST" for ph in phases) > 30  # 1 回目と 2 回目の間は立っている
    assert not sent[-1][1].any()


@needs_policy
def test_sighup_stands_and_keeps_running(monkeypatch, tmp_path):
    """ssh が切れた（SIGHUP）ときは立ったまま止まり、プログラムは終わらない（終わると誰も支えない）"""
    seen, phases, _ = _run(monkeypatch, tmp_path, [(0.4, "start"), (3.7, "A"), (4.8, "HUP"), (6.0, "L2+B"), (6.4, "INT")])
    assert seen == ["WAIT", "RAMP", "HOLD", "STAND", "POST", "DAMP"]
    assert sum(ph == "POST" for ph in phases) > 50  # SIGHUP の後 1.2 秒は立ち続けた


@needs_policy
def test_button_held_at_launch_does_not_start(monkeypatch, tmp_path):
    """起動時に start を押しっぱなしでも RAMP に進まない。離してから押すと進む"""
    seen, _, _ = _run(monkeypatch, tmp_path, [(1.0, "start"), (1.5, "L2+B"), (1.8, "INT")], held_at_start=("start",))
    assert seen[:2] == ["WAIT", "RAMP"]
    seen2, _, _ = _run(monkeypatch, tmp_path / "b", [(0.6, "INT")], held_at_start=("start",))
    assert seen2 == ["WAIT"]


@needs_policy
def test_stale_lowstate_damps(monkeypatch, tmp_path):
    seen, _, _ = _run(monkeypatch, tmp_path, [(0.4, "start"), (5.0, "INT")], freeze_after=1.5)
    assert seen == ["WAIT", "RAMP", "DAMP"]


@needs_policy
def test_nan_joint_damps(monkeypatch, tmp_path):
    seen, _, _ = _run(monkeypatch, tmp_path, [(0.4, "start"), (3.7, "A"), (5.0, "INT")], nan_after=4.0)
    assert seen == ["WAIT", "RAMP", "HOLD", "STAND", "DAMP"]


@needs_policy
def test_tilted_body_refuses_stand(monkeypatch, tmp_path):
    """体が 12° 傾いたまま（吊られて傾いている）では A を押しても HOMIE に任せない（10-16 の実機: 15.6° から立たせて暴れた）"""
    seen, _, _ = _run(monkeypatch, tmp_path, [(0.4, "start"), (3.7, "A"), (4.5, "INT"), (4.8, "INT")], pitch_deg=12.0)
    assert seen == ["WAIT", "RAMP", "HOLD", "FREEZE", "DAMP"]
    seen2, _, _ = _run(monkeypatch, tmp_path / "b", [(0.4, "start"), (3.7, "A"), (4.5, "INT"), (4.8, "INT")], pitch_deg=5.0)
    assert seen2 == ["WAIT", "RAMP", "HOLD", "STAND", "POST", "DAMP"]  # 5° なら立つ


@needs_policy
def test_stand_only_ignores_second_a(monkeypatch, tmp_path):
    """--stand-only では、STAND で A を押しても扉の方策へ進まない"""
    seen, _, _ = _run(monkeypatch, tmp_path, [(0.4, "start"), (3.7, "A"), (5.9, "A"), (6.5, "INT"), (6.8, "INT")],
                      extra_args=("--stand-only",))
    assert seen == ["WAIT", "RAMP", "HOLD", "STAND", "POST", "DAMP"]


@needs_policy
def test_warm_up_runs_both_modes_without_touching_state():
    """起動時の空回しは、脚だけ（hold）と扉（door）の両方を回し、方策の記憶（LSTM の状態など）を持ち越さない"""
    import door_run_real as drr

    p = dc.load_params(POLICY)
    st = _State(p.q_default, 0)
    modes = []
    real_step = dc.step

    def spy(p_, s_, inp_, mode="hold"):
        modes.append(mode)
        return real_step(p_, s_, inp_, mode=mode)

    s0 = dc.initial_state(p)
    before = {k: np.array(v, copy=True) for k, v in vars(s0).items() if isinstance(v, np.ndarray)}
    dc.step = spy
    try:
        ms = drr.warm_up(p, st, dfr.door_geometry(0.94, 2.05, 1.0, 0.064), dfr.PlanarPose(-1.0, 0.0, 0.0))
    finally:
        dc.step = real_step
    assert {"hold", "door"} <= set(modes) and ms >= 0.0
    for k, v in before.items():  # 空回しは自分の状態を使う（呼び出し側の状態を書き換えない）
        np.testing.assert_array_equal(getattr(s0, k), v)
