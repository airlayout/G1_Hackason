"""11-02: 純正の脚の口（door_stock_io.py・door_run_stock.py）の試験（機体なしで動く）。

  DOOR_POLICY=<書き出した npz> python -m pytest G1_Hackason/Door/real/tests/test_door_run_stock.py -q

輪の試験は偽の機体（立った姿勢の lowstate）・偽の arm_sdk・偽の LocoClient で door_run_stock.main() を実時間で回す。
PC の入力は、決まった秒に行を返す偽の標準入力で打つ。
"""
from __future__ import annotations

import os
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_stock_io as dsio  # noqa: E402
from test_door_real import SIGNALS, _State, needs_policy  # noqa: E402  偽の lowstate は HOMIE の経路の試験と同じもの

POLICY = os.environ.get("DOOR_POLICY")
CONSTS = {"homie_obs_lin_scale": 2.0, "homie_obs_ang_scale": 0.5, "homie_cmd_deadzone": 0.1, "homie_switch_threshold": 0.1,
          "action_scale": 0.25, "action_clip": 100.0}


def _p(**kw):
    return types.SimpleNamespace(c=dict(CONSTS, **kw), q_default=np.linspace(-0.3, 0.3, dc.NUM_DOF))


# ---- 部品 ----

def test_stock_velocity_uses_homie_switch_line():
    """歩く／立つは HOMIE と同じ線: [2vx, 2vy, 0.5wz] の大きさが 0.1 を越えたら歩く"""
    p = _p()
    assert dsio.stock_velocity(p, np.array([0.04, 0.0, 0.0, 0.74, 0, 0, 0])) == (False, 0.0, 0.0, 0.0)  # 0.08
    assert dsio.stock_velocity(p, np.array([0.3, -0.1, 0.0, 0.74, 0, 0, 0])) == (True, 0.3, -0.1, 0.0)
    assert dsio.stock_velocity(p, np.array([0.0, 0.0, 0.3, 0.74, 0, 0, 0])) == (True, 0.0, 0.0, 0.3)  # 0.15
    assert dsio.stock_velocity(p, np.array([0.0, 0.0, 0.18, 0.74, 0, 0, 0]))[0] is False  # 0.09


def test_with_leg_actions_modes():
    p = _p()
    s = dc.CoreState(h=np.zeros((2, 256)), cell=np.zeros((2, 256)), D=np.zeros(17), a_real_prev=np.zeros(17),
                     whole_prev=np.arange(dc.NUM_DOF, dtype=float), cmd_prev=np.zeros(7), homie_hist=np.zeros((5, 86)),
                     homie_a_prev=np.ones(15))
    q = p.q_default + 0.05
    assert dsio.with_leg_actions(p, s, q, "shadow") is s
    fq = dsio.with_leg_actions(p, s, q, "from_q")
    np.testing.assert_allclose(fq.whole_prev[:15], 0.2)  # 0.05 ÷ 0.25
    np.testing.assert_allclose(fq.whole_prev[15:], s.whole_prev[15:])  # 腕はそのまま
    np.testing.assert_allclose(fq.homie_a_prev, 1.0)  # HOMIE 自身の履歴は変えない
    z = dsio.with_leg_actions(p, s, q, "zero")
    assert not z.whole_prev[:15].any() and (z.whole_prev[15:] == s.whole_prev[15:]).all()
    assert s.whole_prev[0] == 0.0 and s.whole_prev[1] == 1.0  # 元は書き換えない
    with pytest.raises(ValueError):
        dsio.with_leg_actions(p, s, q, "homie")


def test_move_toward_limits_each_joint():
    got = dsio.move_toward(np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.01, 0.0]), 0.02)
    np.testing.assert_allclose(got, [0.02, 0.01, 0.98])


class FakeLoco:
    """SetVelocity・Damp を時刻つきで覚える。delay_s: 1 回の RPC にかかる時間／code: 返す値／raise_after: その回から例外"""

    def __init__(self, fsm: int = 501, delay_s: float = 0.0, code: int = 0, raise_after=None) -> None:
        self.fsm, self.delay_s, self.code, self.raise_after = fsm, delay_s, code, raise_after
        self.calls = []
        self.lock = threading.Lock()

    def SetVelocity(self, vx, vy, wz, duration):  # noqa: N802
        time.sleep(self.delay_s)
        with self.lock:
            self.calls.append(("vel", time.monotonic(), (vx, vy, wz, duration)))
            n = len(self.calls)
        if self.raise_after is not None and n >= self.raise_after:
            raise RuntimeError("rpc")
        return self.code

    def Damp(self):  # noqa: N802
        with self.lock:
            self.calls.append(("damp", time.monotonic(), ()))

    def GetFsmId(self):  # noqa: N802
        return 0, self.fsm

    def vel(self):
        with self.lock:
            return [c for c in self.calls if c[0] == "vel"]

    def damps(self):
        with self.lock:
            return [c for c in self.calls if c[0] == "damp"]


def test_sender_walks_then_sends_stop_three_times():
    loco = FakeLoco()
    snd = dsio.VelocitySender(loco)
    snd.start()
    snd.set(False, 0.0, 0.0, 0.0)  # 歩いていなければ何も送らない
    time.sleep(0.15)
    assert loco.vel() == []
    snd.set(True, 0.3, 0.0, 0.1)
    time.sleep(0.32)
    walking = loco.vel()
    assert 5 <= len(walking) <= 8  # 20 回/秒
    assert all(c[2] == (0.3, 0.0, 0.1, dsio.VEL_DURATION_S) for c in walking)
    snd.set(False, 0.0, 0.0, 0.0)
    time.sleep(0.4)
    after = loco.vel()[len(walking):]
    assert [c[2] for c in after] == [(0.0, 0.0, 0.0, dsio.VEL_DURATION_S)] * dsio.STOP_REPEATS  # 止める 3 回の後は何も送らない
    snd.close()
    st = snd.status()
    assert st.failed_in_row == 0 and st.last_code == 0 and st.sent == len(loco.vel())


def test_sender_does_not_block_the_loop_on_slow_rpc():
    loco = FakeLoco(delay_s=0.2)
    snd = dsio.VelocitySender(loco)
    snd.start()
    t0 = time.monotonic()
    for k in range(50):
        snd.set(True, 0.01 * k, 0.0, 0.0)
    assert time.monotonic() - t0 < 0.05
    time.sleep(0.5)
    snd.close()
    assert snd.status().last_rpc_ms >= 190.0
    assert loco.vel()[-1][2][0] == 0.0  # close は止める指令を送ってから終わる


def test_sender_counts_failures_and_exceptions():
    loco = FakeLoco(code=3104)
    snd = dsio.VelocitySender(loco)
    snd.start()
    snd.set(True, 0.2, 0.0, 0.0)
    time.sleep(0.3)
    assert snd.status().failed_in_row >= 4 and snd.status().last_code == 3104
    snd.close()
    loco2 = FakeLoco(raise_after=1)
    snd2 = dsio.VelocitySender(loco2)
    snd2.start()
    snd2.set(True, 0.2, 0.0, 0.0)
    time.sleep(0.2)
    st = snd2.status()
    snd2.close()
    assert st.last_code == -1 and "RuntimeError" in st.last_error and st.failed_in_row >= 2


def test_sender_damp_once_then_ignores_walk():
    loco = FakeLoco()
    snd = dsio.VelocitySender(loco)
    snd.start()
    snd.set(True, 0.2, 0.0, 0.0)
    time.sleep(0.12)
    snd.damp()
    time.sleep(0.12)
    n_vel = len(loco.vel())
    snd.set(True, 0.4, 0.0, 0.0)
    time.sleep(0.2)
    snd.close()
    assert len(loco.damps()) == 1 and snd.status().damped
    assert len(loco.vel()) == n_vel  # 脱力の後は速度を送らない


def test_fifo_reader_reopens_after_each_writer(tmp_path):
    """stock_cmd.sh は 1 回ずつ開いて書いて閉じる。読み手は閉じられても開き直して次の指示を受ける"""
    import queue

    import door_run_stock as drs

    path = tmp_path / "stock_cmd.fifo"
    got: "queue.Queue[str]" = queue.Queue()
    threading.Thread(target=drs.watch_fifo, args=(str(path), got), daemon=True).start()
    for _ in range(100):
        if path.exists():
            break
        time.sleep(0.01)
    for c in ("a", "D", ""):
        with open(path, "w", encoding="utf-8") as f:
            f.write(c + "\n")
        time.sleep(0.05)
    assert [got.get(timeout=1.0) for _ in range(3)] == ["a", "d", ""]


# ---- 輪（偽の機体） ----

class FakeArm:
    def __init__(self, sink) -> None:
        self.sink = sink

    def write(self, q_arm, kp_arm, kd_arm, weight) -> None:
        self.sink.append((time.monotonic(), np.array(q_arm), float(weight)))


def _run(monkeypatch, tmp_path, script, *, fsm=501, pitch_deg=0.0, load=lambda el: 1.0, freeze_after=None,
         run_seconds="0.5", extra_args=()):
    """偽の機体で door_run_stock.main() を回す。script = [(秒, "a" | "d" | "r" | "q" | "" | "B" | "L2+B" | "INT" | "HUP")]。

    文字（と空の Enter）は偽の標準入力の行、ボタン名はその秒から 0.06 秒押す、INT・HUP はその秒にシグナル。
    返り値: (戻り値, 段の並び（重複を除く）, 段の列, 記録, 腕の送信 [(時刻, q, 重み)], 偽の LocoClient, 起動の時刻)
    """
    import signal as _signal

    import door_run_real as drr
    import door_run_stock as drs

    p = dc.load_params(POLICY)
    t0 = [time.monotonic()]
    fired = set()
    arms = []
    loco = FakeLoco(fsm=fsm)

    class FakeIO:
        def __init__(self, iface, send, use_odom):
            assert send is False  # rt/lowcmd は送らない
            self.odom = (0.0, 0.0, 0.0)

        def snapshot(self):
            now = time.monotonic()
            el = now - t0[0]
            keys = 0
            for k, (at, name) in enumerate(script):
                if name in SIGNALS:
                    if el >= at and k not in fired:
                        fired.add(k)
                        os.kill(os.getpid(), getattr(_signal, SIGNALS[name]))
                elif name and name[0].isupper() and at <= el < at + 0.06:
                    for b in name.split("+"):
                        keys |= 1 << drr.KEY[b]
            st = _State(p.q_default, keys, pitch_deg, load(el))
            stamp = now if freeze_after is None or el < freeze_after else t0[0] + freeze_after
            return st, stamp, self.odom, now

    class FakeStdin:
        def __init__(self) -> None:
            self.rows = [(at, name) for at, name in script if not (name and name[0].isupper())]

        def readline(self):
            if not self.rows:
                time.sleep(30.0)  # 打つものが無くなったら黙る（EOF にすると糸が終わるだけなので、どちらでもよい）
                return ""
            at, name = self.rows.pop(0)
            time.sleep(max(0.0, t0[0] + at - time.monotonic()))
            return name + "\n"

    monkeypatch.setattr(drr, "RobotIO", FakeIO)
    monkeypatch.setattr(drr, "check_29dof", lambda st: None)
    monkeypatch.setattr(drr, "motion_mode_name", lambda: "ai")
    monkeypatch.setattr(dsio, "ArmSdkWriter", lambda mode_machine: FakeArm(arms))
    monkeypatch.setattr(dsio, "make_loco_client", lambda timeout: loco)
    monkeypatch.setattr(sys, "stdin", FakeStdin())
    t0[0] = time.monotonic()
    rc = drs.main(["--policy", POLICY, "--door-source", "tape", "--start-x=-1.0", "--start-y=0.0", "--door", "0.94,2.05,0.99,0.064",
                   "--run-seconds", run_seconds, "--log-dir", str(tmp_path), *extra_args])
    logs = sorted(tmp_path.glob("door_stock_*[0-9].npz"))
    if rc != 0:
        return rc, [], [], None, arms, loco, t0[0]
    assert len(logs) == 1
    z = dict(np.load(logs[0]))
    phases = [drs.PHASES[int(v)] for v in z["phase"]]
    seen = [ph for k, ph in enumerate(phases) if k == 0 or ph != phases[k - 1]]
    return rc, seen, phases, z, arms, loco, t0[0]


def drs_step() -> float:
    import door_run_stock as drs

    return drs.DT / drs.WEIGHT_RAMP_S


def _phase_time(z, phases, name):
    k = phases.index(name)
    return float(z["t"][k])


# 段の時刻: a 0.3 → ARMS_IN（重み 2 秒。偽の機体の腕は既に学習の姿勢）→ READY ≈2.4 → 足の確かめ 1 秒 → d 3.8 → DOOR 0.5 秒

@needs_policy
def test_full_flow_borrows_and_returns_arms(monkeypatch, tmp_path):
    """a → 腕を借りる → d → 扉の方策 0.5 秒 → POST → r → 腕を戻して重み 0 → WAIT → q で終わる。脱力は一度も送らない"""
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (4.8, "r"), (8.2, "q")])
    assert rc == 0
    assert seen == ["WAIT", "ARMS_IN", "READY", "DOOR", "POST", "ARMS_OUT", "WAIT"]
    assert 20 <= sum(ph == "DOOR" for ph in phases) <= 30
    w = np.array([a[2] for a in arms])
    assert w.max() == 1.0 and w[-1] == 0.0 and (np.abs(np.diff(w)) <= drs_step() + 1e-9).all()  # 重みは 1 周期 0.01 ずつ
    first_full = int(np.argmax(w >= 1.0))
    assert (np.diff(w[:first_full + 1]) >= 0).all()  # 借りる間は重みが上がるだけ
    assert all(np.isfinite(a[1]).all() for a in arms)
    assert loco.damps() == []
    t_post = _phase_time(z, phases, "POST")
    late = [c for c in loco.vel() if c[1] > t_post + 0.3]
    assert all(c[2][:3] == (0.0, 0.0, 0.0) for c in late)  # 止めた後に歩く指令は出ない
    door = [k for k, ph in enumerate(phases) if ph == "DOOR"]
    assert np.isfinite(z["obs"][door]).all() and np.isfinite(z["cmd"][door]).all()


@needs_policy
def test_enter_in_door_stands_and_keeps_arms(monkeypatch, tmp_path):
    """扉の方策の途中で Enter → 立ったまま（POST）。腕の重みは 1 のまま・脱力しない"""
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (4.0, ""), (5.0, "INT"), (5.4, "INT")],
                                              run_seconds="3.0")
    assert seen[:5] == ["WAIT", "ARMS_IN", "READY", "DOOR", "POST"]
    post = [k for k, ph in enumerate(phases) if ph == "POST"]
    assert len(post) > 30 and (z["arm_weight"][post] == 1.0).all()


@needs_policy
def test_second_ctrl_c_damps_once(monkeypatch, tmp_path):
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (4.0, "INT"), (4.6, "INT")],
                                              run_seconds="3.0")
    assert rc == 0 and seen[-2:] == ["POST", "DAMP"]
    assert len(loco.damps()) == 1
    t_damp = _phase_time(z, phases, "DAMP")
    after = [a for a in arms if a[0] >= t_damp]
    assert after and all(a[2] == 0.0 for a in after)  # 腕も純正へ返して Damp を効かせる


@needs_policy
def test_sighup_stands_and_keeps_running(monkeypatch, tmp_path):
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (2.8, "HUP"), (4.5, "INT"), (4.9, "INT")])
    assert seen[:4] == ["WAIT", "ARMS_IN", "READY", "POST"]
    assert sum(ph == "POST" for ph in phases) > 60  # 1.6 秒以上、立ったまま回り続けた
    assert len(loco.damps()) == 1  # 最後の Ctrl+C の 2 回目だけ


@needs_policy
def test_stale_lowstate_stands_instead_of_damping(monkeypatch, tmp_path):
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (5.5, "INT"), (5.9, "INT")],
                                              freeze_after=4.0, run_seconds="3.0")
    assert seen[:5] == ["WAIT", "ARMS_IN", "READY", "DOOR", "POST"]
    t_post = _phase_time(z, phases, "POST")
    assert 4.0 < t_post - t0 < 4.4
    assert all(d[1] > t_post + 1.0 for d in loco.damps())  # 脱力は最後の Ctrl+C だけ


@needs_policy
def test_tilt_stops_then_damps_when_fallen(monkeypatch, tmp_path):
    _, seen, _, _, _, loco, _ = _run(monkeypatch, tmp_path, [(0.3, "a"), (1.5, "INT"), (1.9, "INT")], pitch_deg=12.0)
    assert seen[:3] == ["WAIT", "ARMS_IN", "POST"]
    _, seen2, _, _, _, loco2, _ = _run(monkeypatch, tmp_path / "b", [(0.3, "a"), (1.5, "q"), (1.9, "INT")], pitch_deg=35.0)
    assert seen2[:3] == ["WAIT", "ARMS_IN", "DAMP"] and len(loco2.damps()) == 1


@needs_policy
def test_refuses_to_start_outside_walking_fsm(monkeypatch, tmp_path):
    rc, _, _, _, arms, loco, _ = _run(monkeypatch, tmp_path, [(0.3, "a")], fsm=802)
    assert rc == 1 and arms == [] and loco.calls == []


@needs_policy
def test_ready_refuses_door_while_rope_holds_weight(monkeypatch, tmp_path):
    """綱が体重の半分を受けている（両足で 50%）と d を受け付けない。L2+B で脱力して終わる"""
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (4.4, "L2+B"), (4.8, "q")],
                                              load=lambda el: 0.5)
    assert seen == ["WAIT", "ARMS_IN", "READY", "DAMP"]


@needs_policy
def test_zero_leg_actions_reach_student_obs(monkeypatch, tmp_path):
    """--leg-actions zero: 生徒の観測の actions の脚と腰（先頭 15）は DOOR の 2 周期目から 0（鏡写しでも 0 は 0）"""
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (4.8, "INT"), (5.2, "INT")],
                                              extra_args=("--leg-actions", "zero"))
    p = dc.load_params(POLICY)
    a0, a1 = p.layout["actions"]
    door = [k for k, ph in enumerate(phases) if ph == "DOOR"]
    assert not z["obs"][door[1:], a0:a0 + 15].any()
    rc2, _, phases2, z2, _, _, _ = _run(monkeypatch, tmp_path / "b", [(0.3, "a"), (3.8, "d"), (4.8, "INT"), (5.2, "INT")])
    door2 = [k for k, ph in enumerate(phases2) if ph == "DOOR"]
    assert np.abs(z2["obs"][door2[1:], a0:a0 + 15]).max() > 1e-3  # 既定（影の HOMIE）は 0 でない


def test_parse_probe():
    import door_run_stock as drs

    assert drs.parse_probe("p", 0.3) == 0.3
    assert drs.parse_probe("p0.5", 0.3) == 0.5
    assert drs.parse_probe("p 0.4", 0.3) == 0.4
    assert drs.parse_probe("p2", 0.3) is None  # 1 m/s を越える指示は押す試しにしない（止める扱い）
    assert drs.parse_probe("px", 0.3) is None and drs.parse_probe("a", 0.3) is None and drs.parse_probe("", 0.3) is None


@needs_policy
def test_probe_pushes_forward_with_arms_held(monkeypatch, tmp_path):
    """POST で p0.5 → 腕は止めた時の目標のまま、前へ 0.4（--max-vx で頭打ち）を 0.5 秒 → POST へ戻る"""
    rc, seen, phases, z, arms, loco, t0 = _run(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (5.0, "p0.5"), (6.4, "INT"), (6.8, "INT")],
                                              extra_args=("--probe-seconds", "0.5", "--max-vx", "0.4"))
    assert seen[:7] == ["WAIT", "ARMS_IN", "READY", "DOOR", "POST", "PROBE", "POST"]
    probe = [k for k, ph in enumerate(phases) if ph == "PROBE"]
    assert 20 <= len(probe) <= 30
    held = z["arm_sent"][probe[0] - 1]
    np.testing.assert_array_equal(z["arm_sent"][probe], np.tile(held, (len(probe), 1)))
    t_a, t_b = float(z["t"][probe[0]]), float(z["t"][probe[-1]])
    pushes = [c for c in loco.vel() if t_a + 0.1 < c[1] < t_b]
    assert pushes and all(c[2] == (0.4, 0.0, 0.0, dsio.VEL_DURATION_S) for c in pushes)
    late = [c for c in loco.vel() if c[1] > t_b + 0.3]
    assert all(c[2][:3] == (0.0, 0.0, 0.0) for c in late)
    import analyze_stock

    r = analyze_stock.summarize(z, dc.load_params(POLICY))  # 輪が書いた記録をそのまま読める
    assert len(r["押す試し"]) == 1 and r["押す試し"][0]["前の指令 m/s"] == pytest.approx(0.4)
    assert "dof_pos（脚と腰）" in r["観測"] and r["段"]["DOOR"]["秒"] > 0.4
    analyze_stock.print_report(r)
