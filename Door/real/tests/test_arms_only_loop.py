"""腕だけの空振り（--legs hold）の通しの試験（2026-10-23。実機の前に「脚に方策・HOMIE の指令が行かない」ことを確かめる）。

door_run_real.py の本物の制御の輪（main）を、作り物の機体（RobotIO の代わり）につないで実時間で回す。作り物のリモコンで
start → A → L2+B と押し、機体へ送った指令を全部記録して確かめる:
  1. STAND（HOMIE が脚を動かす段）に一度も入らない
  2. HOLD・DOOR・POST の間、脚と腰（SDK 0〜14）の目標はいつも決まった姿勢（q_default）。HOMIE の出力と違う
  3. 脚の固さは HOLD と同じ（p.kp）
  4. 腕（15〜28）は DOOR の間に方策に従って動く
重み（door_policy_it45150.npz）が要る。DOOR_POLICY で場所を渡す（無ければ飛ばす）。1 回 15 秒ほどかかる。
"""
from __future__ import annotations

import os
import signal
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
POLICY = os.environ.get("DOOR_POLICY")
needs_policy = pytest.mark.skipif(not (POLICY and Path(POLICY).exists()), reason="DOOR_POLICY が無い")


class _Motor:
    def __init__(self, q: float) -> None:
        self.q, self.dq, self.tau_est, self.temperature, self.vol, self.motorstate, self.mode = q, 0.0, 0.0, [30, 30], 24.0, 0, 1


class FakeRobot:
    """rt/lowstate を返し、rt/lowcmd を記録する。関節は決まった姿勢から動かない（作り物なので指令に従わない）"""

    def __init__(self, q0: np.ndarray, script) -> None:
        self.t0 = time.monotonic()
        self.q0 = q0
        self.script = script  # [(始め秒, 終わり秒, ボタンの 16 ビット)]
        self.sent = []  # (時刻, q, kp, kd)

    def _state(self):
        t = time.monotonic() - self.t0
        keys = 0
        for a, b, k in self.script:
            if a <= t < b:
                keys |= k
        wr = [0] * 40
        wr[2], wr[3] = keys & 0xFF, (keys >> 8) & 0xFF
        st = types.SimpleNamespace(
            motor_state=[_Motor(float(self.q0[j]) if j < 29 else 0.0) for j in range(35)],
            imu_state=types.SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.0], gyroscope=[0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0]),
            wireless_remote=wr, mode_machine=5)
        return st

    def snapshot(self):
        return self._state(), time.monotonic(), None, None

    def write(self, state, q, kp, kd) -> None:
        self.sent.append((time.monotonic() - self.t0, np.array(q, float), np.array(kp, float), np.array(kd, float)))


@needs_policy
def test_arms_only_never_sends_homie_or_policy_to_legs(tmp_path, monkeypatch):
    import door_core as dc
    import door_run_real as R

    p = dc.load_params(POLICY, mirror=True)
    bit = lambda name: 1 << R.KEY[name]  # noqa: E731
    script = [(0.6, 0.8, bit("start")), (4.6, 4.8, bit("A")), (11.0, 11.2, bit("L2") | bit("B"))]
    robot = FakeRobot(p.q_default.copy(), script)
    monkeypatch.setattr(R, "RobotIO", lambda iface, send, use_odom: robot)
    monkeypatch.setattr(R, "motion_mode_name", lambda: "")
    monkeypatch.setattr(sys, "argv", ["door_run_real.py", "--policy", POLICY, "--door-source", "tape", "--start-x=-0.38",
                                      "--start-y=0.33", "--start-yaw-deg=2", "--odom", "none", "--door", "0.95,2.18,1.00,0.05",
                                      "--run-seconds", "5", "--legs", "hold", "--log-dir", str(tmp_path)])
    threading.Timer(12.5, lambda: os.kill(os.getpid(), signal.SIGHUP)).start()  # DAMP から PC で終わらせる
    old = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        assert R.main() == 0
    finally:
        for s, h in old.items():
            signal.signal(s, h)

    logs = sorted(p_ for p_ in tmp_path.glob("door_real_*.npz") if "damp" not in p_.name)
    z = np.load(logs[-1])
    ph = z["phase"].astype(int)
    names = R.PHASES
    seen = {names[i] for i in np.unique(ph)}
    # 1. STAND に一度も入らない・DOOR と POST は通った
    assert "STAND" not in seen
    assert {"HOLD", "DOOR", "POST", "DAMP"} <= seen
    legs = slice(0, dc.NUM_HOMIE)
    for name in ("HOLD", "DOOR", "POST"):
        m = ph == names.index(name)
        q_sent = z["q_sent"][m]
        # 2. 脚と腰の目標はいつも決まった姿勢（可動域の端から 0.05 rad 内側で切った値）
        want = np.clip(p.q_default[legs], p.q_lower[legs] + 0.05, p.q_upper[legs] - 0.05)
        assert np.abs(q_sent[:, legs] - want).max() < 1e-9, name
        # 3. 脚の固さは HOLD と同じ
        assert np.abs(z["kp_sent"][m][:, legs] - p.kp[legs]).max() < 1e-9, name
    door = ph == names.index("DOOR")
    # HOMIE の出力は計算されているが、脚へは送っていない（送っていれば目標が決まった姿勢から外れる）
    homie_targets = p.c["action_scale"] * z["homie_a"][door] + p.q_default[legs]
    assert np.abs(homie_targets - z["q_sent"][door][:, legs]).max() > 1e-3
    print(f"\n[腕だけ] DOOR {door.sum()} 周期: HOMIE が出した脚の目標は決まった姿勢から最大 "
          f"{np.abs(homie_targets - want).max():.3f} rad 離れていた／送った脚の指令は最大 "
          f"{np.abs(z['q_sent'][door][:, legs] - want).max():.1e} rad／腕の指令の動いた幅の最大 "
          f"{np.ptp(z['q_sent'][door][:, dc.NUM_HOMIE:], axis=0).max():.3f} rad")
    # 4. 腕は DOOR の間に方策に従って動く
    arms = z["q_sent"][door][:, dc.NUM_HOMIE:]
    assert np.ptp(arms, axis=0).max() > 0.05
    # 作り物の機体へ実際に送った指令でも、DOOR の間の脚は決まった姿勢のまま
    door_t = z["t"][door] - robot.t0
    sent_door = [q for (t, q, kp, kd) in robot.sent if door_t.min() <= t <= door_t.max()]
    assert len(sent_door) > 100
    assert max(np.abs(q[legs] - want).max() for q in sent_door) < 1e-9
