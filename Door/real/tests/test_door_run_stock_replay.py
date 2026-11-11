"""door_run_stock.py の 11-11 の口（R1 --replay-arm・R4 --waist-comp-*）の試験。偽の機体で輪を丸ごと回す（test_door_run_stock の _run）。

  DOOR_POLICY=<書き出した npz> Navigation/.venv/bin/python -m pytest -q G1_Hackason/Door/real/tests/test_door_run_stock_replay.py

- 再生: d で DOOR に入ると記録の腕を時刻どおりに送り、再生の長さで POST へ。歩きの指令は 0。腰の保ちは借りた時の 1 回だけ（打ち消し 0）
- 打ち消し: 体が回ると腰 yaw の目標が回りの逆へ（毎周期書く）。POST の後は 0 へ戻る
- 回りの止め: --replay-max-yaw-deg を越えたら再生の途中で POST
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import arm_replay as arp  # noqa: E402
import door_core as dc  # noqa: E402
import test_door_run_stock as tdrs  # noqa: E402
from test_door_real import _State, needs_policy  # noqa: E402

SCRIPT = [(0.3, "a"), (3.8, "d"), (5.6, "r"), (9.0, "q")]
REPLAY_S = 1.0


def _replay_npz(tmp_path: Path) -> Path:
    """段 2 の方策の区間 1.0 秒。腕は学習の初めの姿勢のまわりを ±0.1 rad で動く（守り ±0.2 の内）"""
    p = dc.load_params(tdrs.POLICY)
    t = np.arange(0.0, REPLAY_S + 1e-9, 0.02)
    n = len(t)
    staged = np.full((n, 9), np.nan)
    staged[:, 0], staged[:, 2] = 2, 2
    base = p.q_default[dc.NUM_HOMIE:dc.NUM_DOF]
    wave = 0.1 * np.sin(2 * math.pi * t / REPLAY_S)[:, None] * np.linspace(1.0, 0.3, 14)[None, :]
    tmp_path.mkdir(parents=True, exist_ok=True)
    path = tmp_path / "replay_src.npz"
    np.savez(path, t=t + 50.0, staged=staged, arm_sent=base[None, :] + wave)
    return path


def _yaw_state(yaw_fn):
    """偽の lowstate に、起動からの秒で決まる yaw（pelvis の IMU）を足す"""
    t0 = time.monotonic()

    class YawState(_State):
        def __init__(self, *args, **kw) -> None:
            super().__init__(*args, **kw)
            h = yaw_fn(time.monotonic() - t0) / 2
            self.imu_state.quaternion = [math.cos(h), 0.0, 0.0, math.sin(h)]

    return YawState


def _door_rows(phases):
    return [k for k, ph in enumerate(phases) if ph == "DOOR"]


@needs_policy
def test_replay_follows_record_and_stands(monkeypatch, tmp_path):
    src = _replay_npz(tmp_path / "src")
    rc, seen, phases, z, arms, loco, t0 = tdrs._run(monkeypatch, tmp_path / "log", SCRIPT,
                                                    extra_args=("--replay-arm", str(src)))
    assert rc == 0
    assert seen == ["WAIT", "ARMS_IN", "READY", "DOOR", "POST", "ARMS_OUT", "WAIT"]
    door = _door_rows(phases)
    assert 45 <= len(door) <= 56  # 再生の 1.0 秒
    tr = z["replay"][door, 0]
    assert tr[0] < 0.05 and abs(tr[-1] - REPLAY_S) < 0.06 and (np.diff(tr) > 0).all()
    track = arp.load_replay(src)
    import door_run_stock as drs

    p = dc.load_params(tdrs.POLICY)
    q_arm, kp_arm = p.q_default[dc.NUM_HOMIE:dc.NUM_DOF], p.kp[dc.NUM_HOMIE:dc.NUM_DOF]
    # 送る値は DOOR の肩 roll の押し付けの上限（6 N·m）を通る（実機の記録の arm_sent も切った後の値）
    want = np.array([drs.cap_roll_press(arp.track_at(track, v), q_arm, kp_arm, drs.ROLL_PRESS_MAX_NM) for v in tr])
    assert np.abs(z["arm_sent"][door] - want).max() < 0.02
    t_door = float(z["t"][door[0]])
    assert all(c[2][:3] == (0.0, 0.0, 0.0) for c in loco.vel() if c[1] > t_door + 0.1)  # 脚は立ったまま
    assert len(tdrs.WAIST_HOLDS) == 1  # 打ち消しを切っている時は元のまま
    assert np.isnan(z["waist_comp"][:, 2]).all() and (z["waist_comp"][:, 0] == 0.0).all()


@needs_policy
def test_waist_comp_follows_body_turn_and_returns(monkeypatch, tmp_path):
    src = _replay_npz(tmp_path / "src")
    turn = math.radians(20.0)
    monkeypatch.setattr(tdrs, "_State", _yaw_state(lambda el: 0.0 if el < 4.0 else min(turn, (el - 4.0) * math.radians(60.0))))
    rc, seen, phases, z, arms, loco, t0 = tdrs._run(monkeypatch, tmp_path / "log", SCRIPT,
                                                    extra_args=("--replay-arm", str(src), "--waist-comp-gain", "1.0",
                                                                "--waist-comp-rate", "2.0"))
    assert rc == 0 and "DOOR" in seen
    hold = tdrs.WAIST_HOLDS[0][0]
    yaw_targets = np.array([w[0][0] for w in tdrs.WAIST_HOLDS])
    assert len(tdrs.WAIST_HOLDS) > 50  # 毎周期書く
    assert abs((yaw_targets - hold[0]).min() + turn) < math.radians(1.5)  # 回り +20° の逆へ −20°
    door = _door_rows(phases)
    assert abs(z["waist_comp"][door, 0].min() + turn) < math.radians(1.5)
    assert abs(z["waist_comp"][-1, 0]) < 1e-9  # POST・ARMS_OUT の後は 0 へ戻る
    assert abs(yaw_targets[-1] - hold[0]) < 1e-9


@needs_policy
def test_replay_stops_on_body_turn(monkeypatch, tmp_path):
    src = _replay_npz(tmp_path / "src")
    monkeypatch.setattr(tdrs, "_State", _yaw_state(lambda el: 0.0 if el < 4.0 else (el - 4.0) * math.radians(60.0)))
    rc, seen, phases, z, arms, loco, t0 = tdrs._run(monkeypatch, tmp_path / "log", SCRIPT,
                                                    extra_args=("--replay-arm", str(src), "--replay-max-yaw-deg", "10"))
    assert rc == 0 and "POST" in seen
    door = _door_rows(phases)
    assert len(door) < 45  # 再生の途中で止まった
    assert np.nanmax(np.abs(z["replay"][door, 1])) <= 10.0 + 60.0 * 0.02 * 2  # 線を越えた周期で止まる


def test_cli_rejects_non_finite_and_out_of_range():
    """レビューの指摘（11-11）: nan・inf・大きすぎる値は起動の前に止める（腰へ NaN や最大の角度を送らない）"""
    import pytest

    import door_run_stock as drs

    base = ["--policy", "x.npz", "--door", "0.94,2.05,0.99,0.064", "--door-source", "tape", "--start-x=-1", "--start-y=0"]
    for bad in (["--waist-comp-gain", "nan"], ["--waist-comp-gain", "3"], ["--waist-comp-rate", "nan"], ["--waist-comp-rate", "inf"],
                ["--waist-comp-max-deg", "nan"], ["--waist-comp-max-deg", "60"], ["--replay-speed", "nan"], ["--replay-speed", "0"],
                ["--replay-max-yaw-deg", "nan"]):
        with pytest.raises(SystemExit):
            drs.parse_args(base + bad)
    a = drs.parse_args(base + ["--waist-comp-gain", "1.0", "--replay-speed", "0.5"])
    assert a.waist_comp_gain == 1.0 and a.replay_speed == 0.5
