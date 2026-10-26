"""door_stand_guard.py（STAND の見張り・10-23 の再発防止）の試験。実機の記録（10-16 に立てた・10-23 に揺れた）があればそれでも確かめる"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import door_stand_guard as G  # noqa: E402

GOOD_1016 = HERE / "logs_real" / "door_real_20261016_175033.npz"
BAD_1023 = HERE / "logs_real" / "door_real_20261023_165924_damp.npz"


def test_balanced_load_passes():
    assert G.load_problem([11.0, 10.8, 11.2], [9.8, 9.6, 10.0]) == ""


def test_light_right_foot_fails():
    why = G.load_problem([7.0, 7.1], [1.9, 2.0])
    assert "右足" in why


def test_both_light_fails():
    assert G.load_problem([3.0], [3.2]) != ""


def test_oscillation_by_speed_and_tilt():
    dq = np.zeros(29)
    assert G.oscillation_problem(dq, 4.0) == ""
    dq[2] = 6.8
    assert "関節 2" in G.oscillation_problem(dq, 4.0)
    assert "傾き" in G.oscillation_problem(np.zeros(29), 12.2)


def test_arm_speed_does_not_trigger():
    dq = np.zeros(29)
    dq[20] = 15.0  # 腕（STAND では動かないはずだが、足だけを見る）
    assert G.oscillation_problem(dq, 3.0) == ""


def test_load_check_decides_once_after_window():
    chk = G.StandLoadCheck()
    tau = np.zeros(29)
    tau[4], tau[10] = 11.0, 2.0
    out = [chk.update(t, tau) for t in np.arange(0.0, 2.0, 0.02)]
    decided = [o for o in out if o is not None]
    assert len(decided) == 1 and "右足" in decided[0]
    assert not chk.passed


def _replay(path: Path):
    z = np.load(path)
    ph = z["phase"].astype(int)
    st = np.nonzero(ph == 3)[0]
    t = z["t"][st] - z["t"][st[0]]
    q = z["quat"][st]
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    chk = G.StandLoadCheck()
    load, osc_t = None, None
    for k in range(len(st)):
        v = chk.update(float(t[k]), z["tau"][st[k]])
        load = v if v is not None else load
        if osc_t is None and G.oscillation_problem(z["dq"][st[k]], float(tilt[k])):
            osc_t = float(t[k])
    return load, osc_t, float(t[-1])


@pytest.mark.skipif(not GOOD_1016.exists(), reason="10-16 の記録が無い")
def test_real_good_stand_1016_is_not_stopped():
    load, osc_t, dur = _replay(GOOD_1016)
    assert load == ""  # 両足に体重が乗っていた
    assert osc_t is None  # 144 秒のあいだ揺れの見張りは掛からない
    assert dur > 100


@pytest.mark.skipif(not BAD_1023.exists(), reason="10-23 の記録が無い")
def test_real_bad_stand_1023_is_caught_early():
    load, osc_t, dur = _replay(BAD_1023)
    assert "右足" in load  # 1.5 s で片足が軽いと分かる（揺れは 1.5 s から・脱力は 6.2 s）
    assert osc_t is not None and osc_t < 3.6  # 揺れの見張りも、13 rad/s に育つ前に掛かる
