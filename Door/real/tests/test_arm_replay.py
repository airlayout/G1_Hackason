"""arm_replay.py（11-11・実機の R1 腕の動きの再生・R4 腰 yaw の打ち消し）の試験。numpy だけ・機体なし。

  Navigation/.venv/bin/python -m pytest -q G1_Hackason/Door/real/tests/test_arm_replay.py

- 記録の読み込み: 段 2 の方策の区間（staged[:,0]==2 かつ staged[:,2]==2）・構えを含む区間・腕の NaN の行を除く・時刻は区間の始めから
- 時刻で補間・倍率（0.5 で 2 倍の長さ）・区間の外は端の値
- ±δ の守り: 今の角度 ±δ に収める
- 腰の打ち消し: −倍率 × 回り（上限）・変化の速さの上限・止めると 0 へ戻る・回りは ±π をまたいでも正しい
- 胴の向きの姿勢（yaw に打ち消しを足す）・観測の腰 yaw から打ち消しを引く（元は書き換えない）
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import arm_replay as arp  # noqa: E402
from door_frame import PlanarPose  # noqa: E402


def _record(tmp_path: Path) -> Path:
    """段 1（腕は NaN）→ 段 2 の構え（staged[:,2]==1）→ 段 2 の方策（==2）→ POST（staged NaN）の偽の記録"""
    t = np.arange(0.0, 6.0, 0.02)
    n = len(t)
    staged = np.full((n, 9), np.nan)
    arm = np.full((n, 14), np.nan)
    k1, k2, k3 = 50, 100, 250  # 1.0 s 段 2 の構え・2.0 s 方策・5.0 s 終わり
    staged[:k1, 0] = 1
    staged[k1:k3, 0] = 2
    staged[k1:k2, 2] = 1
    staged[k2:k3, 2] = 2
    arm[k1:k3] = np.linspace(0.0, 1.0, k3 - k1)[:, None] * np.arange(1, 15)[None, :] * 0.01
    p = tmp_path / "door_stock_test.npz"
    np.savez(p, t=t + 100.0, staged=staged, arm_sent=arm)
    return p


def test_load_policy_segment_and_stage2(tmp_path):
    p = _record(tmp_path)
    tr = arp.load_replay(p, segment="policy", speed=1.0)
    assert tr.t[0] == 0.0 and math.isclose(tr.duration, 2.98, abs_tol=1e-9)
    assert tr.arm.shape == (150, 14) and np.isfinite(tr.arm).all()
    full = arp.load_replay(p, segment="stage2", speed=1.0)
    assert math.isclose(full.duration, 3.98, abs_tol=1e-9) and full.arm.shape == (200, 14)
    with pytest.raises(ValueError):
        arp.load_replay(p, segment="policy", speed=0.0)
    with pytest.raises(ValueError):
        arp.load_replay(p, segment="walk", speed=1.0)


def test_track_at_interpolates_and_scales_time(tmp_path):
    tr = arp.load_replay(_record(tmp_path), segment="policy", speed=1.0)
    mid = arp.track_at(tr, 0.01)
    assert np.allclose(mid, (tr.arm[0] + tr.arm[1]) / 2)
    assert np.allclose(arp.track_at(tr, -1.0), tr.arm[0]) and np.allclose(arp.track_at(tr, 99.0), tr.arm[-1])
    slow = arp.load_replay(_record(tmp_path), segment="policy", speed=0.5)
    assert math.isclose(slow.duration, 2 * tr.duration, abs_tol=1e-9)
    assert np.allclose(arp.track_at(slow, 1.0), arp.track_at(tr, 0.5))


def test_guard_lead():
    q = np.zeros(14)
    target = np.array([0.5, -0.5] + [0.1] * 12)
    delta = np.array([0.2, 0.3] + [0.3] * 12)
    out = arp.guard_lead(target, q, delta)
    assert np.allclose(out[:2], [0.2, -0.3]) and np.allclose(out[2:], 0.1)
    assert target[0] == 0.5  # 元は書き換えない


def test_waist_comp_sign_clamp_rate_and_release():
    cfg = arp.WaistCompConfig(gain=1.0, max_rad=math.radians(35.0), rate=1.0)
    st = arp.WaistCompState()
    st, off, rate = arp.waist_comp_step(st, 0.3, True, 0.02, cfg)  # 始めの周期で基準を取る
    assert st.yaw0 == 0.3 and off == 0.0
    for _ in range(100):  # 2 s。回り +20° → 目標 −20° に 1 rad/s で寄る
        st, off, rate = arp.waist_comp_step(st, 0.3 + math.radians(20.0), True, 0.02, cfg)
    assert math.isclose(off, -math.radians(20.0), abs_tol=1e-9) and rate == 0.0
    st2, off2, _ = arp.waist_comp_step(st, 0.3 + math.radians(90.0), True, 0.02, cfg)
    assert off2 >= -math.radians(35.0) - 1e-12 and off2 <= off  # 上限の向きへ 1 周期ぶんだけ
    for _ in range(100):  # 止めると 0 へ戻り、基準も捨てる
        st, off, _ = arp.waist_comp_step(st, 1.0, False, 0.02, cfg)
    assert off == 0.0 and st.yaw0 is None
    st, off, _ = arp.waist_comp_step(arp.WaistCompState(), math.pi - 0.05, True, 0.02, cfg)
    for _ in range(50):
        st, off, _ = arp.waist_comp_step(st, -math.pi + 0.05, True, 0.02, cfg)  # +0.1 rad の回り（±π をまたぐ）
    assert math.isclose(off, -0.1, abs_tol=1e-9)


def test_waist_comp_returns_to_zero_on_nan_yaw():
    """IMU の値が壊れた周期は打ち消しを止めて 0 へ戻す（NaN を送らない・ねじれたまま固めない。レビューの指摘）"""
    cfg = arp.WaistCompConfig(gain=1.0, max_rad=0.6, rate=1.0)
    st, _, _ = arp.waist_comp_step(arp.WaistCompState(), 0.0, True, 0.02, cfg)
    for _ in range(20):
        st, off, _ = arp.waist_comp_step(st, 0.3, True, 0.02, cfg)
    assert off < -0.1
    for _ in range(30):
        st, off, rate = arp.waist_comp_step(st, float("nan"), True, 0.02, cfg)
        assert math.isfinite(off) and math.isfinite(rate)
    assert off == 0.0 and st.yaw0 is None


def test_waist_comp_rejects_non_finite_config():
    for bad in (arp.WaistCompConfig(float("nan"), 0.6, 1.0), arp.WaistCompConfig(1.0, float("inf"), 1.0),
                arp.WaistCompConfig(1.0, 0.6, float("nan"))):
        with pytest.raises(ValueError):
            arp.waist_comp_step(arp.WaistCompState(), 0.0, True, 0.02, bad)


def test_waist_comp_off_when_gain_zero():
    cfg = arp.WaistCompConfig(gain=0.0, max_rad=0.6, rate=1.0)
    st, off, rate = arp.waist_comp_step(arp.WaistCompState(), 0.0, True, 0.02, cfg)
    st, off, rate = arp.waist_comp_step(st, 0.5, True, 0.02, cfg)
    assert off == 0.0 and rate == 0.0


def test_torso_pose_and_hide_waist():
    pose = PlanarPose(-0.4, 0.3, -0.8)
    tp = arp.torso_pose(pose, -0.2)
    assert (tp.x, tp.y) == (-0.4, 0.3) and math.isclose(tp.yaw, -1.0)
    q, dq = np.zeros(29), np.zeros(29)
    q2, dq2 = arp.hide_waist(q, dq, -0.2, -0.5)
    assert q2[12] == 0.2 and dq2[12] == 0.5 and q[12] == 0.0 and np.count_nonzero(q2) == 1


def test_waist_with_yaw_changes_only_yaw():
    hold = np.array([0.1, -0.05, 0.2])
    out = arp.waist_with_yaw(hold, -0.3, -2.618, 2.618)
    assert np.allclose(out, [-0.2, -0.05, 0.2]) and hold[0] == 0.1
    assert arp.waist_with_yaw(hold, 5.0, -2.618, 2.618)[0] == 2.618  # yaw だけ可動域で切る
