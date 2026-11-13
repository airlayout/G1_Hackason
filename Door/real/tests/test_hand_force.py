"""11-13 hand_force.HandForce: 腕の関節の力から手の力を逆に解く。合成した関節の力（重さ − Jᵀ F ＋ 基準のずれ）で確かめる。機体は要らない。
（MuJoCo の逆動力学との突き合わせは ana_hand_force_1113.py・シムの接触センサとの突き合わせは計画 16 節）"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import g1_arm_fk as fk  # noqa: E402
import hand_force as hfm  # noqa: E402

LEVEL = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz・pelvis が水平
POSE7 = np.array([-0.6, 0.5, 0.3, -0.3, 1.5, 0.4, -0.5])  # 台本の基準の姿勢（左腕）


@pytest.fixture(scope="module")
def chain():
    return fk.load_chain()


def make_q(hf: hfm.HandForce, q7=POSE7) -> np.ndarray:
    q = np.zeros(29)
    q[hf.idx] = q7
    return q


def tau_for(hf: hfm.HandForce, q: np.ndarray, quat: np.ndarray, F: np.ndarray, offset: np.ndarray) -> np.ndarray:
    """手の力 F（pelvis の座標・手が受ける力）を受けて止まっている腕の関節の力（読みのずれ offset 付き）"""
    g, J = hf.gravity_and_jacobian(q, quat)
    tau = np.zeros(29)
    tau[hf.idx] = g - J.T @ F - offset
    return tau


def test_force_is_nan_until_calibrated(chain):
    hf = hfm.HandForce(chain, fk.default_urdf(), "left", tip_m=hfm.TIP_M)
    q = make_q(hf)
    assert np.isnan(hf.force(q, LEVEL, np.zeros(29))).all()


def test_no_contact_reads_zero_after_calibration_even_with_sensor_offset(chain):
    hf = hfm.HandForce(chain, fk.default_urdf(), "left", tip_m=hfm.TIP_M)
    q = make_q(hf)
    offset = np.array([5.0, -1.0, 0.5, 0.3, 0.0, 0.1, 0.0])  # 10-29 の左腕の 5 N·m のずれの形
    for _ in range(hfm.CALIB_N):
        hf.calibrate(q, LEVEL, tau_for(hf, q, LEVEL, np.zeros(3), offset))
    for _ in range(30):
        F = hf.force(q, LEVEL, tau_for(hf, q, LEVEL, np.zeros(3), offset))
    assert np.allclose(F, 0.0, atol=1e-6)


@pytest.mark.parametrize("F", [np.array([-20.0, 0.0, 0.0]), np.array([-8.0, 3.0, 12.0]), np.array([0.0, 0.0, 15.0])])
def test_recovers_known_hand_force_at_the_tip(chain, F):
    hf = hfm.HandForce(chain, fk.default_urdf(), "left", tip_m=hfm.TIP_M)
    q = make_q(hf)
    offset = np.array([2.0, 0.0, -0.4, 0.2, 0.0, 0.0, 0.1])
    for _ in range(hfm.CALIB_N):
        hf.calibrate(q, LEVEL, tau_for(hf, q, LEVEL, np.zeros(3), offset))
    for _ in range(60):  # なましが落ち着くまで
        est = hf.force(q, LEVEL, tau_for(hf, q, LEVEL, F, offset))
    assert np.allclose(est, F, atol=0.05)


def test_gravity_follows_pelvis_tilt(chain):
    """pelvis が前へ傾いても、傾きを渡せば外の力は 0 のまま（重さの向きを pelvis の座標へ回している）"""
    hf = hfm.HandForce(chain, fk.default_urdf(), "left", tip_m=hfm.TIP_M)
    q = make_q(hf)
    a = np.radians(8.0)
    tilt = np.array([np.cos(a / 2), 0.0, np.sin(a / 2), 0.0])  # y 軸回りに 8°
    for _ in range(hfm.CALIB_N):
        hf.calibrate(q, LEVEL, tau_for(hf, q, LEVEL, np.zeros(3), np.zeros(7)))
    g_level, _ = hf.gravity_and_jacobian(q, LEVEL)
    g_tilt, _ = hf.gravity_and_jacobian(q, tilt)
    assert not np.allclose(g_level, g_tilt)
    for _ in range(30):
        F = hf.force(q, tilt, tau_for(hf, q, tilt, np.zeros(3), np.zeros(7)))
    assert np.allclose(F, 0.0, atol=1e-6)


def test_reset_drops_calibration(chain):
    hf = hfm.HandForce(chain, fk.default_urdf(), "left")
    q = make_q(hf)
    hf.calibrate(q, LEVEL, tau_for(hf, q, LEVEL, np.zeros(3), np.zeros(7)))
    hf.reset()
    assert hf.res0 is None and np.isnan(hf.force(q, LEVEL, np.zeros(29))).all()
