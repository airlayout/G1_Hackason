"""11-12 台本（S1）: 腕の逆運動学 g1_arm_ik.py。機体は要らない"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import g1_arm_fk as fk  # noqa: E402
import g1_arm_ik as aik  # noqa: E402

POSTURE = np.array([-0.8, 0.17, 0.64, -0.67, 1.85, 0.64, -0.65])


@pytest.fixture(scope="module")
def ik() -> aik.ArmIK:
    return aik.ArmIK(fk.load_chain(), "left")


def test_arm_indices_follow_sdk_order():
    assert aik.arm_indices("left") == list(range(15, 22))
    assert aik.arm_indices("right") == list(range(22, 29))
    with pytest.raises(ValueError):
        aik.arm_indices("middle")


def test_jacobian_matches_numeric_difference(ik):
    q = np.zeros(29)
    q[12] = 0.2  # 腰 yaw が回っていても合う
    _, J = ik.jacobian(q, POSTURE)
    Jn = np.zeros((3, 7))
    for k in range(7):
        d = np.zeros(7)
        d[k] = 1e-6
        Jn[:, k] = (ik.palm(q, POSTURE + d) - ik.palm(q, POSTURE - d)) / 2e-6
    assert np.abs(J - Jn).max() < 1e-6


def test_solve_reaches_target_within_5mm_and_stays_inside_limits(ik):
    q = np.zeros(29)
    start = ik.palm(q, POSTURE)
    for off in ([0.05, 0.0, 0.0], [0.0, -0.05, 0.03], [0.04, 0.03, -0.08]):
        q7, err = ik.solve(q, POSTURE, start + np.array(off), POSTURE)
        assert err < 0.005, (off, err)
        assert (q7 >= ik.lo - 1e-9).all() and (q7 <= ik.hi + 1e-9).all()
        assert np.linalg.norm(ik.palm(q, q7) - (start + np.array(off))) < 0.005


def test_step_moves_at_most_max_step_per_call(ik):
    q = np.zeros(29)
    far = ik.palm(q, POSTURE) + np.array([0.3, 0.0, 0.0])
    q7 = ik.step(q, POSTURE, far, POSTURE)
    assert np.abs(q7 - POSTURE).max() <= aik.MAX_STEP_RAD + 1e-9


def test_step_returns_input_when_target_is_not_finite(ik):
    q = np.zeros(29)
    out = ik.step(q, POSTURE, np.array([np.nan, 0.0, 0.0]), POSTURE)
    assert np.array_equal(out, POSTURE) and out is not POSTURE


def test_unreachable_target_leaves_residual_instead_of_blowing_up(ik):
    q = np.zeros(29)
    q7, err = ik.solve(q, POSTURE, np.array([1.5, 0.3, 0.3]), POSTURE, iters=200)
    assert np.isfinite(q7).all() and err > 0.5
