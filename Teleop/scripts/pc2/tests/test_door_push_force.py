"""door_push_force.py の試験（2026-10-30）。pinocchio の要らない所（扉の板の形・骨盤の傾き・区間のまとめ）だけ。Mac の Navigation/.venv で:

    G1_Hackason/Navigation/.venv/bin/python -m pytest G1_Hackason/Teleop/scripts/pc2/tests -q
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import door_push_force as P  # noqa: E402

W = 0.954   # UIS Room 4 の扉の幅（10-29 のライダー）


def test_hand_on_closed_door_pushing_inward_counts_as_pushing():
    # Arrange: 扉は閉じている（0°）。右手が板の手前 3 cm・蝶番から 0.8 m で、奥（+x）へ 12 N
    hand = np.array([-0.03, -W / 2 + 0.8])

    # Act
    c = P.panel_contact(hand, np.array([12.0, 0.0]), 0.0, W, lever_side=+1)

    # Assert
    assert c.on_panel and c.push_n == pytest.approx(12.0)
    assert c.from_hinge_m == pytest.approx(0.8) and c.off_panel_m == pytest.approx(-0.03)
    assert c.moment_nm == pytest.approx(0.8 * 12.0)


def test_panel_follows_the_door_angle_and_ignores_pulling():
    # Arrange: 扉が 60° 開いた。板の上の点を、板の向きに直して置く
    th = math.radians(60.0)
    hinge = np.array([0.0, -W / 2])
    u, n = np.array([math.sin(th), math.cos(th)]), np.array([math.cos(th), -math.sin(th)])
    hand = hinge + 0.7 * u - 0.02 * n

    # Act
    push = P.panel_contact(hand, 10.0 * n, 60.0, W, lever_side=+1)
    pull = P.panel_contact(hand, -10.0 * n, 60.0, W, lever_side=+1)

    # Assert: 板を奥へ押す向きだけを「押している」と数える
    assert push.on_panel and push.push_n == pytest.approx(10.0) and push.from_hinge_m == pytest.approx(0.7)
    assert not pull.on_panel and pull.moment_nm == 0.0


def test_lever_on_the_right_puts_the_hinge_on_the_left():
    # Arrange: 把手が右の扉（蝶番は左＝+y）。閉じた板の、蝶番から 0.6 m
    hand = np.array([-0.02, W / 2 - 0.6])

    # Act
    c = P.panel_contact(hand, np.array([8.0, 0.0]), 0.0, W, lever_side=-1)

    # Assert
    assert c.on_panel and c.from_hinge_m == pytest.approx(0.6)


def test_hand_far_from_the_panel_is_not_pushing():
    c = P.panel_contact(np.array([-0.40, 0.0]), np.array([20.0, 0.0]), 0.0, W, lever_side=+1)
    assert not c.on_panel


def test_pitch_deg_of_forward_lean():
    # Arrange: y 軸まわりに 5°（前へ傾く）。四元数は w, x, y, z
    h = math.radians(5.0) / 2

    # Act / Assert
    assert P.pitch_deg(np.array([[math.cos(h), 0.0, math.sin(h), 0.0]]))[0] == pytest.approx(5.0)


def test_segment_rows_measure_push_progress_and_hand_retraction():
    # Arrange: 10 Hz の 1 本。2〜4 秒に前へ 0.25 m/s の指令。その間、右手は板の上で 14 N、手先は骨盤から 3 cm 縮む
    t = np.arange(0.0, 6.0, 0.1)
    fwd = (t >= 2.0) & (t < 4.0)
    samples = {
        "t": t,
        "cmd_vx": np.where(fwd, 0.25, 0.0),
        "on_right": fwd.copy(),
        "push_right": np.where(fwd, 14.0, 0.0),
        "hand_x_right": np.where(fwd, 0.40 - 0.015 * (t - 2.0), 0.40),
        "angle": np.where(fwd, 10.0 + 3.0 * (t - 2.0), 0.0),
        "body_fwd": np.where(t < 2.0, 0.0, np.where(t < 4.0, 0.02 * (t - 2.0), 0.04)),
    }

    # Act
    rows = P.segment_rows(samples)

    # Assert
    assert len(rows) == 1
    r = rows[0]
    assert r["t0"] == pytest.approx(2.0) and r["cmd_vx"] == pytest.approx(0.25)
    assert r["on_right_frac"] == pytest.approx(1.0) and r["push_right_median"] == pytest.approx(14.0)
    assert r["hand_retract_m"] == pytest.approx(0.03, abs=0.003)
    assert r["body_fwd_m"] == pytest.approx(0.04, abs=0.003)
    assert r["angle_max"] == pytest.approx(15.7, abs=0.5)
