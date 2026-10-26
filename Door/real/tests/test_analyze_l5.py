"""analyze_l5.py（L5 の記録の解析）の試験。作り物の記録で、odometry だけで運んだ姿勢と生徒に渡した姿勢の差が戻るかを見る"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import analyze_l5 as A  # noqa: E402


def _fake_log(n_wait=50, n_door=250, n_post=50, lat_scale=0.9):
    """扉の方策の 5 秒で横へ 0.5 m 歩く。odometry は横を 0.9 倍に短く出す。生徒に渡した姿勢は本当の値"""
    n = n_wait + n_door + n_post
    t = np.arange(n) * 0.02
    phase = np.r_[np.full(n_wait, A.PHASES.index("STAND")), np.full(n_door, A.PHASES.index("DOOR")), np.full(n_post, A.PHASES.index("POST"))]
    y = np.r_[np.zeros(n_wait), np.linspace(0, 0.5, n_door), np.full(n_post, 0.5)]
    pose = np.c_[np.full(n, -0.8), y, np.zeros(n)]
    odom = np.c_[np.full(n, 3.0), 1.0 + y * lat_scale, np.zeros(n)]  # odometry の座標は扉と同じ向き・原点だけ違う
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    ang = np.c_[np.radians(np.r_[np.zeros(n_wait), np.linspace(0, 20, n_door), np.full(n_post, 20)]), np.ones(n), np.full(n, 0.1)]
    return {"t": t, "phase": phase, "pose": pose, "odom": odom, "quat": quat, "period": np.full(n, 0.02),
            "lidar_rx_age": np.full(n, 0.03), "door_angle_obs": ang}


def test_odometry_shortfall_is_reported_as_difference():
    # Act
    s = A.summarize(_fake_log())

    # Assert: 横に 0.5 m・odometry は 0.9 倍 → 最後に 5 cm の差
    assert s["DOOR から"]["最後 cm"] == pytest.approx(5.0, abs=0.1)
    assert s["DOOR から"]["歩いた道のり m"] == pytest.approx(0.5, abs=0.01)
    assert s["扉の角度"]["最大°"] == pytest.approx(20.0, abs=0.01)
    assert s["phases"]["DOOR"]["周期最大ms"] == pytest.approx(20.0)


def test_no_difference_when_odometry_is_exact():
    s = A.summarize(_fake_log(lat_scale=1.0))
    assert s["DOOR から"]["当て直し−odometry 最大 cm"] < 1e-6
    assert math.isfinite(s["電文の古さ"]["最大ms"])
