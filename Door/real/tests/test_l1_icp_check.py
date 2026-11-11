"""l1_icp_check_1110.planar_of の試験: 頭の傾きが変わっても、ICP の答えからライダーそのものの位置と向きが出る。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import l1_icp_check_1110 as chk  # noqa: E402
import lidar_door_est as est  # noqa: E402
from test_l1_fastlio_carry import rx, ry, rz  # noqa: E402


def test_planar_of_follows_lidar_origin_when_head_tilt_changes() -> None:
    # Arrange: 段 0 は前に 12° 垂れ、着いた先は腕を上げて 15° に（横にも 2°）。ライダーは (0.6, 0.45) へ、向き −21°
    h = 1.2
    Ra = ry(math.radians(12)) @ rx(math.radians(1)) @ est.FLIP
    level = est.Level(Ra, h)
    x, y, psi = 0.60, 0.45, math.radians(-21)
    A1 = rz(psi) @ ry(math.radians(15)) @ rx(math.radians(-2)) @ est.FLIP
    # ICP が返す変換: 着いた先の点を段 0 の床の水平（q' = Ra p + [0,0,h]）で表したもの → 段 0 の水平の座標
    T = np.eye(4)
    T[:3, :3] = A1 @ Ra.T
    T[:3, 3] = np.array([x, y, h]) - A1 @ Ra.T @ np.array([0.0, 0.0, h])
    # Act
    got = chk.planar_of(T, level)
    # Assert
    np.testing.assert_allclose(got, [x, y, psi], atol=1e-9)
    # 並進をそのまま読むと、傾きの変わり 3° の分（約 1.2 m × 3°）ずれる＝直す前の読み方
    assert np.hypot(T[0, 3] - x, T[1, 3] - y) > 0.05
