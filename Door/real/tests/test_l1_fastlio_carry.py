"""l1_fastlio_carry_1110.py の試験: FAST-LIO の 3 次元の姿勢を床に投げる式と、出力を点群の回に当てる式。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import l1_fastlio_carry_1110 as l1  # noqa: E402
import lidar_door_est as est  # noqa: E402


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def world_from_raw(psi: float, pitch: float, roll: float) -> np.ndarray:
    """ライダーの生の座標（上下逆さまの取付）→ 重力で水平な世界。推定器の水平の座標と同じ「Rz·Ry·Rx·FLIP」"""
    return rz(psi) @ ry(pitch) @ rx(roll) @ est.FLIP


def make_track(poses: list[tuple[float, float, float, float, float]]) -> tuple[np.ndarray, est.Level]:
    """世界での (x, y, 向き, 前の垂れ, 横の傾き) の列 → FAST-LIO の camera_init → body（camera_init は 1 つ目の体）と 1 つ目の床"""
    W = []
    for x, y, psi, pitch, roll in poses:
        T = np.eye(4)
        T[:3, :3] = world_from_raw(psi, pitch, roll)
        T[:3, 3] = (x, y, 1.2)
        W.append(T)
    W = np.array(W)
    Ts = np.linalg.inv(W[0]) @ W
    _, _, _, p0, r0 = poses[0]
    level = est.Level(ry(p0) @ rx(r0) @ est.FLIP, 1.2)
    return Ts, level


def test_planar_track_recovers_floor_motion_with_tilt_and_nod() -> None:
    # Arrange: 頭は前に 20° 垂れ・横に 3° 傾いて始まり、歩く間に頷く（垂れが 12°〜26° に変わる）
    p0, r0 = math.radians(20), math.radians(3)
    poses = [(0.0, 0.0, 0.0, p0, r0), (0.30, 0.05, math.radians(-10), math.radians(12), r0),
             (0.55, 0.40, math.radians(-60), math.radians(26), math.radians(-2)),
             (0.50, 0.45, math.radians(-73), p0, r0)]
    Ts, level = make_track(poses)
    # Act
    got = l1.planar_track(Ts, Ts[0], level)
    # Assert: 1 つ目の体の水平の座標（向き 0・原点）で見た位置と向きが、世界の値そのものになる
    want = np.array([(x, y, psi) for x, y, psi, _, _ in poses])
    np.testing.assert_allclose(got, want, atol=1e-9)


def test_planar_track_in_rotated_world() -> None:
    # Arrange: 1 つ目の体の向きが世界で 40° でも、段 0 の向きを 0 とした差になる
    poses = [(1.0, 2.0, math.radians(40), 0.15, 0.02), (1.0 + 0.5 * math.cos(math.radians(40)),
                                                       2.0 + 0.5 * math.sin(math.radians(40)), math.radians(10), 0.3, 0.0)]
    Ts, level = make_track(poses)
    got = l1.planar_track(Ts, Ts[0], level)
    np.testing.assert_allclose(got[1], [0.5, 0.0, math.radians(-30)], atol=1e-9)


def test_compose_on_door_frame() -> None:
    a = np.array([-1.0, 0.1, math.radians(90)])
    got = l1.compose(a, np.array([[0.2, 0.0, 0.1]]))
    np.testing.assert_allclose(got[0], [-1.0, 0.3, math.radians(90) + 0.1], atol=1e-12)


def test_match_to_scans_uses_recv_time_of_same_scan() -> None:
    # Arrange: ライダーの時計は受け取りより 1.99 s 遅れ、受け取りはヘッダの 0.13 s 後（11-04 の型）
    hdr = 1000.0 + 0.1 * np.arange(50)
    recv = hdr + 1.99 + 0.13
    fl = hdr[[3, 4, 10, 40]] + 0.1 + np.array([0.001, -0.002, 0.0, 0.05])  # 最後は回の間（どの回にも当たらない）
    # Act
    t_wall, ok, rep = l1.match_to_scans(fl, recv, hdr)
    # Assert
    assert ok.tolist() == [True, True, True, False]
    np.testing.assert_allclose(t_wall[:3], recv[[3, 4, 10]] - 0.055 + 0.05, atol=1e-9)  # 走査の終わり
    assert rep["回に当たった"] == 3


def test_mean_pose_is_rotation() -> None:
    T = np.eye(4)
    T[:3, :3] = rz(0.3) @ ry(0.1)
    T2 = T.copy()
    T2[:3, :3] = rz(0.31) @ ry(0.1)
    m = l1.mean_pose(np.array([T, T2]))
    np.testing.assert_allclose(m[:3, :3] @ m[:3, :3].T, np.eye(3), atol=1e-12)
    assert abs(math.atan2(m[1, 0], m[0, 0]) - 0.305) < 2e-3


def test_match_to_scans_keeps_one_output_per_scan() -> None:
    hdr = 1000.0 + 0.1 * np.arange(10)
    recv = hdr + 0.13
    fl = np.array([hdr[2] + 0.1, hdr[2] + 0.105, hdr[3] + 0.1])  # 2 つ目は同じ回に当たる
    _, ok, _ = l1.match_to_scans(fl, recv, hdr)
    assert ok.tolist() == [True, False, True]


def test_interp_planar_unwraps_yaw() -> None:
    t = np.array([0.0, 1.0])
    p = np.array([[0.0, 0.0, math.radians(179)], [1.0, 2.0, math.radians(-179)]])
    got = l1.interp_planar(np.array([0.5]), t, p)
    np.testing.assert_allclose(got[0, :2], [0.5, 1.0])
    assert abs(abs(got[0, 2]) - math.pi) < 1e-9  # 0 でなく ±180°


def test_match_rows_with_tolerance() -> None:
    rows = np.array([10.0, 10.1, 10.2])
    got = l1.match_rows(np.array([10.1 + 1e-6, 10.15, 9.0]), rows)
    assert got.tolist() == [1, -1, -1]
