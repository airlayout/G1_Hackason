"""lidar_door_est.py（頭のライダーで扉の座標でのライダーの姿勢と扉の角度を出す）の試験。人工の点群で、置いた値に戻るかを見る。

  uvx --with numpy pytest -q G1_Hackason/Door/real/tests
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import lidar_door_est as L  # noqa: E402
import synthetic_door as S  # noqa: E402

START = (-0.80, 0.06, math.radians(2.0))


@pytest.fixture(scope="module")
def world():
    return S.scene(0.0)


@pytest.fixture(scope="module")
def tracker_scans(world):
    rng = np.random.default_rng(1)
    return [S.lidar_scan(world, *START, pitch_deg=5.0, noise=0.003, rng=rng) for _ in range(5)]


def _tracker(scans):
    return L.DoorTracker.from_still(scans, S.THICK, lever_side=+1)


def test_pose2d_compose_inverse_roundtrip():
    # Arrange
    a = L.Pose2D(0.3, -0.2, 0.7)
    b = L.Pose2D(-1.1, 0.4, -2.0)

    # Act
    back = a.compose(b).compose(b.inverse())

    # Assert
    assert back.x == pytest.approx(a.x) and back.y == pytest.approx(a.y) and back.yaw == pytest.approx(a.yaw)


def test_still_measurement_recovers_door_and_lidar_pose(tracker_scans):
    # Act
    tr = _tracker(tracker_scans)

    # Assert: 幅・ライダーの姿勢（扉の座標）・床からの高さ・頭の傾き
    p = tr.door.lidar_in_door
    assert tr.door.width == pytest.approx(S.WIDTH, abs=0.01)
    assert p.x == pytest.approx(START[0], abs=0.005)
    assert p.y == pytest.approx(START[1], abs=0.005)
    assert math.degrees(p.yaw) == pytest.approx(math.degrees(START[2]), abs=0.2)
    assert tr.level.h == pytest.approx(1.21, abs=0.005)
    assert tr.level.tilt_deg() == pytest.approx(5.0, abs=0.3)


def test_tracker_follows_walk_and_turn(world, tracker_scans):
    # Arrange: 止まって測った後、横へ 0.4 m・前へ 0.2 m 歩きながら右へ 60° 回る（10 Hz・3 秒）
    tr = _tracker(tracker_scans)
    rng = np.random.default_rng(2)
    n = 30
    errs = []

    # Act
    for k in range(1, n + 1):
        f = k / n
        x, y, yaw = START[0] + 0.2 * f, START[1] - 0.4 * f, START[2] - math.radians(60.0) * f
        e = tr.step(0.1 * k, S.lidar_scan(world, x, y, yaw, pitch_deg=5.0 + 2.0 * math.sin(k), noise=0.003, rng=rng))
        assert e.accepted
        errs.append((e.lidar.x - x, e.lidar.y - y, math.degrees(e.lidar.yaw - yaw)))

    # Assert: 生徒に要る精度は位置 1.5 cm・向き 1°（10-17）。点の揺れ 3 mm・頭の揺れ ±2° で、最大 1 cm・中央 4 mm・向き 0.3° 以内
    errs = np.abs(np.array(errs))
    assert errs[:, :2].max() < 0.010
    assert np.median(errs[:, :2]) < 0.004
    assert errs[:, 2].max() < 0.3


@pytest.mark.parametrize("angle", [0.0, 20.0, 45.0, 75.0])
def test_door_angle_from_tracked_pose(angle, tracker_scans):
    # Arrange: 止まって測った後に扉を開ける（機体は動かない）
    tr = _tracker(tracker_scans)
    scan = S.lidar_scan(S.scene(angle, lever=False), *START, pitch_deg=5.0, noise=0.003)

    # Act
    e = tr.step(0.1, scan)

    # Assert
    assert e.accepted
    assert e.angle_deg == pytest.approx(angle, abs=1.0)


def test_door_opening_does_not_move_pose(tracker_scans):
    # Arrange: 扉が 0 → 80° と開いていく間、機体は止まっている。開いた板は地図に無いので、重ね直しに使わないこと
    tr = _tracker(tracker_scans)

    # Act
    poses = [tr.step(0.1 * (k + 1), S.lidar_scan(S.scene(float(a), lever=False), *START, pitch_deg=5.0)).lidar
             for k, a in enumerate(range(0, 81, 5))]

    # Assert
    assert max(abs(p.x - START[0]) for p in poses) < 0.005
    assert max(abs(p.y - START[1]) for p in poses) < 0.005


def test_jump_is_rejected_and_prediction_kept(world, tracker_scans):
    # Arrange: 1 回だけ 30 cm 離れた所の点群（外れの模擬）が来る
    tr = _tracker(tracker_scans)
    tr.step(0.1, S.lidar_scan(world, *START, pitch_deg=5.0))

    # Act: 初期値を 30 cm ずらして与え、跳びの上限で捨てられるか（予測を保つ）
    far = L.Pose2D(START[0] + 0.3, START[1], START[2])
    e = tr.step(0.2, S.lidar_scan(world, START[0] + 0.3, START[1], START[2], pitch_deg=5.0))

    # Assert: 予測（その場）から 30 cm 跳ぶ結果は受け入れない
    assert not e.accepted
    assert abs(e.lidar.x - START[0]) < 0.01
    assert far.x - e.lidar.x > 0.25


def test_lost_after_one_second_of_rejections(world, tracker_scans):
    # Arrange
    tr = _tracker(tracker_scans)
    tr.step(0.1, S.lidar_scan(world, *START, pitch_deg=5.0))

    # Act: 地図に当たらない点（何も無い所）だけの回が続く
    empty = np.random.default_rng(3).uniform(-0.2, 0.2, (500, 3)) + np.array([0.0, 0.0, -1.2])
    lost = [tr.step(0.1 + 0.1 * k, empty).lost for k in range(1, 16)]

    # Assert: 1 秒までは見失っていない・それを越えたら見失った
    assert not any(lost[:9])
    assert lost[-1]


def test_closed_door_far_from_hinge_spike_is_rejected(tracker_scans):
    # Arrange: 扉は閉じているのに、蝶番の近くに板でない小さな塊（自分の腕など）がある
    tr = _tracker(tracker_scans)
    world = S.scene(0.0, lever=False)
    blob = np.c_[np.full(200, -0.3), np.random.default_rng(4).uniform(-0.3, -0.2, 200), np.random.default_rng(5).uniform(0.4, 1.2, 200)]
    scan = S.lidar_scan(np.concatenate([world, blob]), *START, pitch_deg=5.0)

    # Act
    e = tr.step(0.1, scan)

    # Assert: 閉じた板（0°）を読むか、測れないと言う。塊の角度は返さない
    assert (not math.isfinite(e.angle_deg)) or abs(e.angle_deg) < 1.5


def test_relocalize_finds_pose_without_initial_guess(world, tracker_scans):
    # Arrange: 別の所から始まった記録（扉の右へ 0.8 m・向き 30°）
    tr = _tracker(tracker_scans)
    truth = (-1.1, -0.8, math.radians(30.0))
    lev = tr.level
    xy = L.band_xy(lev.apply(L.valid_points(S.lidar_scan(world, *truth, pitch_deg=5.0))))

    # Act
    res = L.relocalize(tr.map, xy, tr.door)

    # Assert
    assert res.ok
    assert res.pose.x == pytest.approx(truth[0], abs=0.01) and res.pose.y == pytest.approx(truth[1], abs=0.01)
    assert math.degrees(res.pose.yaw) == pytest.approx(30.0, abs=0.5)


def test_head_roll_does_not_bias_yaw(world):
    # Arrange: 頭が前に 24° 垂れ、横に 4° 傾いている（10-21 の実機は 24°）。止まって測った向きが本当の向きと合うか。
    # 歩く間の頭の揺れ（±1〜2°）で横の傾きが 2.5° 変わった回も見る
    truth = (-0.80, 0.06, math.radians(2.0))
    scans = [S.lidar_scan(world, *truth, pitch_deg=24.0, roll_deg=4.0, noise=0.002, rng=np.random.default_rng(k)) for k in range(5)]

    # Act
    tr = L.DoorTracker.from_still(scans, S.THICK)
    e = tr.step(0.1, S.lidar_scan(world, *truth, pitch_deg=23.0, roll_deg=1.5, noise=0.002))

    # Assert: 最小の回転で水平にすると 0.6〜1.3° 偏った（10-23 のレビュー）。傾きだけの回転なら 0.2° 以内
    assert math.degrees(tr.door.lidar_in_door.yaw) == pytest.approx(2.0, abs=0.2)
    assert e.accepted and math.degrees(e.lidar.yaw) == pytest.approx(2.0, abs=0.2)


def test_noisy_scan_is_not_accepted(world, tracker_scans):
    # Arrange: 点が 8 cm も揺れる回（当てはまりが悪い）
    tr = _tracker(tracker_scans)
    scan = S.lidar_scan(world, *START, pitch_deg=5.0, noise=0.08, rng=np.random.default_rng(9))

    # Act
    e = tr.step(0.1, scan)

    # Assert: 残差が上限を越えるので受け入れない（予測を保つ）
    assert not e.accepted
    assert abs(e.lidar.x - START[0]) < 0.01


def test_still_with_open_door_refuses_to_build():
    # Arrange: 止まって測る時に扉が 15° 開いている
    scans = [S.lidar_scan(S.scene(15.0, lever=False), *START, pitch_deg=5.0, noise=0.003, rng=np.random.default_rng(k)) for k in range(5)]

    # Act / Assert: 扉を作らない（縁が見つからないか、閉じていないと言う）
    with pytest.raises(ValueError):
        L.DoorTracker.from_still(scans, S.THICK)
