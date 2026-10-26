"""door_lidar_msg.py（推定器 → 制御の電文と、ライダー → pelvis の直し）と door_lidar_source.py（制御の側の受け取り）の試験"""
from __future__ import annotations

import math
import socket
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import door_lidar_msg as M  # noqa: E402
from door_lidar_source import LidarDoorSource  # noqa: E402


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _msg(seq: int, stamp: float, x=-0.8, y=0.05, yaw=0.0, angle=0.0, state=M.STATE_OK | M.STATE_ANGLE_OK) -> M.LidarMsg:
    return M.LidarMsg(seq, stamp, x, y, yaw, angle, 500, state, 0.007, 23.0)


def test_pack_unpack_roundtrip():
    m = _msg(7, 1234.5, -0.81, 0.07, 0.03, math.radians(12.0))
    got = M.unpack(M.pack(m))
    assert got is not None
    assert got.seq == 7 and got.stamp == pytest.approx(1234.5)
    assert (got.x, got.y, got.yaw) == pytest.approx((-0.81, 0.07, 0.03))
    assert got.angle == pytest.approx(math.radians(12.0)) and got.pose_ok and got.angle_ok


def test_unpack_rejects_garbage_and_nan():
    assert M.unpack(b"xxxx") is None
    assert M.unpack(b"\0" * M.SIZE) is None  # magic が違う
    assert M.unpack(M.pack(_msg(1, float("nan")))) is None


def test_lidar_above_pelvis_when_upright():
    # Arrange: 腰 0・傾き 0
    o = M.lidar_offset_xy((0.0, 0.0, 0.0), 0.0, 0.0, (0.0, 0.0))

    # Assert: URDF ではライダーは pelvis のほぼ真上（前に −3.7 mm）
    assert o[0] == pytest.approx(0.0002835 - 0.0039635, abs=1e-6)
    assert abs(o[1]) < 1e-4


def test_waist_pitch_and_pelvis_pitch_move_lidar_forward():
    # Arrange: pelvis が 2.3° 前へ傾き、腰で 2.2° 起こす（09-28 の実測）→ ほぼ打ち消す
    o_cancel = M.lidar_offset_xy((0.0, 0.0, math.radians(-2.2)), 0.0, math.radians(2.3), (0.0, 0.0))
    o_lean = M.lidar_offset_xy((0.0, 0.0, 0.0), 0.0, math.radians(2.3), (0.0, 0.0))

    # Assert: 打ち消すと数 mm 以内（腰の関節が pelvis の 5 cm 上にあるので約 3 mm 残る）・pelvis だけ傾くと約 1.8 cm 前（0.46 m × sin 2.3°）
    assert abs(o_cancel[0] - (0.0002835 - 0.0039635)) < 0.004
    assert o_lean[0] == pytest.approx(0.46 * math.sin(math.radians(2.3)) - 0.0037, abs=0.003)


def test_lidar_to_pelvis_with_waist_yaw():
    # Arrange: 扉の座標でライダーが (−0.8, 0.1) で 30° を向き、腰が 20° 左へひねっている・取付の補正 (0.05, 0)
    x, y, yaw = M.lidar_to_pelvis(-0.8, 0.1, math.radians(30.0), (math.radians(20.0), 0.0, 0.0), 0.0, 0.0, (0.05, 0.0))

    # Assert: pelvis は 10° を向き、ライダーは pelvis から見て（腰で回った）前 5 cm 付近
    assert math.degrees(yaw) == pytest.approx(10.0)
    o = M.lidar_offset_xy((math.radians(20.0), 0.0, 0.0), 0.0, 0.0, (0.05, 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    assert (x + c * o[0] - s * o[1], y + s * o[0] + c * o[1]) == pytest.approx((-0.8, 0.1))


def test_head_xy_from_center_offset_roundtrip():
    q = (0.0, math.radians(1.0), math.radians(-2.0))
    head = M.head_xy_from_center_offset((0.047, -0.016), q, math.radians(0.5), math.radians(2.0))
    o = M.lidar_offset_xy(q, math.radians(0.5), math.radians(2.0), head)
    assert tuple(o) == pytest.approx((0.047, -0.016))


@pytest.fixture()
def source():
    port = _free_port()
    src = LidarDoorSource(port, mode="track")
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    yield src, (lambda m: tx.sendto(M.pack(m), ("127.0.0.1", port)))
    src.close()


def _wait(pred, timeout=1.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.01)
    return False


def _wall_of(t_mono: float) -> float:
    """monotonic の時刻を、今の time.time() の目盛りへ（推定器が付ける電文の時刻）"""
    return time.time() - (time.monotonic() - t_mono)


ZERO = (0.0, 0.0, 0.0)


def test_source_carries_scan_pose_with_odometry(source):
    # Arrange: 点群の時刻 0.05 s 前の姿勢と、その間に odometry が前へ 2 cm 進んだ
    src, send = source
    now = time.monotonic()
    t_scan = now - 0.05
    src.note_state(t_scan, (1.0, 2.0, 0.0), ZERO, 0.0, 0.0)
    src.note_state(now, (1.02, 2.0, 0.0), ZERO, 0.0, 0.0)
    send(_msg(1, _wall_of(t_scan), x=-0.8, y=0.05, yaw=0.0))
    assert _wait(lambda: src.ready(time.monotonic()))

    # Act
    pose, why = src.pose(time.monotonic(), (1.02, 2.0, 0.0))

    # Assert: ライダー → pelvis の直し（−3.7 mm）＋ odometry の 2 cm
    assert why == ""
    assert pose.x == pytest.approx(-0.8 - (0.0002835 - 0.0039635) + 0.02, abs=1e-4)
    assert pose.y == pytest.approx(0.05, abs=1e-4)


def test_source_uses_joints_at_scan_time_not_now(source):
    # Arrange: 点群の時刻には腰の yaw が 0、今は 0.3 rad（その間 odometry の向きは変わらない＝pelvis は回っていない）
    src, send = source
    now = time.monotonic()
    t_scan = now - 0.05
    src.note_state(t_scan, (0.0, 0.0, 0.0), ZERO, 0.0, 0.0)
    src.note_state(now, (0.0, 0.0, 0.0), (0.3, 0.0, 0.0), 0.0, 0.0)
    send(_msg(1, _wall_of(t_scan), yaw=math.radians(10.0)))
    assert _wait(lambda: src.ready(time.monotonic()))

    # Act
    pose, why = src.pose(time.monotonic(), (0.0, 0.0, 0.0))

    # Assert: pelvis の向きは点群の時刻の腰（0）で直した 10°（今の腰で直すと 10° − 17.2° になる）
    assert why == ""
    assert math.degrees(pose.yaw) == pytest.approx(10.0, abs=1e-3)


def test_source_reports_stale_and_lost(source):
    src, send = source
    now = time.monotonic()
    src.note_state(now, ZERO, ZERO, 0.0, 0.0)
    send(_msg(1, _wall_of(now)))
    assert _wait(lambda: src.ready(time.monotonic()))
    # 0.6 s 後には「届かない」
    pose, why = src.pose(time.monotonic() + 0.6, ZERO)
    assert pose is None and "届かない" in why
    # 推定器が見失った（電文は届くが、受け入れない回だけが 0.5 s 以上続く）。時計は本物なので実際に待つ
    for k in range(2, 10):
        time.sleep(0.1)
        t = time.monotonic()
        src.note_state(t, ZERO, ZERO, 0.0, 0.0)
        send(_msg(k, _wall_of(t), state=0))
    assert _wait(lambda: src.snapshot()[0].seq == 9)
    pose, why = src.pose(time.monotonic(), ZERO)
    assert pose is None and "見失った" in why


def test_source_stops_when_odometry_gaps(source):
    # Arrange: 推定は届き続けるが、odometry が来ない
    src, send = source
    why = ""
    t_end = time.monotonic() + 0.6
    k = 0
    while time.monotonic() < t_end:
        t = time.monotonic()
        src.note_state(t, None, ZERO, 0.0, 0.0)
        send(_msg(k, _wall_of(t)))
        assert _wait(lambda: src.snapshot()[0] is not None and src.snapshot()[0].seq == k)
        pose, why = src.pose(time.monotonic(), None)
        k += 1
        time.sleep(0.05)

    # Assert: 0.3 s を越えたら理由つきで止める（それまでは理由なしで前の姿勢を保たせる）
    assert pose is None and "odometry" in why


def test_source_rejects_message_with_bad_clock(source):
    # Arrange: 電文の時刻が 5 s 先（時計が食い違っている）
    src, send = source
    send(_msg(1, time.time() + 5.0))
    time.sleep(0.2)

    # Assert: 受け入れない（年齢が負）
    assert src.snapshot()[0] is None and src.clock_bad == 1


def test_source_flags_disagreement_with_odometry(source):
    # Arrange: odometry は止まっているのに、ライダーの推定が 1 s で 15 cm 動いた（外れの積み重ね）
    src, send = source
    why = ""
    for k in range(12):  # 時計は本物なので 0.1 s ずつ実際に待つ
        t = time.monotonic()
        src.note_state(t, ZERO, ZERO, 0.0, 0.0)
        send(_msg(k, _wall_of(t), x=-0.8 + 0.015 * k))
        assert _wait(lambda: src.snapshot()[0] is not None and src.snapshot()[0].seq == k)
        _, why = src.pose(time.monotonic(), ZERO)
        if why:
            break
        time.sleep(0.1)

    # Assert
    assert "食い違った" in why


def test_source_angle_obs_holds_value_and_ages(source):
    src, send = source
    now = time.monotonic()
    assert src.angle_obs(now).tolist() == pytest.approx([0.0, 1.0, 0.1])  # まだ何も無い: 閉じている前提
    send(_msg(1, _wall_of(now), angle=math.radians(30.0)))
    assert _wait(lambda: src.snapshot()[0] is not None)
    a1 = src.angle_obs(now + 0.05)
    assert a1[0] == pytest.approx(math.radians(30.0)) and a1[1] == 1.0 and a1[2] == pytest.approx(0.1)
    send(_msg(2, time.time(), angle=float("nan"), state=M.STATE_OK))  # 角度は測れない回（受け取る時刻に付ける）
    assert _wait(lambda: src.snapshot()[0].seq == 2)
    a2 = src.angle_obs(now + 0.4)
    assert a2[0] == pytest.approx(math.radians(30.0)) and a2[1] == 0.0 and a2[2] == pytest.approx(0.4, abs=0.02)


def test_ref_status_reports_silent_estimator(source):
    src, _ = source
    src.request_ref()
    assert "準備中" in src.ref_status(time.monotonic())
    assert "応えない" in src.ref_status(time.monotonic() + 20.0)


def test_still_mode_carries_each_message_to_now_then_odometry():
    port = _free_port()
    src = LidarDoorSource(port, mode="still")
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Arrange: 10 回の電文。その間に機体は前へ 1 cm ずつ歩いていた（odometry も同じだけ）→ 今へ運べば全部同じ姿勢
        base = time.monotonic() - 0.95  # 電文の年齢は 1 s まで（それより古い電文は時計の食い違いとして捨てる）
        for k in range(10):
            t = base + 0.1 * k
            src.note_state(t, (5.0 + 0.01 * k, 5.0, 0.0), ZERO, 0.0, 0.0)
            tx.sendto(M.pack(_msg(k, _wall_of(t), x=-0.8 + 0.01 * k)), ("127.0.0.1", port))
        now = time.monotonic()
        src.note_state(now, (5.10, 5.0, 0.0), ZERO, 0.0, 0.0)
        assert _wait(lambda: len(src.recent) == 10)

        # Act
        sp = src.take_still(now)
        pose, why = src.pose(now, (5.40, 5.0, 0.0))

        # Assert: 今の姿勢は −0.8 + 0.10（運んだ分）、以後は odometry の 0.3 m を足すだけ
        assert sp is not None and sp.x == pytest.approx(-0.70 - (0.0002835 - 0.0039635), abs=1e-4)
        assert why == "" and pose.x == pytest.approx(sp.x + 0.3)
    finally:
        src.close()
