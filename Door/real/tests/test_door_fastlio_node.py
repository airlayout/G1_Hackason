"""door_fastlio_node（11-10 手順 L2）の試験: REF の後の止まった回で床と基準を作り、FAST-LIO の動きを扉の座標で電文にする。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import door_fastlio_node as dfn  # noqa: E402
import door_lidar_msg as msg  # noqa: E402
import lidar_door_est as est  # noqa: E402

H = 1.2  # ライダーの床からの高さ


def floor_scan(rng: np.random.Generator, n: int = 600) -> np.ndarray:
    """上下逆さまのライダー（FLIP）から見た床の点（水平・高さ H）。取付を戻すと床は z = −H"""
    xy = rng.uniform(-3, 3, size=(n, 2))
    level_pts = np.c_[xy, np.full(n, -H)]
    return level_pts @ est.FLIP  # FLIP は対角（自分の逆）: 取付の座標 = FLIP · 水平の座標


def pose_T(x: float, y: float, yaw: float) -> np.ndarray:
    """camera_init（= 段 0 のライダーの座標）で見たライダーの姿勢。水平に yaw 回って (x, y) へ動いた（取付は FLIP のまま）"""
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    T = np.eye(4)
    T[:3, :3] = est.FLIP @ Rz @ est.FLIP  # ライダーの座標で見た「水平に yaw 回る」
    T[:3, 3] = est.FLIP @ np.array([x, y, 0.0])
    return T


def feed(node: dfn.FastlioCarry, t: float, hdr: float, xyz, T: np.ndarray):
    """1 回ぶん: 点群を受け、30 ms 後に FAST-LIO の出力（ヘッダ＋1 回）を受ける"""
    out = node.on_scan(t, hdr, xyz) + node.on_odom(hdr + dfn.SCAN_PERIOD_S, T)
    return out[0] if out else None


def run_anchor(node: dfn.FastlioCarry, rng: np.random.Generator, t0: float = 100.0, hdr0: float = 5.0,
               pose=(-1.0, 0.2, math.radians(10))):
    node.request_ref(t0, pose)
    outs = []
    for k in range(40):
        t = t0 + 0.1 * k
        outs.append(feed(node, t, hdr0 + 0.1 * k, floor_scan(rng) if node.wants_points(t) else None, pose_T(0, 0, 0)))
    return outs, t0 + 0.1 * 40, hdr0 + 0.1 * 40


def test_ref_builds_anchor_after_skip_and_still_scans() -> None:
    # Arrange
    rng = np.random.default_rng(0)
    node = dfn.FastlioCarry()
    # Act: REF の後 1 秒捨て、25 回で床と基準を作る（35 回目の出力から電文）
    outs, _, _ = run_anchor(node, rng)
    # Assert
    first = next(i for i, o in enumerate(outs) if o is not None)
    assert first == 10 + dfn.STILL_SCANS - 1
    assert node.tracking
    assert node.level.tilt_deg() == pytest.approx(0.0, abs=0.5)
    m = outs[first]
    assert isinstance(m, msg.LidarMsg) and m.pose_ok and not m.angle_ok
    # 止まったままなら扉の座標でのライダーは REF の値
    assert (m.x, m.y, m.yaw) == pytest.approx((-1.0, 0.2, math.radians(10)), abs=1e-6)


def test_motion_is_carried_in_door_frame() -> None:
    # Arrange: 段 0 の後、段 0 のライダーの向きで前へ 0.3 m・左へ 0.1 m・左へ 20° 回った
    rng = np.random.default_rng(1)
    node = dfn.FastlioCarry()
    _, t, hdr = run_anchor(node, rng)
    # Act: 1 秒かけて少しずつ（跳びの見張りに掛からない 0.32 m/s・20°/s）
    for k in range(1, 11):
        m = feed(node, t + 0.1 * (k - 1), hdr + 0.1 * (k - 1), None, pose_T(0.03 * k, 0.01 * k, math.radians(2 * k)))
    # Assert: 扉の座標では、REF の向き 10° で回した動きが足される
    c, s = math.cos(math.radians(10)), math.sin(math.radians(10))
    want = (-1.0 + c * 0.3 - s * 0.1, 0.2 + s * 0.3 + c * 0.1, math.radians(30))
    assert (m.x, m.y, m.yaw) == pytest.approx(want, abs=1e-6)


def test_message_time_is_scan_end_on_receive_clock() -> None:
    rng = np.random.default_rng(2)
    node = dfn.FastlioCarry()
    _, t, hdr = run_anchor(node, rng)
    m = feed(node, t, hdr, None, pose_T(0, 0, 0))
    # 受け取り − 55 ms が走査の真ん中、FAST-LIO の姿勢は走査の終わり（＋50 ms）
    assert m.stamp == pytest.approx(t - dfn.SCAN_DELAY_S + dfn.SCAN_PERIOD_S / 2)


def test_unmatched_and_duplicate_outputs_are_dropped() -> None:
    rng = np.random.default_rng(3)
    node = dfn.FastlioCarry()
    _, t, hdr = run_anchor(node, rng)
    node.on_scan(t, hdr, None)
    assert node.on_odom(hdr + dfn.SCAN_PERIOD_S - 0.05, pose_T(0, 0, 0)) == []  # 50 ms ずれ（届いた回の間）: どの回でもない
    assert node.on_odom(hdr + dfn.SCAN_PERIOD_S, pose_T(0, 0, 0))
    assert node.on_odom(hdr + dfn.SCAN_PERIOD_S + 0.005, pose_T(0, 0, 0)) == []  # 同じ回の 2 つ目
    assert node.stats["unmatched"] == 1 and node.stats["duplicate"] == 1


def test_no_message_before_ref_and_new_ref_restarts() -> None:
    rng = np.random.default_rng(4)
    node = dfn.FastlioCarry()
    assert feed(node, 50.0, 1.0, None, pose_T(0, 0, 0)) is None
    _, t, hdr = run_anchor(node, rng)
    assert node.tracking
    node.request_ref(t, (0.0, 0.0, 0.0))
    assert not node.tracking
    assert feed(node, t, hdr, None, pose_T(0, 0, 0)) is None


def test_anchor_fails_without_enough_fastlio_outputs() -> None:
    # FAST-LIO が段 0 の回にほとんど出ていない: 電文を出さず REF を待つ
    rng = np.random.default_rng(5)
    node = dfn.FastlioCarry()
    node.request_ref(100.0, (0.0, 0.0, 0.0))
    for k in range(45):
        t = 100.0 + 0.1 * k
        node.on_scan(t, 5.0 + 0.1 * k, floor_scan(rng) if node.wants_points(t) else None)
        if k > 40:
            node.on_odom(5.0 + 0.1 * k + dfn.SCAN_PERIOD_S, pose_T(0, 0, 0))
    assert not node.tracking and node.ref_t is None
    assert "段 0" in node.last_error


@pytest.mark.parametrize("line, want", [
    (b"REF", (0.0, 0.0, 0.0)),
    (b"REF2 -1.25 0.30 0.1745", (-1.25, 0.30, 0.1745)),
    (b"REF2 1 2", None),
    (b"REF2 a b c", None),
    (b"REF2 nan 0 0", None),
    (b"HELLO", None),
])
def test_parse_ref(line: bytes, want) -> None:
    got = msg.parse_ref(line)
    if want is None:
        assert got is None
    else:
        assert got == pytest.approx(want)


def test_pack_ref_round_trips() -> None:
    assert msg.parse_ref(msg.pack_ref((-1.234567, 0.5, -0.25))) == pytest.approx((-1.234567, 0.5, -0.25), abs=1e-6)


@pytest.mark.parametrize("q_waist, roll, pitch", [((0.0, 0.0, 0.0), 0.0, 0.0), ((0.3, -0.05, 0.1), 0.03, -0.04)])
def test_pelvis_to_lidar_inverts_lidar_to_pelvis(q_waist, roll: float, pitch: float) -> None:
    lx, ly, lyaw = msg.pelvis_to_lidar(-0.9, 0.12, math.radians(-30), q_waist, roll, pitch, (0.0, 0.0))
    back = msg.lidar_to_pelvis(lx, ly, lyaw, q_waist, roll, pitch, (0.0, 0.0))
    assert back == pytest.approx((-0.9, 0.12, math.radians(-30)), abs=1e-12)


def test_pack_ref_without_pose_is_plain_ref() -> None:
    # 自前の推定器（door_lidar_node）は b"REF" そのものを待つ
    assert msg.pack_ref(None) == b"REF"
    with pytest.raises(ValueError):
        msg.pack_ref((float("nan"), 0.0, 0.0))


def test_source_request_ref_sends_pose() -> None:
    import socket
    import door_lidar_source as dls
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", port + 1))
    rx.settimeout(1.0)
    src = dls.LidarDoorSource(port=port)
    try:
        src.request_ref((-1.0, 0.25, 0.5))
        assert msg.parse_ref(rx.recvfrom(128)[0]) == pytest.approx((-1.0, 0.25, 0.5))
        src.request_ref()
        assert rx.recvfrom(128)[0] == b"REF"
    finally:
        src.close() if hasattr(src, "close") else src.rx.close()
        rx.close()


def tracking_node(seed: int = 10):
    node = dfn.FastlioCarry()
    _, t, hdr = run_anchor(node, np.random.default_rng(seed))
    assert node.tracking
    return node, t, hdr


def test_output_before_its_scan_waits_and_is_sent_on_scan() -> None:
    # FAST-LIO の出力が点群より先に届いた: 待たせ、その点群が来た時に電文を出す
    node, t, hdr = tracking_node()
    assert node.on_odom(hdr + dfn.SCAN_PERIOD_S, pose_T(0.05, 0, 0)) == []
    out = node.on_scan(t, hdr, None)
    assert len(out) == 1 and out[0].stamp == pytest.approx(t - dfn.SCAN_DELAY_S + dfn.SCAN_PERIOD_S / 2)
    assert node.stats["late_match"] == 1


def test_waiting_output_without_its_scan_is_dropped() -> None:
    node, t, hdr = tracking_node()
    node.on_odom(hdr + dfn.SCAN_PERIOD_S, pose_T(0, 0, 0))  # この回の点群は来ない
    node.on_scan(t + 0.1, hdr + 0.1, None)
    assert node.stats["unmatched"] == 1 and len(node.pending) == 0


def test_duplicate_scan_is_not_counted_twice_in_anchor() -> None:
    node = dfn.FastlioCarry()
    node.request_ref(100.0)
    rng = np.random.default_rng(11)
    xyz = floor_scan(rng)
    node.on_scan(101.0, 6.0, xyz)
    node.on_scan(101.0, 6.0, xyz)  # 同じ回が二度
    node.on_scan(101.05, 5.9, xyz)  # 古い回が遅れて
    assert len(node.anchor_xyz) == 1 and node.stats["scan_dup"] == 2


def test_older_output_after_newer_is_dropped() -> None:
    node, t, hdr = tracking_node()
    node.on_scan(t, hdr, None)
    node.on_scan(t + 0.1, hdr + 0.1, None)
    assert node.on_odom(hdr + 0.1 + dfn.SCAN_PERIOD_S, pose_T(0, 0, 0))
    assert node.on_odom(hdr + dfn.SCAN_PERIOD_S, pose_T(0, 0, 0)) == []  # 1 回前の出力が遅れて届いた
    assert node.stats["duplicate"] == 1


def test_nan_output_is_dropped_without_crash() -> None:
    node, t, hdr = tracking_node()
    T = pose_T(0, 0, 0)
    T[0, 3] = np.nan
    node.on_scan(t, hdr, None)
    assert node.on_odom(hdr + dfn.SCAN_PERIOD_S, T) == []
    assert node.stats["bad"] == 1 and node.tracking


@pytest.mark.parametrize("T_jump, word", [
    (pose_T(1.0, 0, 0), "跳んだ"),               # 0.1 s で 1 m（FAST-LIO の再起動で原点に戻ったなど）
    (pose_T(0, 0, math.radians(45)), "跳んだ"),  # 0.1 s で 45°
])
def test_jump_stops_messages_until_new_ref(T_jump: np.ndarray, word: str) -> None:
    # Arrange
    node, t, hdr = tracking_node()
    assert feed(node, t, hdr, None, pose_T(0, 0, 0)) is not None
    # Act
    assert feed(node, t + 0.1, hdr + 0.1, None, T_jump) is None
    # Assert: 以後は REF まで送らない
    assert not node.tracking and node.ref_t is None and word in node.last_error
    assert feed(node, t + 0.2, hdr + 0.2, None, pose_T(0, 0, 0)) is None


def test_vertical_rise_stops_messages() -> None:
    node, t, hdr = tracking_node()
    T = pose_T(0, 0, 0)
    T[:3, 3] = est.FLIP @ np.array([0.0, 0.0, 0.5])  # 段 0 から 0.5 m 上
    assert feed(node, t, hdr, None, T) is None
    assert "高さ" in node.last_error


def test_ref_during_anchor_restarts_collection() -> None:
    rng = np.random.default_rng(12)
    node = dfn.FastlioCarry()
    node.request_ref(100.0)
    for k in range(15):  # 1 秒の後に 5 回集めたところで
        t = 100.0 + 0.1 * k
        feed(node, t, 5.0 + 0.1 * k, floor_scan(rng) if node.wants_points(t) else None, pose_T(0, 0, 0))
    assert len(node.anchor_xyz) == 5
    node.request_ref(101.5, (0.5, 0.0, 0.0))
    assert node.anchor_xyz == [] and node.anchor_T == [] and not node.wants_points(102.0)
    assert node.wants_points(102.5)


def test_anchor_times_out_when_points_never_come() -> None:
    # 点群は来るが中身が読めない（xyz が None のまま）: REF から REF_TOTAL_S で失敗
    node = dfn.FastlioCarry()
    node.request_ref(100.0)
    k = 0
    while node.ref_t is not None and k < 200:
        node.on_scan(100.0 + 0.1 * k, 5.0 + 0.1 * k, None)
        k += 1
    assert node.ref_t is None and "そろわない" in node.last_error
    assert 100.0 + 0.1 * (k - 1) > 100.0 + dfn.REF_TOTAL_S


def test_new_ref_after_failure_recovers() -> None:
    node, t, hdr = tracking_node()
    feed(node, t, hdr, None, pose_T(0, 0, 0))
    feed(node, t + 0.1, hdr + 0.1, None, pose_T(2.0, 0, 0))  # 跳んで失敗
    assert not node.tracking
    rng = np.random.default_rng(13)
    node.request_ref(t + 1.0, (0.0, 0.0, 0.0))
    for k in range(40):
        tk = t + 1.0 + 0.1 * k
        m = feed(node, tk, hdr + 1.0 + 0.1 * k, floor_scan(rng) if node.wants_points(tk) else None, pose_T(2.0, 0, 0))
    assert node.tracking and m is not None and (m.x, m.y) == pytest.approx((0.0, 0.0), abs=1e-6)


def test_anchor_fails_when_floor_is_not_found() -> None:
    # 床の点が無い（空の点群）: 落ちずに失敗して REF を待つ
    node = dfn.FastlioCarry()
    node.request_ref(100.0)
    for k in range(40):
        t = 100.0 + 0.1 * k
        feed(node, t, 5.0 + 0.1 * k, np.zeros((0, 3)) if node.wants_points(t) else None, pose_T(0, 0, 0))
    assert not node.tracking and node.ref_t is None and "段 0" in node.last_error


def test_anchor_fails_when_robot_moves_during_anchor() -> None:
    rng = np.random.default_rng(14)
    node = dfn.FastlioCarry()
    node.request_ref(100.0)
    for k in range(40):
        t = 100.0 + 0.1 * k
        feed(node, t, 5.0 + 0.1 * k, floor_scan(rng) if node.wants_points(t) else None, pose_T(0.004 * k, 0, 0))
    assert not node.tracking and "動いた" in node.last_error


def test_yaw_wraps_across_pi() -> None:
    node = dfn.FastlioCarry()
    _, t, hdr = run_anchor(node, np.random.default_rng(15), pose=(0.0, 0.0, math.radians(170)))
    for k in range(1, 11):
        m = feed(node, t + 0.1 * (k - 1), hdr + 0.1 * (k - 1), None, pose_T(0, 0, math.radians(2 * k)))
    assert math.degrees(m.yaw) == pytest.approx(-170.0, abs=1e-6)


def test_pelvis_to_lidar_near_pi() -> None:
    q = (0.4, 0.0, 0.0)
    lx, ly, lyaw = msg.pelvis_to_lidar(0.1, -0.2, math.radians(179), q, 0.0, 0.0, (0.0, 0.0))
    assert math.degrees(lyaw) == pytest.approx(179 + math.degrees(0.4) - 360, abs=1e-9)
    back = msg.lidar_to_pelvis(lx, ly, lyaw, q, 0.0, 0.0, (0.0, 0.0))
    assert back[:2] == pytest.approx((0.1, -0.2), abs=1e-12)
    assert math.degrees(back[2]) == pytest.approx(179.0, abs=1e-9)


def test_source_drops_messages_scanned_before_ref() -> None:
    # 受け口: REF より前の走査の電文（推定器がまだ古い段 0 で追っていた分）は、REF の後に届いても捨てる
    import socket
    import time as _t
    import door_lidar_source as dls
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    src = dls.LidarDoorSource(port=port)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        src.request_ref((0.0, 0.0, 0.0))
        old = msg.LidarMsg(1, _t.time() - 0.05, 1.0, 0.0, 0.0, float("nan"), 0, msg.STATE_OK, 0.0, 0.0)
        tx.sendto(msg.pack(old), ("127.0.0.1", port))
        _t.sleep(0.3)
        new = msg.LidarMsg(2, _t.time(), 2.0, 0.0, 0.0, float("nan"), 0, msg.STATE_OK, 0.0, 0.0)
        tx.sendto(msg.pack(new), ("127.0.0.1", port))
        _t.sleep(0.3)
        got, _ = src.snapshot()
        assert src.stale == 1 and got is not None and got.seq == 2
    finally:
        tx.close()
        src.close()
