"""T3（11-11・R5）段 0 の点群での直し（door_cloud_fix）と、FAST-LIO の口（door_fastlio_node）に組み込んだ時の試験。

人工の扉（synthetic_door.scene: 板の面 x = −0.02・枠の段 3 cm・把手が左）の前の本当の姿勢にライダーを置いて点群を作り、
機体が思っている姿勢（本当から既知の直し G だけずれた REF）で扉の座標に置いて、直しで本当の姿勢に戻るかを見る。
"""
from __future__ import annotations

import math
import sys
from functools import lru_cache
from pathlib import Path
from typing import List, Tuple

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_cloud_fix as cf  # noqa: E402
import door_fastlio_node as dfn  # noqa: E402
import door_lidar_msg as msg  # noqa: E402
import lidar_door_est as est  # noqa: E402
import synthetic_door as sd  # noqa: E402

Pose = Tuple[float, float, float]
# 人工の扉の開口の幅: 点の格子（1 cm）で、板の最後の点が ±0.465・枠の最初の点が ±0.475 なので、段は ±0.470 付近（幅 0.94〜0.95）
SYN_OPENING = 0.945
POS_TOL_M = 0.005   # 依頼の線（約 5 mm・0.2°）
YAW_TOL_DEG = 0.2
LIDAR_H = 1.21
LIDAR_PITCH_DEG = 8.4  # G1 の MID-360 は前に 8.4° 傾いている（memory g1-mid360-mounted-upside-down）
TRUE_POSE: Pose = (-0.95, 0.05, math.radians(3.0))


@lru_cache(maxsize=2)
def _world(mirror: bool) -> np.ndarray:
    w = sd.scene()
    return w * np.array([1.0, -1.0, 1.0]) if mirror else w


@lru_cache(maxsize=16)
def scans_at(true_pose: Pose, mirror: bool = False, n: int = 3) -> Tuple[np.ndarray, ...]:
    """本当の姿勢に置いたライダーの点群 n 回（雑音 5 mm・回ごとに雑音を変える）"""
    return tuple(sd.lidar_scan(_world(mirror), *true_pose, h=LIDAR_H, pitch_deg=LIDAR_PITCH_DEG, noise=0.005,
                               rng=np.random.default_rng(k)) for k in range(n))


def believed_from(true_pose: Pose, G: Pose) -> Pose:
    """G ⊕ 思っている姿勢 = 本当の姿勢 になる、思っている姿勢（REF）"""
    b = est.Pose2D(*G).inverse().compose(est.Pose2D(*true_pose))
    return (b.x, b.y, b.yaw)


def door_points(true_pose: Pose, G: Pose, mirror: bool = False) -> Tuple[np.ndarray, Pose]:
    scans = scans_at(true_pose, mirror)
    believed = believed_from(true_pose, G)
    return cf.points_in_door(scans, est.level_from_scans(list(scans)), believed), believed


def assert_pose_close(got: Pose, want: Pose) -> None:
    assert abs(got[0] - want[0]) < POS_TOL_M, f"x {got[0]:+.4f} と {want[0]:+.4f}"
    assert abs(got[1] - want[1]) < POS_TOL_M, f"y {got[1]:+.4f} と {want[1]:+.4f}"
    assert abs(math.degrees(est.wrap(got[2] - want[2]))) < YAW_TOL_DEG, f"向き {math.degrees(got[2]):+.3f}° と {math.degrees(want[2]):+.3f}°"


# ── door_cloud_fix ────────────────────────────────────────────
@pytest.mark.parametrize("true_pose, G", [
    ((-0.95, 0.05, math.radians(3.0)), (0.035, -0.020, math.radians(-1.00))),   # 11-11 の 1 回目に近い（前後 +3.5・向き −1°・横 +2）
    ((-0.90, -0.10, math.radians(-5.0)), (-0.030, 0.062, math.radians(1.50))),  # 3・4 回目に近い（横 −6.2 cm）
    ((-1.10, 0.00, 0.0), (0.0, 0.0, 0.0)),                                       # ずれなし
    ((-0.80, 0.10, math.radians(8.0)), (0.070, 0.090, math.radians(2.50))),     # 上限の少し内
])
def test_recovers_known_offset_yaw_and_lateral(true_pose: Pose, G: Pose) -> None:
    # Arrange
    P, believed = door_points(true_pose, G)
    # Act
    fix = cf.measure(P, width=SYN_OPENING)
    # Assert: 直した REF が本当の姿勢に 5 mm・0.2° で戻り、両側の縁が読めて幅がそろう
    assert fix.ok and fix.lateral_ok and not fix.clamped, fix.summary()
    assert_pose_close(cf.apply_fix(believed, fix), true_pose)
    assert math.degrees(fix.dyaw) == pytest.approx(math.degrees(G[2]), abs=YAW_TOL_DEG)
    assert fix.width_m == pytest.approx(SYN_OPENING, abs=0.01)


def test_positive_slope_means_true_yaw_is_believed_plus_atan_b() -> None:
    # Arrange: 思っている扉の座標で、板が x = −0.055 + tan(1°)·y に見えた（板だけ・縁は写らない）
    b = math.tan(math.radians(1.0))
    y, z = np.meshgrid(np.arange(-0.45, 0.30, 0.01), np.arange(0.35, 1.55, 0.02))
    y, z = y.ravel(), z.ravel()
    P = np.c_[-0.055 + b * y, y, z]
    # Act
    fix = cf.measure(P, width=SYN_OPENING)
    pose = cf.apply_fix((-1.0, 0.0, 0.0), fix)
    # Assert: 本当の向き = 思っている向き + atan(b)・前後のそろえはオフラインの表と同じ定義（y = −0.08 で BOARD_X − 板）
    assert fix.ok
    assert fix.yaw_err_deg == pytest.approx(1.0, abs=1e-6)
    assert math.degrees(pose[2]) == pytest.approx(1.0, abs=1e-6)
    assert fix.shift_m == pytest.approx(cf.BOARD_X - (-0.055 + b * -0.08), abs=1e-9)
    # 縁が無いので横は直さない（dy = 0）・回した後の板の直線は x = BOARD_X に来る
    assert not fix.lateral_ok and fix.dy == 0.0 and "縁" in fix.why
    assert fix.dx + (-0.055) * math.cos(fix.dyaw) == pytest.approx(cf.BOARD_X, abs=1e-9)


def test_one_side_hidden_uses_opening_width() -> None:
    # Arrange: 把手の側の枠（y > 0.40）が写らない（11-11 の 3 回目）
    G = (0.022, 0.058, math.radians(1.56))
    P, believed = door_points(TRUE_POSE, G)
    P = P[P[:, 1] < 0.40]
    # Act
    fix = cf.measure(P, width=SYN_OPENING)
    # Assert
    assert fix.ok and fix.lateral_ok
    assert math.isnan(fix.latch_y) and math.isfinite(fix.hinge_y)
    assert "把手の側の縁が読めない" in fix.why
    assert_pose_close(cf.apply_fix(believed, fix), TRUE_POSE)


def test_no_edges_keeps_lateral_but_fixes_front_back_and_yaw() -> None:
    # Arrange: 枠が両側とも写らない
    G = (0.035, -0.02, math.radians(-1.0))
    P, believed = door_points(TRUE_POSE, G)
    P = P[np.abs(P[:, 1]) < 0.40]
    # Act
    fix = cf.measure(P, width=SYN_OPENING)
    got = cf.apply_fix(believed, fix)
    # Assert: 前後と向きだけ直る（横は直さないので、本当とは G の横の分だけ違う）
    assert fix.ok and not fix.lateral_ok and fix.dy == 0.0
    assert "両側とも読めない" in fix.why
    assert math.degrees(fix.dyaw) == pytest.approx(-1.0, abs=YAW_TOL_DEG)
    assert got[0] == pytest.approx(TRUE_POSE[0], abs=0.01)


def test_width_mismatch_skips_lateral() -> None:
    # Arrange: 開口の幅を 0.85 と渡した（本当は 0.945）。両側の縁は読めるが間が合わない
    P, _ = door_points(TRUE_POSE, (0.01, 0.02, 0.0))
    # Act
    fix = cf.measure(P, width=0.85)
    # Assert
    assert fix.ok and not fix.lateral_ok and fix.dy == 0.0
    assert "開口の幅" in fix.why


def test_too_few_board_points_does_not_fix() -> None:
    # Arrange: 扉の板の所に点が無い（扉から離れた所しか写っていない）
    rng = np.random.default_rng(0)
    P = np.c_[rng.uniform(-0.35, -0.2, 500), rng.uniform(-1.0, 1.0, 500), rng.uniform(0.3, 1.6, 500)]
    pose = (-1.0, 0.1, 0.2)
    # Act
    fix = cf.measure(P)
    # Assert
    assert not fix.ok and "板の点が足りない" in fix.why
    assert cf.apply_fix(pose, fix) == pose


def test_narrow_board_does_not_fix() -> None:
    # Arrange: 板が 10 cm の幅しか写っていない（腕で隠れた）→ 傾きが決まらない
    y, z = np.meshgrid(np.arange(-0.05, 0.05, 0.002), np.arange(0.35, 1.55, 0.01))
    P = np.c_[np.full(y.size, -0.03), y.ravel(), z.ravel()]
    # Act
    fix = cf.measure(P)
    # Assert
    assert not fix.ok and "広がり" in fix.why


def test_large_corrections_are_clamped_and_logged() -> None:
    # Arrange: 前後 10 cm・向き 4°（上限 8 cm・3° の外）
    G = (0.10, 0.0, math.radians(4.0))
    P, _ = door_points(TRUE_POSE, G)
    # Act
    fix = cf.measure(P, width=SYN_OPENING)
    # Assert: 上限で切り、切ったことを残す（測った値はそのまま残す）
    assert fix.ok
    assert fix.dx == pytest.approx(cf.LIMIT_X_M)
    assert math.degrees(fix.dyaw) == pytest.approx(cf.LIMIT_YAW_DEG)
    assert fix.yaw_err_deg == pytest.approx(4.0, abs=YAW_TOL_DEG)
    assert "前後" in fix.clamped and "向き" in fix.clamped
    assert "上限で切った" in fix.summary()


def test_lever_right_door_is_mirrored() -> None:
    # Arrange: 把手が右の扉（人工の扉を y で裏返した）
    G = (0.035, -0.02, math.radians(-1.0))
    true_pose = (-0.95, -0.05, math.radians(-3.0))
    P, believed = door_points(true_pose, G, mirror=True)
    # Act
    fix = cf.measure(P, width=SYN_OPENING, lever_right=True)
    # Assert: 蝶番は +y・把手は −y
    assert fix.ok and fix.lateral_ok, fix.summary()
    assert fix.hinge_y > 0 > fix.latch_y
    assert_pose_close(cf.apply_fix(believed, fix), true_pose)


def test_bad_input_shape_does_not_fix() -> None:
    fix = cf.measure(np.zeros((10, 2)))
    assert not fix.ok and "形" in fix.why


# ── door_fastlio_node に組み込んだ時 ─────────────────────────────
def feed(node: dfn.FastlioCarry, t: float, hdr: float, xyz) -> List[msg.LidarMsg]:
    """1 回ぶん: 点群を受け、FAST-LIO の出力（ヘッダ＋1 回・止まったまま＝単位行列）を受ける"""
    return node.on_scan(t, hdr, xyz) + node.on_odom(hdr + dfn.SCAN_PERIOD_S, np.eye(4))


def anchor_with_door(node: dfn.FastlioCarry, ref: Pose, scans, t0: float = 100.0, hdr0: float = 5.0) -> msg.LidarMsg:
    """REF の後、扉の前に止まったまま 40 回流し、最初の電文を返す"""
    node.request_ref(t0, ref)
    first = None
    for k in range(40):
        t = t0 + 0.1 * k
        out = feed(node, t, hdr0 + 0.1 * k, scans[k % len(scans)] if node.wants_points(t) else None)
        if out and first is None:
            first = out[0]
    assert first is not None, node.last_error
    return first


def test_node_default_off_keeps_ref_unchanged() -> None:
    # Arrange
    believed = believed_from(TRUE_POSE, (0.035, -0.02, math.radians(-1.0)))
    node = dfn.FastlioCarry()
    # Act
    m = anchor_with_door(node, believed, scans_at(TRUE_POSE))
    # Assert: 今までと同じ（REF の値そのまま・直しは測りもしない）
    assert node.cloud_fix == "off"
    assert (m.x, m.y, m.yaw) == pytest.approx(believed, abs=1e-9)
    assert node.fix_log == [] and node.ref_pose == pytest.approx(believed, abs=1e-12)


def test_node_on_corrects_ref_pose() -> None:
    # Arrange
    believed = believed_from(TRUE_POSE, (0.035, -0.02, math.radians(-1.0)))
    node = dfn.FastlioCarry("on", SYN_OPENING)
    # Act
    m = anchor_with_door(node, believed, scans_at(TRUE_POSE))
    # Assert: 電文は直した REF・REF の記録は受け取ったまま・直しの記録は 1 行
    assert_pose_close((m.x, m.y, m.yaw), TRUE_POSE)
    assert node.ref_log[-1][1:] == pytest.approx(believed)
    assert len(node.fix_log) == 1
    row = dict(zip(dfn.FIX_LOG_HEAD + cf.FIX_COLS, node.fix_log[0]))
    assert row["applied"] == 1.0 and row["ok"] == 1.0 and row["lateral_ok"] == 1.0
    assert (row["raw_x"], row["raw_y"], row["raw_yaw"]) == pytest.approx(believed)
    assert_pose_close((row["fix_x"], row["fix_y"], row["fix_yaw"]), TRUE_POSE)


def test_node_on_fixes_every_ref() -> None:
    # Arrange: 段 0 の 2 回の REF（巻き尺なし）。2 回目は 0.2 m 前へ出た所で、網の別のずれ
    node = dfn.FastlioCarry("on", SYN_OPENING)
    anchor_with_door(node, believed_from(TRUE_POSE, (0.035, -0.02, math.radians(-1.0))), scans_at(TRUE_POSE))
    second = (TRUE_POSE[0] + 0.2, TRUE_POSE[1], TRUE_POSE[2])
    # Act
    m = anchor_with_door(node, believed_from(second, (-0.02, 0.04, math.radians(0.8))), scans_at(second), t0=110.0, hdr0=15.0)
    # Assert: 2 回目の REF もその場の点群で直す
    assert len(node.fix_log) == 2 and all(r[7] == 1.0 for r in node.fix_log)
    assert_pose_close((m.x, m.y, m.yaw), second)


def test_node_log_mode_records_but_keeps_ref() -> None:
    # Arrange
    believed = believed_from(TRUE_POSE, (0.035, -0.02, math.radians(-1.0)))
    node = dfn.FastlioCarry("log", SYN_OPENING)
    # Act
    m = anchor_with_door(node, believed, scans_at(TRUE_POSE))
    # Assert: 電文は REF のまま・直しの値は記録に残る
    assert (m.x, m.y, m.yaw) == pytest.approx(believed, abs=1e-9)
    row = dict(zip(dfn.FIX_LOG_HEAD + cf.FIX_COLS, node.fix_log[0]))
    assert row["applied"] == 0.0 and row["ok"] == 1.0
    assert_pose_close((row["fix_x"], row["fix_y"], row["fix_yaw"]), TRUE_POSE)


def test_node_on_without_door_keeps_ref_and_logs_why() -> None:
    # Arrange: 床しか写っていない（扉の板が無い）
    rng = np.random.default_rng(3)
    floor = tuple((np.c_[rng.uniform(-3, 3, (800, 2)), np.full(800, -LIDAR_H)] @ est.FLIP) for _ in range(3))
    node = dfn.FastlioCarry("on")
    ref = (-1.0, 0.1, 0.05)
    # Act
    m = anchor_with_door(node, ref, floor)
    # Assert: 段 0 は作り、電文は REF のまま・理由を残す
    assert (m.x, m.y, m.yaw) == pytest.approx(ref, abs=1e-9)
    assert node.fix_log[0][7] == 0.0 and node.fix_log[0][8] == 0.0
    assert "板" in node.fix_why[0]


def test_node_plain_ref_is_not_fixed() -> None:
    # Arrange: "REF" だけ（扉の代わりに段 0 のライダーの真下が原点の試し）
    node = dfn.FastlioCarry("on", SYN_OPENING)
    # Act
    m = anchor_with_door(node, (0.0, 0.0, 0.0), scans_at(TRUE_POSE))
    # Assert
    assert (m.x, m.y, m.yaw) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert "立ち位置" in node.fix_why[0]


def test_node_crash_in_fix_keeps_anchor(monkeypatch: pytest.MonkeyPatch) -> None:
    # Arrange: 直しの計算が例外を出す
    def boom(*_a, **_k):
        raise FloatingPointError("試験")
    monkeypatch.setattr(dfn.cf, "measure", boom)
    believed = believed_from(TRUE_POSE, (0.035, -0.02, math.radians(-1.0)))
    node = dfn.FastlioCarry("on", SYN_OPENING)
    # Act
    m = anchor_with_door(node, believed, scans_at(TRUE_POSE))
    # Assert: 段 0 はでき、REF のまま運ぶ
    assert node.tracking
    assert (m.x, m.y, m.yaw) == pytest.approx(believed, abs=1e-9)
    assert "例外" in node.fix_why[0]


def test_node_save_writes_cloud_fix(tmp_path: Path) -> None:
    # Arrange
    believed = believed_from(TRUE_POSE, (0.035, -0.02, math.radians(-1.0)))
    node = dfn.FastlioCarry("on", SYN_OPENING)
    m = anchor_with_door(node, believed, scans_at(TRUE_POSE))
    # Act
    dfn.save(node, [m], tmp_path / "n.npz")
    z = np.load(tmp_path / "n.npz")
    # Assert
    cols = [str(c) for c in z["cloud_fix_cols"]]
    assert z["cloud_fix"].shape == (1, len(cols)) and cols[:8] == list(dfn.FIX_LOG_HEAD)
    assert str(z["cloud_fix_mode"]) == "on" and z["cloud_fix_why"].shape == (1,)
    assert z["ref_log"][0, 1:] == pytest.approx(believed)


def test_node_save_off_has_empty_cloud_fix(tmp_path: Path) -> None:
    node = dfn.FastlioCarry()
    dfn.save(node, [], tmp_path / "n.npz")
    z = np.load(tmp_path / "n.npz")
    assert z["cloud_fix"].shape == (0, len(z["cloud_fix_cols"])) and str(z["cloud_fix_mode"]) == "off"


def test_cli_defaults_keep_current_behaviour() -> None:
    a = dfn.parse_args([])
    assert (a.cloud_fix, a.door_width, a.lever_right) == ("off", None, False)
    a = dfn.parse_args(["--cloud-fix", "on", "--door-width", "0.95", "--lever-right"])
    assert (a.cloud_fix, a.door_width, a.lever_right) == ("on", 0.95, True)
    with pytest.raises(SystemExit):
        dfn.parse_args(["--cloud-fix", "yes"])


def test_node_rejects_unknown_mode_and_missing_module(monkeypatch: pytest.MonkeyPatch) -> None:
    with pytest.raises(ValueError):
        dfn.FastlioCarry("yes")
    monkeypatch.setattr(dfn, "cf", None)
    with pytest.raises(RuntimeError, match="door_cloud_fix"):
        dfn.FastlioCarry("on")
    assert dfn.FastlioCarry().cloud_fix == "off"  # 配っていなくても off なら動く
