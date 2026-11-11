"""hand_door_angle（11-10 手順 L2b）の試験: 蝶番のまわりの角度・前進量の角度・レバーを下げた判定（0100 と同じ線）・方策へ渡す形。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import door_frame as dfr  # noqa: E402
import hand_door_angle as hda  # noqa: E402

WIDTH, BACKSET = 0.94, 0.064


def rotate_about(p: np.ndarray, hinge_xy: np.ndarray, ang: float) -> np.ndarray:
    """床の上で蝶番のまわりに ang [rad] 回した点（z はそのまま）"""
    c, s = math.cos(ang), math.sin(ang)
    v = p[:2] - hinge_xy
    return np.array([hinge_xy[0] + c * v[0] - s * v[1], hinge_xy[1] + s * v[0] + c * v[1], p[2]])


@pytest.mark.parametrize("lever_left", [True, False])
def test_door_line_matches_door_frame_convention(lever_left: bool) -> None:
    # 把手が左なら蝶番は y = −幅/2、蝶番 → 戸先は +y、押す向きは +x（door_frame.py の決まり）
    line = hda.door_line(WIDTH, lever_left)
    lr = 1.0 if lever_left else -1.0
    np.testing.assert_allclose(line.hinge_xy, [0.0, -lr * WIDTH / 2])
    np.testing.assert_allclose(line.along, [0.0, lr])
    np.testing.assert_allclose(line.push, [1.0, 0.0])
    # 開く向きに回すと、戸先は +x（押す向き）へ出る
    tip = np.array([0.0, lr * WIDTH / 2, 1.0])
    opened = rotate_about(tip, line.hinge_xy, line.open_sign * math.radians(10))
    assert opened[0] > 0.0


@pytest.mark.parametrize("method", ["hinge", "advance"])
@pytest.mark.parametrize("lever_left", [True, False])
def test_angle_of_point_on_rotating_board(method: str, lever_left: bool) -> None:
    # Arrange: レバーの手前 7 cm（手のひら）に置いた点を、扉と一緒に開く向きへ回す
    line = hda.door_line(WIDTH, lever_left)
    lr = 1.0 if lever_left else -1.0
    ref = np.array([-0.07, lr * (WIDTH / 2 - BACKSET), 0.99])
    for deg in (0.5, 3.0, 8.0):
        p = rotate_about(ref, line.hinge_xy, line.open_sign * math.radians(deg))
        # Act
        got = hda.door_angle(line, ref, p, method)
        # Assert: 前進量の角度は手のひらが板の手前にある分だけ小さく外れる（3° で 0.02° 以内）
        tol = 1e-9 if method == "hinge" else 0.01 * deg
        assert math.degrees(got) == pytest.approx(deg, abs=tol)


def test_hinge_angle_ignores_sliding_along_the_board() -> None:
    # 手のひらが板に沿って蝶番の側へ 5 cm 滑っても、蝶番のまわりの角度は変わらない
    line = hda.door_line(WIDTH, True)
    ref = np.array([0.0, WIDTH / 2 - BACKSET, 0.99])
    slid = ref + np.array([0.0, -0.05, 0.0])
    p = rotate_about(slid, line.hinge_xy, line.open_sign * math.radians(3.0))
    assert math.degrees(hda.door_angle(line, ref, p, "hinge")) == pytest.approx(3.0, abs=1e-9)


def test_angle_is_clipped_at_closed() -> None:
    # 手が手前へ引いても扉は枠より閉じない → 0
    line = hda.door_line(WIDTH, True)
    ref = np.array([-0.07, WIDTH / 2 - BACKSET, 0.99])
    pulled = ref + np.array([-0.03, 0.0, 0.0])
    for method in ("hinge", "advance"):
        assert hda.door_angle(line, ref, pulled, method) == 0.0


def test_unknown_method_is_rejected() -> None:
    line = hda.door_line(WIDTH, True)
    with pytest.raises(ValueError):
        hda.door_angle(line, np.zeros(3), np.zeros(3), "lidar")


def test_press_trigger_needs_hold_steps_in_a_row() -> None:
    # Arrange: 0100 と同じ線（水平 0.10 m 以内・0.02 m 下・5 N・5 歩）
    trig = hda.PressTrigger()
    ref = np.array([0.0, 0.40, 1.00])
    trig.set_ref(ref)
    good = ref + np.array([0.03, 0.0, -0.03])
    # Act / Assert: 4 歩で途切れたら数え直し
    for _ in range(4):
        assert not trig.step(good, 6.0)
    assert not trig.step(good, 4.0)  # 力が足りない歩で途切れる
    for _ in range(4):
        assert not trig.step(good, 6.0)
    assert trig.step(good, 6.0)
    assert trig.fired


@pytest.mark.parametrize("palm_offset, force", [
    (np.array([0.11, 0.0, -0.03]), 6.0),   # 水平に遠い
    (np.array([0.0, 0.0, -0.015]), 6.0),   # 下がりが足りない
    (np.array([0.0, 0.0, -0.03]), 4.9),    # 力が足りない
])
def test_press_trigger_rejects(palm_offset: np.ndarray, force: float) -> None:
    trig = hda.PressTrigger()
    ref = np.array([0.0, 0.40, 1.00])
    for _ in range(10):
        trig.step(ref + palm_offset, force, ref)
    assert not trig.fired


def test_press_trigger_tracks_reference_only_while_closed() -> None:
    # 閉じている間に渡した基準の点だけを使い、None の歩は最後の基準を使う
    trig = hda.PressTrigger(hold=1)
    ref = np.array([0.0, 0.40, 1.00])
    trig.set_ref(ref)
    assert trig.step(ref + np.array([0.0, 0.0, -0.03]), 6.0)
    np.testing.assert_allclose(trig.ref, ref)


def test_press_trigger_without_ref_never_fires() -> None:
    trig = hda.PressTrigger(hold=1)
    assert not trig.step(np.array([0.0, 0.4, 0.9]), 9.0)


def test_estimator_obs_before_and_after_release() -> None:
    # Arrange
    line = hda.door_line(WIDTH, True)
    est = hda.HandDoorAngle(line, method="hinge", hold=2)
    ref_lever = np.array([-0.06, WIDTH / 2 - BACKSET, 0.99])
    est.set_lever_ref(ref_lever)
    palm = ref_lever + np.array([0.0, 0.0, -0.03])
    # Act / Assert: 掛け金が外れるまでは閉じた扉の値（シムの初めの値と同じ形）
    np.testing.assert_allclose(est.step(0.00, palm, 6.0), dfr.door_closed_angle_obs())
    assert not est.released
    est.step(0.02, palm, 6.0)
    assert est.released and est.release_t == pytest.approx(0.02)
    # 外れた時の手のひらが基準。そこから扉と一緒に 3° 回すと 3°
    opened = rotate_about(palm, line.hinge_xy, line.open_sign * math.radians(3.0))
    obs = est.step(0.20, opened, 6.0)
    assert math.degrees(obs[0]) == pytest.approx(3.0, abs=1e-9)
    assert obs[1] == 1.0
    assert obs[2] == pytest.approx(hda.LATENCY_S / hda.MAX_AGE_S)
    assert est.max_deg == pytest.approx(3.0, abs=1e-9)


def test_palm_world_matches_fk_with_identity_pose() -> None:
    # 姿勢が原点・向き 0 なら、世界の手のひら = pelvis の座標の順運動学
    import g1_arm_fk as fk
    chain = fk.load_chain()
    q = np.zeros(29)
    q[15], q[18] = -0.8, 0.6  # 左肩 pitch・左肘
    want = fk.palm_in_pelvis(chain, q, "left")
    got = hda.palm_world(chain, q, np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), "left")
    np.testing.assert_allclose(got, want, atol=1e-12)
    # 向きを 90° 回すと x → y
    s = math.sin(math.pi / 4)
    got90 = hda.palm_world(chain, q, np.array([1.0, 2.0, 0.8]), np.array([0.0, 0.0, s, s]), "left")
    np.testing.assert_allclose(got90, [1.0 - want[1], 2.0 + want[0], 0.8 + want[2]], atol=1e-9)


def test_palm_in_door_frame_uses_planar_pose_and_tilt() -> None:
    # 実機の口: 扉の座標の床の姿勢（FAST-LIO で運んだ値）・IMU の傾き・pelvis の高さ
    import g1_arm_fk as fk
    chain = fk.load_chain()
    q = np.zeros(29)
    q[15] = -1.0
    pose = dfr.PlanarPose(-0.43, 0.10, math.radians(-45))
    imu = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz・水平
    got = hda.palm_in_door_frame(chain, q, pose, imu, 0.78, "left")
    want = hda.palm_world(chain, q, np.array([-0.43, 0.10, 0.78]),
                          np.array([0.0, 0.0, math.sin(math.radians(-22.5)), math.cos(math.radians(-22.5))]), "left")
    np.testing.assert_allclose(got, want, atol=1e-9)


def test_contact_ref_fires_without_drop_but_press_does_not() -> None:
    # 手のひらがレバーの当てる点の高さのまま力をかけた: 触れた（contact）は立ち、0100 の引き金（press）は立たない
    line = hda.door_line(WIDTH, True)
    lever = np.array([-0.06, WIDTH / 2 - BACKSET, 0.99])
    for mode, want in (("contact", True), ("press", False)):
        est = hda.HandDoorAngle(line, ref_mode=mode)
        est.set_lever_ref(lever)
        for i in range(6):
            est.step(0.02 * i, lever + np.array([0.01, 0.0, 0.0]), 8.0)
        assert est.released is want, mode


def test_unknown_ref_mode_is_rejected() -> None:
    with pytest.raises(ValueError):
        hda.HandDoorAngle(hda.door_line(WIDTH, True), ref_mode="lever_true")


def make_released(hold: int = 2, lost_s: float = 0.3):
    """触れて基準が決まった推定（t = 0.00・0.02 の 2 歩で立つ）と、その時の手のひら"""
    line = hda.door_line(WIDTH, True)
    est = hda.HandDoorAngle(line, hold=hold, lost_s=lost_s)
    lever = np.array([-0.06, WIDTH / 2 - BACKSET, 0.99])
    est.set_lever_ref(lever)
    palm = lever + np.array([0.0, 0.0, -0.01])
    est.step(0.00, palm, 6.0)
    est.step(0.02, palm, 6.0)
    assert est.released
    return line, est, palm


def test_reference_is_first_step_of_the_run() -> None:
    # 立つまでの hold 歩の間に手が 1 cm 前へ出ても、基準は続きの最初の歩の手のひら
    line = hda.door_line(WIDTH, True)
    est = hda.HandDoorAngle(line, hold=3)
    lever = np.array([-0.06, WIDTH / 2 - BACKSET, 0.99])
    est.set_lever_ref(lever)
    first = lever.copy()
    for i, dx in enumerate((0.0, 0.005, 0.01)):
        est.step(0.02 * i, first + np.array([dx, 0.0, 0.0]), 6.0)
    np.testing.assert_allclose(est.palm_ref, first)


def test_hand_leaving_holds_angle_and_marks_invalid() -> None:
    # Arrange
    line, est, palm = make_released()
    opened = rotate_about(palm, line.hinge_xy, line.open_sign * math.radians(2.0))
    est.step(0.10, opened, 6.0)
    # Act: 力が 0.3 s を越えて切れた後、手が大きく前へ出ても角度は 2° のまま・測れない・古さが増える
    est.step(0.30, opened, 0.0)
    far = rotate_about(palm, line.hinge_xy, line.open_sign * math.radians(9.0))
    obs = est.step(0.50, far, 0.0)
    # Assert
    assert est.lost
    assert math.degrees(obs[0]) == pytest.approx(2.0, abs=1e-9)
    assert obs[1] == 0.0
    assert obs[2] == pytest.approx(0.2 / hda.MAX_AGE_S)  # 最後に更新した 0.30 s（猶予の内）から
    # 触れ直しても戻さない
    obs = est.step(0.60, far, 9.0)
    assert obs[1] == 0.0 and math.degrees(obs[0]) == pytest.approx(2.0, abs=1e-9)
    assert est.max_deg == pytest.approx(2.0, abs=1e-9)


def test_short_force_drop_does_not_mark_lost() -> None:
    line, est, palm = make_released()
    est.step(0.10, palm, 0.0)
    est.step(0.30, palm, 0.0)  # 0.28 s しか切れていない
    obs = est.step(0.32, palm, 6.0)
    assert not est.lost and obs[1] == 1.0


def test_nan_palm_is_invalid_for_that_step_only() -> None:
    line, est, palm = make_released()
    obs = est.step(0.10, np.array([np.nan, 0.0, 0.0]), 6.0)
    assert obs[1] == 0.0 and math.isfinite(obs[0])
    obs = est.step(0.12, palm, 6.0)
    assert obs[1] == 1.0


def test_nan_before_release_does_not_fire_and_resets_count() -> None:
    line = hda.door_line(WIDTH, True)
    est = hda.HandDoorAngle(line, hold=2)
    lever = np.array([-0.06, WIDTH / 2 - BACKSET, 0.99])
    est.set_lever_ref(lever)
    est.step(0.00, lever, 6.0)
    est.step(0.02, np.full(3, np.nan), 6.0)
    est.step(0.04, lever, 6.0)
    assert not est.released


def test_brief_touch_does_not_set_reference() -> None:
    # 寄る途中で 4 歩だけ触れて離れた: 基準は決まらない
    line = hda.door_line(WIDTH, True)
    est = hda.HandDoorAngle(line)
    lever = np.array([-0.06, WIDTH / 2 - BACKSET, 0.99])
    est.set_lever_ref(lever)
    for i in range(4):
        est.step(0.02 * i, lever, 8.0)
    est.step(0.08, lever, 1.0)
    assert not est.released


def test_set_lever_ref_after_release_changes_nothing() -> None:
    line, est, palm = make_released()
    before = est.palm_ref.copy()
    est.set_lever_ref(palm + np.array([0.3, 0.3, 0.3]))
    opened = rotate_about(palm, line.hinge_xy, line.open_sign * math.radians(3.0))
    obs = est.step(0.10, opened, 6.0)
    np.testing.assert_allclose(est.palm_ref, before)
    assert math.degrees(obs[0]) == pytest.approx(3.0, abs=1e-9)


def test_bad_trigger_settings_raise_value_error() -> None:
    with pytest.raises(ValueError):
        hda.PressTrigger(hold=0)
    with pytest.raises(ValueError):
        hda.PressTrigger().set_ref(np.array([np.nan, 0.0, 0.0]))


def test_signed_angle_keeps_closing_direction() -> None:
    line = hda.door_line(WIDTH, True)
    ref = np.array([-0.07, WIDTH / 2 - BACKSET, 0.99])
    closing = rotate_about(ref, line.hinge_xy, -line.open_sign * math.radians(1.0))
    assert math.degrees(hda.door_angle_signed(line, ref, closing)) == pytest.approx(-1.0, abs=1e-9)
    assert hda.door_angle(line, ref, closing) == 0.0
