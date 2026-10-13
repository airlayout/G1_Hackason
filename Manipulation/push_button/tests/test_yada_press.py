"""位置誤差・接触のたわみ・画像遅延がある押下と、成功確認の境界。"""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from yada import YadaButtonController
from yada_press import ButtonLampCheck, PressCommandController, PressJointCommandFilter


def press(start=(-0.058, 0, 0), face=(0, 0, 0), depth=0.015):
    return PressCommandController(face, [1, 0, 0], 0.008, start, depth)


def test_biased_face_and_compliant_button_can_light_past_old_estimated_stroke_limit():
    # 独立な一軸の接触モデル。カメラは実際の面より4mm手前を面と推定する。
    # 手先PD=500N/m、ボタンばね=1000N/m、実ストローク=3mm、点灯=2mm。
    controller = press(face=(-0.004, 0, 0))
    tip = np.array([-0.058, 0.0, 0.0])
    crossed_old_limit = lit = False
    for _ in range(400):
        goal = controller.next_goal(tip, 0.02)
        if goal is None:
            break
        requested_depth = goal[0] + 0.008
        actual_depth = min(0.003, max(0, requested_depth) * 500 / (500 + 1000))
        tip[0] = goal[0] if requested_depth < 0 else actual_depth - 0.008
        if controller.penetration(tip) > 0.00275 and not crossed_old_limit:
            crossed_old_limit = True
            assert actual_depth < 0.002
        lit |= actual_depth >= 0.002
        assert controller.penetration(goal) <= 0.015 + 1e-12
    else:
        pytest.fail("押下と保持が終了しない")
    assert crossed_old_limit and lit
    assert controller.penetration(tip) > 0.00275
    assert actual_depth <= 0.003


def test_contact_freezes_lateral_compensation_even_if_tip_moves_back():
    controller = press()
    controller.next_goal([-0.058, 0.001, 0], 0.02)
    assert controller.correction[1] < 0
    assert controller.correction[0] == 0
    close = np.array([-0.017, 0.001, 0])  # 推定面まで球表面で9mm。
    controller.elapsed_s = 0.55 * controller.duration_s
    controller.command_xyz = close.copy()
    controller.next_goal(close, 0.02)
    frozen = controller.correction.copy()
    controller.next_goal([-0.025, 0.002, 0], 0.02)
    assert controller.correction_frozen
    np.testing.assert_allclose(controller.correction, frozen)


@pytest.mark.parametrize("tip, message", [([-0.058, 0.011, 0], "横方向"),
                                          ([-0.084, 0, 0], "法線方向"),
                                          ([np.nan, 0, 0], "手先位置")])
def test_unbounded_tracking_error_still_stops(tip, message):
    with pytest.raises(ValueError, match=message):
        press().next_goal(tip, 0.02)


@pytest.mark.parametrize("dt", [0, -0.02, 0.101, np.nan])
def test_invalid_press_timing_stops(dt):
    with pytest.raises(ValueError, match="時刻"):
        press().next_goal([-0.058, 0, 0], dt)


def test_retry_command_depth_cannot_exceed_30mm():
    with pytest.raises(ValueError, match="深さ"):
        press(depth=0.0301)


def test_joint_filter_brakes_and_settles_within_speed_and_acceleration_limits():
    filter_ = PressJointCommandFilter(np.zeros(7))
    dt = 0.02
    prior_velocity = np.zeros(7)
    positions = []
    for step in range(150):
        # 最初の動きの途中で目標が変わり、その後静止する。
        goal = np.full(7, 0.08 if step < 8 else -0.03)
        old_position = filter_.position.copy()
        current = filter_.next_goal(goal, dt)
        assert np.max(np.abs(current - old_position)) <= 0.8 * dt + 1e-12
        assert np.max(np.abs(filter_.velocity - prior_velocity)) <= 2.0 * dt + 1e-12
        prior_velocity = filter_.velocity.copy()
        positions.append(current[0])
    np.testing.assert_allclose(current, goal, atol=1e-7)
    assert min(positions) > -0.031 and max(positions) < 0.08


def image(brightness):
    rgb = np.full((20, 20, 3), 20, dtype=np.uint8)
    rgb[5:15, 5:15] = brightness
    return rgb


def test_lamp_requires_three_distinct_frames_after_retreat():
    lamp = ButtonLampCheck(image(100), (5, 5, 15, 15), 0.0)
    lamp.reset_frames(1.0)
    assert lamp.observe(image(150), 1.1, 1.0) is None  # 後退終了以前の画像。
    assert lamp.observe(image(150), 1.2, 1.1) is None
    assert lamp.observe(image(150), 1.25, 1.1) is None  # 同じフレーム。
    assert lamp.observe(image(150), 1.3, 1.2) is None
    assert lamp.observe(image(150), 1.4, 1.3) is True


@pytest.mark.parametrize("brightness, expected", [(100, False), (124, False), (125, True)])
def test_lamp_uses_target_brightness_rise(brightness, expected):
    lamp = ButtonLampCheck(image(100), (5, 5, 15, 15), 0.0)
    results = [lamp.observe(image(brightness), t, t) for t in [0.1, 0.2, 0.3]]
    assert results == [None, None, expected]


@pytest.mark.parametrize("t, image_t", [(1.0, 0.4), (0.4, 1.0), (np.nan, 0.1)])
def test_stale_or_invalid_lamp_frame_cannot_confirm_success(t, image_t):
    lamp = ButtonLampCheck(image(100), (5, 5, 15, 15), 0.0)
    with pytest.raises(ValueError, match="撮影時刻"):
        lamp.observe(image(150), t, image_t)


def checked_controller():
    controller = YadaButtonController.__new__(YadaButtonController)
    controller.detector = None
    controller.reset(SimpleNamespace(target="down", control_dt=0.02, upper_kp=np.ones(17)))
    controller.q_home = np.zeros(29)
    controller.q_cmd = np.zeros(29)
    controller.face_fixed = np.array([0.32, 0, 0.20])
    controller.axis_fixed = controller.finger_fixed = np.array([1.0, 0, 0])
    controller.kin = SimpleNamespace(fk_pos=lambda q: np.array([0.262, 0, 0.20]))
    controller._gravity_offset = lambda obs: np.zeros(17)
    controller.lamp_check = ButtonLampCheck(image(100), (5, 5, 15, 15), 0.0)
    controller._enter("check", 0.0)
    return controller


def observe(t, brightness=100, image_t=None):
    return SimpleNamespace(t=t, q=np.zeros(29), imu_quat=np.array([1, 0, 0, 0]),
                           rgb=image(brightness), image_t=t if image_t is None else image_t)


def test_unlit_button_retries_once_and_never_becomes_success_from_elapsed_time():
    controller = checked_controller()
    for t in [0.1, 0.2, 0.3]:
        _, done = controller.act(observe(t))
        assert not done and not controller.lamp_confirmed
    assert controller.phase == "press"
    assert controller.retry_count == 1
    assert controller.servo.command_depth_m == pytest.approx(0.025)
    controller._enter("check", 0.3)  # 2回目の後退が終わった状態。
    controller.lamp_check.reset_frames(0.3)
    for t in [0.4, 0.5]:
        controller.act(observe(t))
    with pytest.raises(ValueError, match="押し直し後"):
        controller.act(observe(0.6))
    assert controller.retry_count == 1 and not controller.lamp_confirmed


def test_only_lamp_confirmation_allows_return_home():
    controller = checked_controller()
    for t in [0.1, 0.2, 0.3]:
        _, done = controller.act(observe(t, 150))
        assert not done
    assert controller.lamp_confirmed and controller.phase == "home"
    assert controller.retry_count == 0


def test_tracking_guard_uses_filtered_joint_command_in_all_directions():
    controller = checked_controller()
    controller._start_press(observe(0.0), np.eye(3), np.array([0.262, 0, 0.20]))
    controller.kin.fk_pos = lambda q: np.array([0.262 + q[22], q[23], 0.20])

    def solve(goal):
        q = np.zeros(29)
        q[22:24] = goal[:2] - [0.262, 0]
        return q

    # 関節速度の制限中、要求位置に対して横方向にも差が出る。
    def filter_goal(goal, dt):
        goal = goal.copy()
        goal[1] = 0.007
        return goal

    controller._solve = solve
    controller.joint_filter.next_goal = filter_goal
    controller.act(observe(0.02))
    np.testing.assert_allclose(controller.servo.command_xyz,
                               controller.kin.fk_pos(controller.q_cmd), atol=1e-12)
    assert controller.servo.command_xyz[1] == 0.007
    assert controller.servo.end_xyz[1] == 0  # 制限中の差で押下経路自体は変更しない。


def test_missing_fresh_lamp_frames_times_out_without_success():
    controller = checked_controller()
    controller.last_t = 1.9
    controller.lamp_check.reset_frames(2.0)
    with pytest.raises(ValueError, match="新しい画像"):
        controller.act(observe(2.02, 150, image_t=1.98))
    assert not controller.lamp_confirmed
