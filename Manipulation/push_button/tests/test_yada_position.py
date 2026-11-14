"""鉛直パネルの位置補正と、撮影・押下時の姿勢の整合性。"""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import yada
from vision import ButtonTarget
from yada import PoseHistory, YadaButtonController, correct_vertical_panel, imu_rotation


@pytest.mark.parametrize("mount_tilt_deg", [-1.5, 0.0, 1.5])
@pytest.mark.parametrize("body_rpy_deg", [[0, 0, 0], [6, -3, 14]])
def test_mount_tilt_correction_recovers_position_in_tilted_body_frame(mount_tilt_deg, body_rpy_deg):
    # 正解は独立に置いた点。取付角の誤差をカメラ原点まわりに加える。
    body = Rotation.from_euler("xyz", body_rpy_deg, degrees=True)
    to_pelvis = body.as_matrix().T
    camera_world = np.array([0.0576, 0.0175, 0.4299])
    face_world = np.array([0.32, 0.025, 0.18])
    panel_yaw = np.deg2rad(2)
    normal_world = np.array([np.cos(panel_yaw), np.sin(panel_yaw), 0])
    tilt_axis = np.cross([0, 0, 1], normal_world)
    mount_error = Rotation.from_rotvec(np.deg2rad(mount_tilt_deg) * tilt_axis)
    measured_face = to_pelvis @ (camera_world + mount_error.apply(face_world - camera_world))
    measured_normal = to_pelvis @ mount_error.apply(normal_world)
    quat_xyzw = body.as_quat()
    corrected_face, corrected_normal = correct_vertical_panel(
        measured_face, measured_normal, to_pelvis @ camera_world, quat_xyzw[[3, 0, 1, 2]])
    np.testing.assert_allclose(corrected_face, to_pelvis @ face_world, atol=1e-12)
    np.testing.assert_allclose(corrected_normal, to_pelvis @ normal_world, atol=1e-12)


@pytest.mark.parametrize("quat", [[0, 0, 0, 0], [1, 0, 0], [np.nan, 0, 0, 0]])
def test_invalid_imu_cannot_calibrate_panel(quat):
    with pytest.raises(ValueError, match="IMU"):
        correct_vertical_panel(np.array([0.32, 0, 0.18]), np.array([1, 0, 0]),
                               np.array([0.05, 0, 0.43]), quat)


def test_large_panel_tilt_is_rejected():
    normal = Rotation.from_euler("y", 6, degrees=True).apply([1, 0, 0])
    with pytest.raises(ValueError, match="5度"):
        correct_vertical_panel(np.array([0.32, 0, 0.18]), normal,
                               np.array([0.05, 0, 0.43]), [1, 0, 0, 0])


def test_remeasurement_depth_hint_stays_in_original_camera_coordinates(monkeypatch):
    # 深度選択へ補正済みの点を戻すと、公称変換の画像座標と食い違う。
    controller = YadaButtonController.__new__(YadaButtonController)
    controller.detector = None
    controller.reset(SimpleNamespace(target="down", control_dt=0.02, upper_kp=np.ones(17)))
    origin = np.array([0.05, 0, 0.43])
    controller.camera_tf = SimpleNamespace(pelvis_from_optical=lambda q: (np.eye(3), origin))
    normal = Rotation.from_euler("y", 1.5, degrees=True).apply([1, 0, 0])
    faces = [np.array([0.32, 0, 0.28]), np.array([0.32, 0, 0.20])]
    hints = []
    monkeypatch.setattr(yada, "color_boxes", lambda rgb: [0, 1])

    def estimate(depth, bbox, intr, transform, *, expected_face_xyz):
        hints.append(None if expected_face_xyz is None else expected_face_xyz.copy())
        return ButtonTarget(tuple(faces[bbox]), tuple(normal), 0.3, 1, 1)

    monkeypatch.setattr(yada, "estimate_button_target", estimate)
    results = []
    for t in [0.0, 0.1, 0.2, 0.3]:
        controller.pose_history.add(t, np.zeros(3), [1, 0, 0, 0])
        results.append(controller._measure(SimpleNamespace(
            t=t, image_t=t, q=np.zeros(29), K=np.eye(3), rgb=None, depth=None,
            imu_quat=np.array([1, 0, 0, 0]))))
    assert results[0] is None and results[1] is None
    corrected_face, corrected_normal = results[2]
    assert np.linalg.norm(corrected_face - faces[1]) > 0.004
    np.testing.assert_allclose(corrected_normal, [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(controller.previous, faces[1])
    for hint in hints[-2:]:
        np.testing.assert_allclose(hint, faces[1])


def test_pose_history_interpolates_at_capture_time_and_handles_quaternion_sign():
    history = PoseHistory()
    start = Rotation.from_euler("z", 170, degrees=True)
    end = Rotation.from_euler("z", -170, degrees=True)
    history.add(1.0, [0, 0.1, 0.2], start.as_quat()[[3, 0, 1, 2]])
    history.add(1.04, [0.4, 0.2, 0.6], -end.as_quat()[[3, 0, 1, 2]])
    q, quat = history.at(1.01)
    np.testing.assert_allclose(q, [0.1, 0.125, 0.3], atol=1e-12)
    np.testing.assert_allclose(imu_rotation(quat), Rotation.from_euler("z", 175, degrees=True).as_matrix(),
                               atol=1e-12)


def test_pose_history_copies_sensor_data_and_discards_old_samples():
    history = PoseHistory()
    q, quat = np.zeros(3), np.array([1.0, 0, 0, 0])
    history.add(0.0, q, quat)
    q[0], quat[0] = 1, 0
    np.testing.assert_allclose(history.at(0.0)[0], [0, 0, 0])
    np.testing.assert_allclose(history.at(0.0)[1], [1, 0, 0, 0])
    for t in np.arange(0.02, 2.02, 0.02):
        history.add(t, [t, 0, 0], [1, 0, 0, 0])
    assert len(history.samples) < 40
    with pytest.raises(ValueError, match="履歴がありません"):
        history.at(0.0)
    with pytest.raises(ValueError, match="履歴がありません"):
        history.at(3.0)


def test_pose_history_rejects_missing_state_updates_and_reversed_times():
    history = PoseHistory()
    history.add(0.0, [0, 0, 0], [1, 0, 0, 0])
    history.add(0.2, [0, 0, 0], [1, 0, 0, 0])
    with pytest.raises(ValueError, match="100ms"):
        history.at(0.1)
    with pytest.raises(ValueError, match="逆行"):
        history.add(0.1, [0, 0, 0], [1, 0, 0, 0])
    boundary = PoseHistory()
    boundary.add(0.3, [0, 0, 0], [1, 0, 0, 0])
    boundary.add(0.4, [0.1, 0, 0], [1, 0, 0, 0])
    np.testing.assert_allclose(boundary.at(0.35)[0], [0.05, 0, 0])


def test_measurement_waits_for_capture_pose_instead_of_using_latest_pose(monkeypatch):
    controller = YadaButtonController.__new__(YadaButtonController)
    controller.detector = None
    controller.reset(SimpleNamespace(target="down", control_dt=0.02, upper_kp=np.ones(17)))
    controller.pose_history.add(0.2, [0, 0, 0], [1, 0, 0, 0])
    monkeypatch.setattr(yada, "color_boxes", lambda rgb: pytest.fail("撮影時の姿勢がない画像を採用した"))
    assert controller._measure(SimpleNamespace(t=0.2, image_t=0.1)) is None
    assert "履歴がありません" in controller.last_measurement_error
    assert controller.measurement_count == 0


def test_delayed_frames_use_capture_waist_and_stabilize_in_one_orientation_frame(monkeypatch):
    controller = YadaButtonController.__new__(YadaButtonController)
    controller.detector = None
    controller.reset(SimpleNamespace(target="down", control_dt=0.02, upper_kp=np.ones(17)))
    transforms = []
    hints = []
    controller.camera_tf = SimpleNamespace(pelvis_from_optical=lambda q: (
        Rotation.from_euler("z", q[0]).as_matrix(), np.zeros(3)))
    monkeypatch.setattr(yada, "color_boxes", lambda rgb: [0, 1])
    face_fixed = [np.array([0.32, 0.02, 0.28]), np.array([0.32, 0.02, 0.20])]
    capture_t = [0.01, 0.05, 0.09, 0.13]
    capture_yaw = np.deg2rad([2, -2, 2, -2])
    # センサ姿勢は20msごと。画像は補間が必要な時刻に撮影され、100ms遅れて届く。
    for t in np.arange(0.0, 0.28, 0.02):
        angle = np.interp(t, capture_t, capture_yaw)
        controller.pose_history.add(t, [0.4*t, 0, 0],
                                    Rotation.from_euler("z", angle).as_quat()[[3, 0, 1, 2]])

    active_capture_rotation = None

    def estimate(depth, bbox, intr, transform, *, expected_face_xyz):
        transforms.append(transform[:3, :3].copy())
        hints.append(expected_face_xyz)
        position = active_capture_rotation.T @ face_fixed[bbox]
        normal = active_capture_rotation.T @ np.array([1, 0, 0])
        return ButtonTarget(tuple(position), tuple(normal), 0.3, 1, 1)

    monkeypatch.setattr(yada, "estimate_button_target", estimate)
    result = None
    for image_t in capture_t:
        _, quat_capture = controller.pose_history.at(image_t)
        active_capture_rotation = imu_rotation(quat_capture)
        now_t = image_t + 0.1
        _, quat_now = controller.pose_history.at(now_t)
        result = controller._measure(SimpleNamespace(
            t=now_t, image_t=image_t, q=np.array([0]*12 + [0.4*now_t, 0, 0] + [0]*14),
            imu_quat=quat_now, K=np.eye(3), rgb=None, depth=None))
    assert result is not None, "姿勢で10mm以上動いて見える同じボタンを安定して計測できない"
    face, normal = result
    now_rotation = imu_rotation(quat_now)
    np.testing.assert_allclose(now_rotation @ face, face_fixed[1], atol=1e-12)
    np.testing.assert_allclose(now_rotation @ normal, [1, 0, 0], atol=1e-12)
    for i, transform in enumerate(transforms):
        expected = Rotation.from_euler("z", 0.4*capture_t[i//2]).as_matrix()
        np.testing.assert_allclose(transform, expected, atol=1e-12)
    for hint in hints[-2:]:
        np.testing.assert_allclose(hint, active_capture_rotation.T @ face_fixed[1], atol=1e-12)


def test_continuous_press_tracks_body_rotation_without_rephotographing():
    from alignment import JointCommandFilter
    from yada_press import PressCommandController

    controller = YadaButtonController.__new__(YadaButtonController)
    controller.detector = None
    controller.reset(SimpleNamespace(target="down", control_dt=0.02, upper_kp=np.ones(17)))
    controller.phase, controller.phase_t = "press", 0.0
    controller.q_home = controller.q_cmd = np.zeros(29)
    controller.face_fixed = np.array([0.32, 0.02, 0.20])
    controller.axis_fixed = np.array([1.0, 0, 0])
    controller.finger_fixed = np.array([1.0, 0, 0])
    tip_fixed = controller.face_fixed - np.array([0.010, 0, 0])
    controller.servo = PressCommandController(controller.face_fixed, controller.axis_fixed,
                                              0.008, tip_fixed)
    controller.joint_filter = JointCommandFilter(np.zeros(7))
    goals = []
    controller._gravity_offset = lambda obs: np.zeros(17)
    current_rotation = None
    controller.kin = SimpleNamespace(fk_pos=lambda q: current_rotation.T @ (
        tip_fixed + np.array([q[22], 0, 0])))

    def solve(goal):
        goals.append(current_rotation @ goal)
        q = np.zeros(29)
        q[22] = (current_rotation @ goal - tip_fixed)[0]
        return q

    controller._solve = solve
    for t, degrees in [(0.0, [0, 0, 0]), (0.02, [0.8, -0.5, 1.0]), (0.04, [-0.7, 0.8, -1.0])]:
        body = Rotation.from_euler("xyz", degrees, degrees=True)
        current_rotation = body.as_matrix()
        controller.act(SimpleNamespace(t=t, q=np.zeros(29), imu_quat=body.as_quat()[[3, 0, 1, 2]]))
        np.testing.assert_allclose(current_rotation @ controller.face, controller.face_fixed, atol=1e-12)
        assert controller.servo.penetration(current_rotation @ controller.kin.fk_pos(np.zeros(29))) == pytest.approx(-0.002)
    assert len(goals) == 3
    for goal in goals:
        np.testing.assert_allclose(goal[1:], controller.face_fixed[1:], atol=1e-12)
    assert goals[0][0] < goals[1][0] < goals[2][0]


def test_filtered_press_does_not_accumulate_unsent_motion():
    from yada_press import PressCommandController

    controller = YadaButtonController.__new__(YadaButtonController)
    controller.detector = None
    controller.reset(SimpleNamespace(target="down", control_dt=0.02, upper_kp=np.ones(17)))
    controller.phase, controller.phase_t = "press", 0.0
    controller.q_home = controller.q_cmd = np.zeros(29)
    controller.face_fixed = np.array([0.32, 0.02, 0.20])
    controller.axis_fixed = controller.finger_fixed = np.array([1.0, 0, 0])
    tip = controller.face_fixed - np.array([0.010, 0, 0])
    controller.servo = PressCommandController(controller.face_fixed, controller.axis_fixed,
                                              0.008, tip)
    controller.kin = SimpleNamespace(fk_pos=lambda q: tip + np.array([q[22], 0, 0]))
    controller._gravity_offset = lambda obs: np.zeros(17)
    attempted_m = []

    def solve(goal):
        q = np.zeros(29)
        q[22] = (goal - tip)[0]
        return q

    sent = np.zeros(7)

    def limited_joint_goal(goal, dt):
        # 関節側で要求の10%しか送れない状況を、理想的な一軸の腕で切り分ける。
        attempted_m.append(goal[0] - sent[0])
        sent[:] += 0.1 * (goal - sent)
        return sent.copy()

    controller._solve = solve
    controller.joint_filter = SimpleNamespace(next_goal=limited_joint_goal)
    for step in range(180):
        controller.act(SimpleNamespace(t=0.02*step, q=np.zeros(29), imu_quat=np.array([1, 0, 0, 0])))
    assert sum(attempted_m) > 0.01, "制限なしなら追従誤差の上限に達する前進量を要求する"
    assert 0 < sent[0] < 0.017  # 開始位置から15mm奥の指令までの距離。
    np.testing.assert_allclose(controller.servo.command_xyz, controller.kin.fk_pos(controller.q_cmd), atol=1e-12)
    np.testing.assert_allclose(controller.servo.command_xyz[1:], tip[1:], atol=1e-12)
