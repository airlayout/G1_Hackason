"""位置合わせの異常系と、実機へ進むための条件を検証する。"""

import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "real"))

from alignment import (AlignmentController, CorrectionCadence, CorrectionTiming,
                       ContinuousNormalController, FixedTargetAlignment, JointCommandFilter,
                       NormalMotionController, Observation, SingleCorrectionTiming,
                       TargetTracker, align_fixed_target, run_continuous_normal)
import align_button
from align_button import ArmMotion, read_calibration, validate_stationary_pose
from vision import ButtonTarget


def button(x=0.35, y=-0.25, z=1.0, normal=(1.0, 0.0, 0.0)):
    return ButtonTarget((x, y, z), normal, 0.5, 1.0, 1.0)


def test_never_uses_old_future_or_reused_images():
    controller = AlignmentController()
    tip = np.array([0.30, -0.25, 1.0])
    with pytest.raises(ValueError, match="古い"):
        controller.next_step(Observation(button(), 0.0, 1), tip, 1.0)
    with pytest.raises(ValueError, match="時刻"):
        controller.next_step(Observation(button(), 2.0, 1), tip, 1.0)
    controller.next_step(Observation(button(), 1.0, 1), tip, 1.0)
    with pytest.raises(ValueError, match="再利用"):
        controller.next_step(Observation(button(), 1.0, 1), tip, 1.1)


def test_lateral_alignment_keeps_distance_and_needs_three_fresh_frames():
    controller = AlignmentController()
    tip = np.array([0.28, -0.28, 1.0])
    step = controller.next_step(Observation(button(), 0, 1), tip, 0)
    assert step.tip_goal[0] == tip[0]
    assert np.linalg.norm(step.tip_goal-tip) <= 0.005 + 1e-12
    controller = AlignmentController()
    aligned = np.array([0.305, -0.25, 1.0])
    for sequence in (1, 2):
        assert not controller.next_step(Observation(button(), sequence, sequence), aligned, sequence).converged
    assert controller.next_step(Observation(button(), 3, 3), aligned, 3).converged


def test_cannot_correct_when_already_touching_or_unconverged():
    with pytest.raises(ValueError, match="接触"):
        AlignmentController().next_step(Observation(button(), 0, 1), np.array([0.334, -0.25, 1]), 0)
    controller = AlignmentController(max_iterations=1)
    tip = np.array([0.28, -0.25, 1])
    controller.next_step(Observation(button(), 0, 1), tip, 0)
    with pytest.raises(ValueError, match="収束"):
        controller.next_step(Observation(button(), 1, 2), tip, 1)


def test_tracker_rejects_ambiguity_and_other_button_instead_of_switching():
    tracker = TargetTracker()
    with pytest.raises(ValueError, match="一意"):
        tracker.update([button(), button(z=0.86)], 0)
    tracker.update([button()], 0)
    tracker.update([button(), button(z=0.86)], 1)
    assert tracker.update([button(), button(z=0.86)], 2).target.face_xyz == button().face_xyz
    with pytest.raises(ValueError, match="一意"):
        tracker.update([button(z=0.86)], 3)
    assert not tracker.samples


def test_unstable_depth_never_produces_target():
    tracker = TargetTracker()
    tracker.update([button()], 0)
    tracker.update([button(x=0.36)], 1)
    assert tracker.update([button(x=0.35)], 2) is None


def test_new_measurement_needs_three_post_wait_frames_and_keeps_target_identity():
    tracker = TargetTracker()
    for stamp in (0.0, 0.1, 0.2):
        observation = tracker.update([button()], stamp)
    tracker.begin_measurement()
    assert tracker.update([button()], 2.0) is None
    assert tracker.update([button()], 2.1) is None
    fresh = tracker.update([button()], 2.2)
    assert fresh.sequence > observation.sequence
    assert fresh.captured_at == 2.2
    tracker.begin_measurement()
    with pytest.raises(ValueError, match="一意"):
        tracker.update([button(z=0.86)], 4.0)


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value

    def wait(self, duration):
        self.value += duration


def test_cadence_spaces_actual_motion_starts_despite_variable_perception_and_ik():
    clock = FakeClock()
    cadence = CorrectionCadence(CorrectionTiming(), clock.now, clock.wait)
    starts = []
    for processing in (0.2, 0.5, 3.0, 0.1):
        cadence.wait_for_next()
        clock.wait(processing)
        starts.append(cadence.start())
        clock.wait(0.6 + 0.6)
    assert min(np.diff(starts)) >= 2.0 - 1e-12
    assert starts[2] - starts[1] > 2.0  # 推論が遅れた分を連続補正で取り戻さない。
    assert starts[3] - starts[2] >= 2.0


def test_cadence_stops_before_waiting_past_phase_deadline():
    clock = FakeClock()
    cadence = CorrectionCadence(CorrectionTiming(), clock.now, clock.wait)
    clock.wait(59.0)
    cadence.start()
    with pytest.raises(ValueError, match="制限時間"):
        cadence.wait_for_next()
    assert clock.now() == 59.0
    clock.wait(1.0)
    with pytest.raises(ValueError, match="制限時間"):
        cadence.start()


@pytest.mark.parametrize("changes", [dict(interval_s=1.0), dict(interval_s=float("nan")),
                                    dict(settle_s=0.0), dict(phase_timeout_s=2.0)])
def test_impossible_or_nonfinite_correction_timing_is_rejected(changes):
    with pytest.raises(ValueError, match="補正間隔"):
        CorrectionTiming(**changes)


@pytest.mark.parametrize("mode,freeze_retarget,normal_motion,stroke", [("iterative", False, "stepped", 0.008),
    ("single", False, "stepped", 0.008), ("single", False, "smooth", 0.008),
    ("single", True, "smooth", 0.008), ("single", False, "smooth", 0.0015),
    ("single", True, "smooth", 0.0015)])
def test_live_alignment_measurement_policy_and_failed_retarget_abort(monkeypatch, tmp_path, mode, freeze_retarget, normal_motion, stroke):
    clock = FakeClock()
    kin = SimpleNamespace(position=np.array([0.1, -0.3, 1.0]))
    kin.update_fixed_base = lambda *args: None
    kin.tip_position = lambda: kin.position.copy()
    kin.solve = lambda xyz, normal, **kwargs: SimpleNamespace(joint_angles=np.r_[xyz, np.zeros(4)])
    kin.validate_path = lambda *args: None
    movements = []
    captures = []

    class FakeMotion:
        release_error = None
        home = (0.0,)*7

        def __init__(self, arm, calibration):
            pass

        def check(self):
            pass

        def enable(self):
            pass

        def wait(self, duration):
            clock.wait(duration)

        def move(self, goal, duration, settle_s=0.1):
            movements.append((clock.now(), duration, settle_s))
            if not freeze_retarget or len(movements) < 3:
                kin.position = np.array(goal[:3])
            clock.wait(duration + settle_s)

        def close(self):
            pass

        def current_target(self):
            return np.r_[kin.position, np.zeros(4)]

        def stream(self, goal):
            kin.position = np.array(goal[:3])

    class FakeCamera:
        def observe(self):
            # 撮影は間隔待ちの後。画像はその場で取得し、古い画像を返さない。
            captures.append(clock.now())
            clock.wait(0.2)
            target = button(y=-0.25 if len(captures) == 1 else -0.23)
            return Observation(target, clock.now(), len(captures))

    arm = SimpleNamespace(current_configuration=lambda: ((0.0,)*29, (0, 0, 0), clock.now(), 0, 0))
    calibration = {"tip_radius_m": 0.015, "reference_waist_q_rad": [0, 0, 0],
                   "reference_imu_rpy_rad": [0, 0, 0]}
    args = SimpleNamespace(clearance=0.05, stroke=stroke, output=tmp_path / "live.json",
                           alignment_mode=mode,
                           normal_motion=normal_motion,
                           correction_interval=2.0, correction_duration=0.6,
                           settle_time=0.6, phase_timeout=60.0)
    monkeypatch.setattr(align_button, "ArmMotion", FakeMotion)
    monkeypatch.setattr(align_button, "time", SimpleNamespace(monotonic=clock.now))
    if freeze_retarget:
        with pytest.raises(ValueError, match="1cm"):
            align_button.execute_alignment(arm, FakeCamera(), kin, calibration, args)
        report = json.loads(args.output.read_text())
        assert len(captures) == report["measurement_count"] == 2
        assert not report["completed"] and not report["normal_steps"]
        return
    report = align_button.execute_alignment(arm, FakeCamera(), kin, calibration, args)
    assert report["completed"]
    assert report["physical_press_success"] is None
    if mode == "single":
        assert len(captures) == report["measurement_count"] == 2
        assert report["remeasurement_count"] == 1
        assert len(report["alignment_steps"]) == 1
        assert captures[1] >= 0.2 + 4.0 + 0.1 + 1.0 + 0.3 - 1e-12
        assert np.allclose(report["alignment_steps"][0]["target_xyz_m"], button(y=-0.23).face_xyz)
        for phase in ("contact", "press"):
            starts = [s["motion_started_s"] for s in report["normal_steps"] if s["phase"] == phase]
            assert max(np.diff(starts)) < 2.0
        assert report["final_alignment_error_m"] <= 0.003
        if normal_motion == "smooth":
            assert report["normal_motion"] == "smooth"
            starts = [s["time_s"] for s in report["normal_steps"]]
            assert max(np.diff(starts)) == pytest.approx(0.02)
        return
    alignment_moves = [s["motion_started_s"] for s in report["alignment_steps"]
                       if s["motion_started_s"] is not None]
    assert len(alignment_moves) >= 3
    assert min(np.diff(alignment_moves)) >= 2.0 - 1e-12
    for phase in ("contact", "press"):
        starts = [s["motion_started_s"] for s in report["normal_steps"] if s["phase"] == phase]
        assert len(starts) >= 2
        assert min(np.diff(starts)) >= 2.0 - 1e-12
    for previous, capture in zip(report["alignment_steps"][:-1], captures[2:]):
        previous_start = previous["motion_started_s"] or previous["time_s"]
        assert capture >= previous_start + 2.0 - 1e-12
    assert all(wait == 0.6 for _, duration, wait in movements if duration == 0.6)


def test_single_retarget_rejects_stale_image_large_shift_and_close_contact():
    obs, tip = Observation(button(), 0.0, 3), np.array([0.28, -0.25, 1.0])
    with pytest.raises(ValueError, match="古い"):
        FixedTargetAlignment(obs, tip, 0.015, 0.05, 1.0)
    with pytest.raises(ValueError, match="8cm"):
        FixedTargetAlignment(obs, tip-np.array([0, 0.1, 0]), 0.015, 0.05, 0.0)
    with pytest.raises(ValueError, match="接触"):
        FixedTargetAlignment(obs, np.array([0.334, -0.25, 1]), 0.015, 0.05, 0.0)


def test_single_target_remains_fixed_after_image_ages_and_bounds_fk_compensation():
    obs, tip = Observation(button(), 0.0, 3), np.array([0.28, -0.25, 1.0])
    servo = FixedTargetAlignment(obs, tip, 0.015, 0.05, 0.0)
    goal = servo.goal.copy()
    # 画像はこの後撮り直さず、止まったボタンへのFK追従だけを確認する。
    measured = goal-np.array([0, 0, 0.004])
    command = servo.next_goal(measured)
    assert np.array_equal(servo.goal, goal)
    assert np.linalg.norm(command-goal) <= 0.005
    with pytest.raises(ValueError, match="1cm"):
        for _ in range(5):
            servo.next_goal(measured)


def test_single_failure_aborts_at_deadline_without_requesting_another_image():
    clock = FakeClock()
    obs = Observation(button(), 0.0, 3)
    goal = np.array([0.285, -0.25, 1.0])
    attempts = []

    def move(xyz, duration, settle):
        attempts.append(xyz)
        clock.wait(duration+settle)

    with pytest.raises(ValueError, match="制限時間"):
        align_fixed_target(obs, 0.015, 0.05, SingleCorrectionTiming(phase_timeout_s=1.1),
                           clock.now, clock.wait, lambda: goal.copy(), move)
    assert len(attempts) == 1


def test_normal_motion_freezes_lateral_position_and_limits_overtravel():
    face = np.array([0.35, -0.25, 1])
    normal = np.array([1, 0, 0])
    command = face + (-0.015+0.008)*normal
    servo = NormalMotionController(face, normal, 0.015, 0.008, command)
    goal = servo.next_goal(command-0.002*normal)
    assert np.allclose(goal[1:], command[1:])
    with pytest.raises(ValueError, match="押込量"):
        servo.next_goal(command+0.002*normal)


def test_normal_motion_rejects_windup():
    face = np.array([0.35, -0.25, 1])
    normal = np.array([1, 0, 0])
    command = face - 0.007*normal
    servo = NormalMotionController(face, normal, 0.015, 0.008, command)
    with pytest.raises(ValueError, match="4mm"):
        for _ in range(10):
            servo.next_goal(command-0.003*normal)


def test_press_completion_band_does_not_relax_contact_gap_or_overtravel_limit():
    face = np.array([0.35, -0.25, 1])
    normal = np.array([1, 0, 0])
    radius = 0.015
    command = face + (0.008-radius)*normal
    press = NormalMotionController(face, normal, radius, 0.008, command)
    assert press.next_goal(command-0.0008*normal) is None
    with pytest.raises(ValueError, match="押込量"):
        press.next_goal(command+0.0011*normal)
    contact = NormalMotionController(face, normal, radius, -0.003, face-(radius+0.003)*normal)
    assert contact.next_goal(contact.command_xyz-0.0008*normal) is not None
    short_command = face + (0.002-radius)*normal
    short_press = NormalMotionController(face, normal, radius, 0.002, short_command)
    short_goal = short_press.next_goal(short_command-0.0008*normal)
    assert short_goal is not None
    assert float((short_goal-short_command) @ normal) <= 0.00025 + 1e-12
    assert short_press.completion_tolerance_m == 0.00025


def test_last_compensation_step_is_clipped_to_existing_four_mm_bound():
    face, normal, radius = np.array([0.35, -0.25, 1]), np.array([1, 0, 0]), 0.015
    command = face + (0.008-radius+0.0039)*normal
    servo = NormalMotionController(face, normal, radius, 0.008, command)
    stuck_tip = face + (0.008-radius-0.0012)*normal
    goal = servo.next_goal(stuck_tip)
    command_depth = float((goal-face) @ normal + radius)
    assert command_depth <= 0.012 + 1e-12
    assert goal[0] > command[0]
    assert np.array_equal(goal[1:], command[1:])
    with pytest.raises(ValueError, match="4mm"):
        servo.next_goal(stuck_tip)


def test_single_press_uses_fine_completion_check_without_tiny_transport_steps():
    face, normal, radius = np.array([0.35, -0.25, 1]), np.array([1, 0, 0]), 0.015
    command = face + (0.008-radius)*normal
    servo = NormalMotionController(face, normal, radius, 0.008, command, single=True)
    assert servo.completion_tolerance_m == 0.0001
    goal = servo.next_goal(command-0.0008*normal)
    assert float((goal-command) @ normal) == pytest.approx(0.0005)
    with pytest.raises(ValueError, match="押込量"):
        servo.next_goal(command+0.0011*normal)


@pytest.mark.parametrize("stroke", [0.0015, 0.008])
def test_continuous_approach_has_no_intermediate_stop_and_preserves_speed_acceleration_limits(stroke):
    clock = FakeClock()
    face, normal = np.array([0.35, -0.25, 1.0]), np.array([1.0, 0, 0])
    tip = face-(0.015+0.05)*normal
    servo = ContinuousNormalController(face, normal, 0.015, stroke, tip)
    steps, waits = [], []

    def send(goal, dt, phase):
        tip[:] = goal
        return np.zeros(7)

    def wait(dt):
        waits.append(dt)
        clock.wait(dt)

    result = run_continuous_normal(servo, clock.now, wait, lambda: tip.copy(), send, 20.0, steps.append)
    assert abs(result["final_penetration_m"]-stroke) <= servo.completion_tolerance_m
    assert all(dt == pytest.approx(0.02) for dt in waits)
    velocity = np.array([s["command_speed_m_s"] for s in steps])
    assert np.max(np.abs(velocity)) <= 0.025 + 1e-12
    assert np.max(np.abs(np.diff(np.r_[0, velocity]))) <= 0.04*0.02 + 1e-12
    moving = [s for s in steps if servo.penetration(s["tip_xyz_m"]) < stroke-0.001]
    assert all(s["command_speed_m_s"] > 0 for s in moving)
    switch = next(i for i, s in enumerate(steps) if s["phase"] == "press")
    assert velocity[switch-1] > 0 and velocity[switch] > 0
    assert np.allclose(np.array([s["tip_goal_xyz_m"] for s in steps])[:, 1:], face[1:])


def test_short_stroke_limits_actual_overtravel_separately_from_tracking_compensation():
    face, normal, radius = np.array([0.35, -0.25, 1.0]), np.array([1.0, 0, 0]), 0.015
    goal = face+(0.0015-radius)*normal
    servo = ContinuousNormalController(face, normal, radius, 0.0015, goal)
    assert servo.completion_tolerance_m == pytest.approx(0.00005)
    with pytest.raises(ValueError, match="押込量"):
        servo.next_goal(goal+0.00016*normal, 0.02)
    # 指令の余裕は実際の押込量に加える値ではない。FKが追従しなければ中止する。
    servo = ContinuousNormalController(face, normal, radius, 0.0015, goal+0.003*normal)
    with pytest.raises(ValueError, match="3mm"):
        servo.next_goal(goal-0.0001*normal, 0.02)


@pytest.mark.parametrize("depth,speed", [(-0.005, 0.005), (-0.001, 0.0016), (0, 0.00075)])
def test_short_stroke_only_slows_to_press_speed_near_contact(depth, speed):
    face, normal, radius = np.array([0.35, -0.25, 1.0]), np.array([1.0, 0, 0]), 0.015
    tip = face+(depth-radius)*normal
    servo = ContinuousNormalController(face, normal, radius, 0.0015, tip)
    for _ in range(10):
        servo.next_goal(tip, 0.02)
    assert servo.velocity == pytest.approx(speed)


def test_continuous_motion_aborts_on_overtravel_stale_cycle_and_windup():
    face, normal, radius = np.array([0.35, -0.25, 1.0]), np.array([1.0, 0, 0]), 0.015
    goal = face+(0.008-radius)*normal
    servo = ContinuousNormalController(face, normal, radius, 0.008, goal)
    with pytest.raises(ValueError, match="押込量"):
        servo.next_goal(goal+0.0011*normal, 0.02)
    with pytest.raises(ValueError, match="状態更新"):
        servo.next_goal(goal, 0.11)
    servo = ContinuousNormalController(face, normal, radius, 0.008, goal+0.004*normal)
    with pytest.raises(ValueError, match="4mm"):
        servo.next_goal(goal-0.0005*normal, 0.02)


def test_continuous_motion_stops_at_deadline_when_tip_does_not_follow():
    clock = FakeClock()
    face, normal = np.array([0.35, -0.25, 1.0]), np.array([1.0, 0, 0])
    tip = face+(0.008-0.015-0.0002)*normal
    servo = ContinuousNormalController(face, normal, 0.015, 0.008, tip)
    steps = []
    with pytest.raises(ValueError, match="制限時間"):
        run_continuous_normal(servo, clock.now, clock.wait, lambda: tip.copy(),
                              lambda *args: np.zeros(7), 0.10, steps.append)
    assert clock.now() == pytest.approx(0.10)
    assert len(steps) == 5


def test_joint_stream_caps_velocity_acceleration_when_ik_goal_jumps():
    initial = np.array([0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0])
    stream = JointCommandFilter(initial)
    outputs = [initial.copy()]
    for goal in [initial+0.3]*60+[initial-0.2]*60:
        outputs.append(stream.next_goal(goal, 0.02))
    velocities = np.diff(outputs, axis=0)/0.02
    assert np.max(np.abs(velocities)) <= 0.8 + 1e-12
    assert np.max(np.abs(np.diff(np.vstack([np.zeros(7), velocities]), axis=0))/0.02) <= 2.0 + 1e-10


def test_calibration_and_moving_body_block_execution(tmp_path):
    payload = {"schema": "g1-button-calibration-v1", "T_base_optical": np.eye(4).tolist(),
               "camera_body": "torso_link",
               "reference_waist_q_rad": [0, 0, 0], "reference_imu_rpy_rad": [0, 0, 0],
               "tip_radius_m": 0.015, "button_radius_m": 0.032, "position_uncertainty_m": 0.004}
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(payload))
    calibration, _ = read_calibration(path)
    q = [0.0]*29
    state = (q, (0, 0, 0), time.monotonic(), 0, 0)
    validate_stationary_pose(state, calibration)
    with pytest.raises(RuntimeError, match="動いて"):
        validate_stationary_pose((q, (0, 0, 0), time.monotonic(), 0.3, 0), calibration)
    q[12] = 0.1
    with pytest.raises(RuntimeError, match="腰姿勢"):
        validate_stationary_pose((q, (0, 0, 0), time.monotonic(), 0, 0), calibration)
    payload["T_base_optical"][0][0] = 2
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="行列"):
        read_calibration(path)


def test_control_keeps_publishing_while_perception_is_blocked_and_releases_on_error():
    class FakeArm:
        writes = 0
        releases = 0
        moving = False

        def current_arm(self):
            return (0.0,)*14

        def set_hold(self, current):
            pass

        def current_configuration(self):
            return ((0.0,)*29, (0, 0, 0), time.monotonic(), 0.3 if self.moving else 0, 0)

        def publish(self, target, weight):
            self.writes += 1

        def release(self):
            self.releases += 1

    arm = FakeArm()
    motion = ArmMotion(arm, {"reference_waist_q_rad": [0, 0, 0], "reference_imu_rpy_rad": [0, 0, 0]})
    motion.thread.start()
    try:
        time.sleep(2.0)  # 2秒の待機・推論中も50Hzの送信を続ける。
        assert arm.writes >= 50
        arm.moving = True
        with pytest.raises(RuntimeError, match="状態監視"):
            motion.wait(0.1)
    finally:
        motion.close()
    assert arm.releases >= 1


def test_release_failure_is_preserved_without_masking_control_error():
    class BrokenArm:
        def current_arm(self):
            return (0.0,)*14

        def set_hold(self, current):
            pass

        def current_configuration(self):
            raise RuntimeError("state lost")

        def release(self):
            raise RuntimeError("release lost")

    motion = ArmMotion(BrokenArm(), {})
    motion.thread.start()
    with pytest.raises(RuntimeError, match="state lost"):
        motion.wait(0.1)
    motion.close()
    assert motion.release_error == "release lost"
