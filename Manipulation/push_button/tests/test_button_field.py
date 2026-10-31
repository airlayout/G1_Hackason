"""実機へ接続せず、ボタンの段階接近・中断・FK許容幅を検証する。"""
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'real')]
import align_button
from alignment import FixedTargetAlignment, Observation
from button_control import STAGED_CLEARANCES_M, preview_button_path, validate_fk_margin
from trial_policy import ButtonTrialPolicy, TrialPause, resolve_trial_arguments
from test_alignment import button
from test_real_button_feedback import live


def parse(extra, config=None):
    argv = ['--model', 'unused', '--base-transform', 'unused', '--weights', 'unused', '--output', '/tmp/unused.json']
    if config:
        argv += ['--trial-config', str(config)]
    else:
        argv += ['--stroke', '.0015']
    return resolve_trial_arguments(align_button.build_argument_parser(), argv+extra, task='button')


def test_field_defaults_relax_fk_without_copying_bottle_distance_or_table_settings(tmp_path):
    args, policy = parse(['--profile', 'field'])
    assert policy.fk_tolerance_m == .005 and policy.fk_abort_m == .010
    assert policy.approach_mode == 'staged' and policy.record_rgbd
    assert policy.position_stability_m == .004 and args.stroke == .0015
    assert not hasattr(args, 'push_distance') and not hasattr(args, 'table_height')
    _, strict = parse([])
    assert strict == ButtonTrialPolicy()
    config = tmp_path/'field.json'
    config.write_text(json.dumps({'profile': 'field', 'stroke': .003, 'fk_tolerance_m': .006,
                                 'fk_abort_m': .012, 'record_rgbd': False}))
    args, policy = parse(['--fk-tolerance-m', '.007', '--record-rgbd'], config)
    assert args.stroke == .003 and policy.fk_tolerance_m == .007 and policy.fk_abort_m == .012
    assert policy.record_rgbd


@pytest.mark.parametrize('values', [
    {'fk_tolerance_m': float('nan')}, {'fk_abort_m': .004}, {'fk_tolerance_m': .011},
    {'fk_tolerance_m': True}, {'table_height': .76}, {'push_distance': .02},
    {'record_rgbd': 'false'}, {'approach_extra_retries': 3},
])
def test_invalid_or_bottle_only_settings_are_rejected(tmp_path, values):
    config = tmp_path/'bad.json'
    config.write_text(json.dumps({'profile': 'field', 'stroke': .0015, **values}))
    with pytest.raises((ValueError, TypeError)):
        parse([], config)


def test_relaxed_fk_accepts_4mm_for_three_samples_but_abort_is_separate():
    observation = Observation(button(), 0., 1)
    goal = np.array(button().face_xyz)-np.array([.065, 0., 0.])
    tip = goal+np.array([0., .004, 0.])
    loose = FixedTargetAlignment(observation, tip, .015, .05, 0., tolerance_m=.005, abort_error_m=.010)
    assert loose.next_goal(tip) is not None
    assert loose.next_goal(tip) is not None
    assert loose.next_goal(tip) is None
    strict = FixedTargetAlignment(observation, tip, .015, .05, 0.)
    assert strict.next_goal(tip) is not None
    with pytest.raises(ValueError, match='1cm'):
        strict.next_goal(tip)
    with pytest.raises(ValueError, match='10mm'):
        loose.next_goal(goal+np.array([0., .011, 0.]))


def test_fk_tolerance_must_fit_button_and_calibration():
    policy = ButtonTrialPolicy(fk_tolerance_m=.005, fk_abort_m=.010)
    calibration = {'button_radius_m': .025, 'tip_radius_m': .015, 'position_uncertainty_m': .005}
    assert validate_fk_margin(calibration, policy) == pytest.approx(.005)
    with pytest.raises(ValueError, match='余裕'):
        validate_fk_margin(calibration, replace(policy, fk_tolerance_m=.006))


@pytest.fixture
def staged_live(live, monkeypatch):
    live.policy = ButtonTrialPolicy(profile='field', approach_mode='staged', fk_tolerance_m=.005, fk_abort_m=.010)
    live.answers, live.pauses, live.shifts, live.resets = [], [], {}, 0
    live.args.stop_before_contact = False
    live.normal_fault = None
    live.observe_fault = None
    execute = align_button.execute_alignment

    def wrapper(arm, camera, kin, calibration, args):
        motion = align_button.ArmMotion(arm, calibration)
        motion.monitor = SimpleNamespace(update=lambda _: None)
        calibration.update(button_radius_m=.035, position_uncertainty_m=.001)
        observe = camera.observe

        def measured():
            index = len(live.captures)+1
            if live.observe_fault is not None:
                live.observe_fault(index)
            observation = observe()
            shift = live.shifts.get(index, 0.)
            if shift:
                target = replace(observation.target, face_xyz=(.35, -.25+shift, 1.))
                observation = replace(observation, target=target)
            return observation

        def reset():
            live.resets += 1

        def pause(error, **kwargs):
            live.pauses.append((error, kwargs))
            return live.answers.pop(0) if live.answers else 'quit'

        camera.observe, camera.reset_measurement_reference = measured, reset
        args.pause_handler = pause
        return execute(arm, camera, kin, calibration, args, motion=motion, policy=live.policy)

    monkeypatch.setattr(align_button, 'execute_alignment', wrapper)
    return live


def test_four_stages_then_button_stroke_lamp_and_reverse_return(staged_live):
    report = staged_live.run()
    assert report['completed'] and report['lamp_confirmed'] and report['home_completed'] and report['release_completed']
    assert [row['clearance_m'] for row in report['approach_steps']] == list(STAGED_CLEARANCES_M)
    assert report['measurement_count'] == 5 and report['approach_remeasurement_count'] == 4
    assert all(row['accepted'] and row['fk_error_m'] <= .005 for row in report['approach_steps'])
    assert staged_live.attempts[0]['stroke'] == .0015
    assert max(row['tip_xyz_m'][0]-.35+.015 for row in report['normal_steps']) <= .0015+.00015
    np.testing.assert_allclose(staged_live.q[22:], staged_live.home)
    assert report['physical_press_success'] is None


def test_precontact_diagnostic_never_presses_or_checks_lamp(staged_live):
    staged_live.args.stop_before_contact = True
    report = staged_live.run()
    assert report['diagnostic_completed'] and report['home_completed'] and report['release_completed']
    assert not report['motion_completed'] and report['lamp_confirmed'] is None and not staged_live.attempts
    assert report['approach_remeasurement_count'] == 4
    assert align_button.button_exit_code(report, diagnostic=True) == 0
    assert align_button.button_exit_code(report) != 0


def test_target_shift_repeats_same_stage_and_shared_budget_holds(staged_live):
    staged_live.shifts = {2: .01, 3: .01}
    report = staged_live.run()
    rows = report['approach_steps']
    assert [row['clearance_m'] for row in rows] == [.10, .10, .07, .07, .05, .03]
    assert report['approach_extra_retry_count'] == 2 and report['completed']


def test_resume_replans_all_stages_without_enabling_again(staged_live):
    staged_live.shifts = {2: .01, 4: .01}
    staged_live.answers = ['resume']
    report = staged_live.run()
    assert report['completed'] and staged_live.resets == 1
    assert staged_live.pauses[0][1] == {'allow_resume': True}
    assert [run['completed'] for run in report['approach_runs']] == [False, True]
    assert report['approach_runs'][1]['extra_retry_count'] == 0


def test_measurement_pause_can_return_before_contact(staged_live):
    staged_live.answers = ['return']

    def fail(index):
        if index == 3:
            raise TrialPause('対象消失')

    staged_live.observe_fault = fail
    report = staged_live.run()
    assert report['cancelled'] and report['home_completed'] and report['release_completed']
    assert not staged_live.attempts and not report['motion_completed']


def test_fault_after_press_start_never_accepts_resume_or_repress(staged_live, monkeypatch):
    staged_live.answers = ['resume']  # 不正なハンドラーでも再押下しない。
    attempts = []

    def fail(*args):
        attempts.append(1)
        raise TrialPause('押下中の姿勢変化')

    monkeypatch.setattr(align_button, 'run_continuous_normal', fail)
    report = staged_live.run()
    assert len(attempts) == 1 and report['cancelled'] and report['release_completed']
    assert staged_live.pauses[0][1] == {'allow_resume': False}
    assert not staged_live.attempts and staged_live.resets == 0


def test_fk_abort_is_fatal_without_spending_extra_measurement_budget(staged_live, monkeypatch):
    def fail(*args, **kwargs):
        raise ValueError('補正後のFK誤差が10mmを超えました')

    monkeypatch.setattr(align_button, 'align_fixed_target', fail)
    with pytest.raises(ValueError, match='10mm'):
        staged_live.run()
    report = json.loads(staged_live.args.output.read_text())
    assert report['fatal_error'] and report['release_completed'] and not report['motion_completed']
    assert not staged_live.pauses and not staged_live.attempts
    assert report['approach_extra_retry_count'] == 0


def test_pause_during_return_retries_return_without_another_press(staged_live, monkeypatch):
    staged_live.answers = ['return']
    replay = align_button.ArmMotion.replay
    calls = []

    def interrupted(motion, targets, validate):
        calls.append(1)
        if len(calls) == 1:
            raise TrialPause('復帰中の姿勢変化')
        return replay(motion, targets, validate)

    monkeypatch.setattr(align_button.ArmMotion, 'replay', interrupted)
    report = staged_live.run()
    assert report['completed'] and report['home_completed'] and report['release_completed']
    assert len(calls) == 2 and len(staged_live.attempts) == 1
    assert staged_live.pauses[0][1] == {'allow_resume': False}


def test_fault_while_waiting_for_operator_preserves_execution_report(staged_live, monkeypatch):
    class FailingAnswers:
        def __bool__(self): return True
        def pop(self, index): raise RuntimeError('state lost while holding')

    staged_live.answers = FailingAnswers()
    monkeypatch.setattr(align_button, 'run_continuous_normal',
                        lambda *args: (_ for _ in ()).throw(TrialPause('pose changed')))
    with pytest.raises(RuntimeError, match='state lost while holding') as caught:
        staged_live.run()
    assert caught.value.button_report['approach_runs'][0]['completed']
    assert caught.value.button_report['release_completed']
    assert caught.value.button_report['fatal_error']
    assert caught.value.button_report['error'] == 'state lost while holding'


@pytest.mark.parametrize('normal_motion', ['smooth', 'stepped'])
def test_relaxed_convergence_never_allows_press_outside_button_margin(staged_live, monkeypatch, normal_motion):
    staged_live.args.normal_motion = normal_motion
    freeze = align_button.ArmMotion.freeze_approach

    def shifted(motion):
        freeze(motion)
        # 接近を確定した直後、押下開始時に25mm横へずれる（余裕は19mm）。
        staged_live.q[23] += .025

    monkeypatch.setattr(align_button.ArmMotion, 'freeze_approach', shifted)
    with pytest.raises(ValueError, match='余裕'):
        staged_live.run()
    report = json.loads(staged_live.args.output.read_text())
    assert report['fatal_error'] and report['release_completed'] and not staged_live.attempts


@pytest.mark.parametrize('lamp', [False, None])
def test_unknown_or_unlit_is_not_cli_success_even_after_return(lamp):
    report = {'completed': True, 'arm_returned': True, 'release_completed': True, 'lamp_confirmed': lamp}
    assert align_button.button_exit_code(report) == 2


def test_staged_iterative_combination_is_rejected_before_loading_models(tmp_path, monkeypatch):
    monkeypatch.setattr(align_button, 'load_robot', lambda *a: pytest.fail('model must not load'))
    assert align_button.main(['--model', 'unused', '--base-transform', 'unused', '--weights', 'unused',
        '--stroke', '.0015', '--output', str(tmp_path/'bad.json'), '--profile', 'field',
        '--alignment-mode', 'iterative']) == 2


@pytest.mark.parametrize('height', [1., .92])
@pytest.mark.parametrize('x,yaw', [(.33, -3.), (.36, 0.), (.39, 3.)])
def test_staged_paths_and_return_with_real_menagerie_model(height, x, yaw):
    path = os.environ.get('G1_MUJOCO_MODEL')
    if not path:
        pytest.skip('G1_MUJOCO_MODELを指定してください')
    from kinematics import ArmKinematics, load_robot
    model, rest = load_robot(Path(path), (.11, -.003, 0), .79)
    kin = ArmKinematics(model)
    kin.update(rest)
    normal = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0.)
    observation = Observation(button(x=x, z=height, normal=normal), 0., 1)
    args = SimpleNamespace(stroke=.0015, clearance=.05, alignment_mode='single', stop_before_contact=False)
    poses = preview_button_path(kin, observation, .015, args, ButtonTrialPolicy(approach_mode='staged'))
    np.testing.assert_array_equal(kin.data.qpos, rest)
    for index, clearance in enumerate(STAGED_CLEARANCES_M, 1):
        q = rest.copy()
        q[kin.qadr] = poses[f'approach_{index}']
        kin.update(q)
        expected = np.array(observation.target.face_xyz)-(.015+clearance)*np.array(normal)
        np.testing.assert_allclose(kin.tip_position(), expected, atol=.0001)


@pytest.mark.parametrize('cleanup_failure', [False, True])
def test_field_entry_records_real_localizer_frames_state_and_commands(monkeypatch, tmp_path, cleanup_failure):
    from test_field_trial import FakeArm
    from test_vision import BBOX, INTRINSICS, sample_depth
    from kinematics import load_robot
    import field_motion
    import localize_button
    model = os.environ.get('G1_MUJOCO_MODEL')
    if not model:
        pytest.skip('G1_MUJOCO_MODELを指定してください')
    _, rest = load_robot(Path(model), (.11, -.003, 0), .79)
    calibration = {'schema': 'g1-button-calibration-v2', 'camera_body': 'torso_link',
        'camera_offset_m': [0, 0, 0], 'camera_rpy_offset_rad': [0, 0, 0],
        'reference_waist_q_rad': (rest[12:15]+.04).tolist(), 'reference_imu_rpy_rad': [0, 0, 0],
        'button_radius_m': .035, 'tip_radius_m': .015, 'tip_offset_m': [.11, -.003, 0],
        'pelvis_height_m': .79, 'position_uncertainty_m': .001}
    cal = tmp_path/'calibration.json'
    cal.write_text(json.dumps(calibration))
    depth = sample_depth()
    # RGB-Dの正面Zを床基準Xへ写す。検出枠の中央が(0.35, -0.25, 1)。
    transform = np.array([[0, 0, 1, -.6], [-1, 0, 0, -.136], [0, -1, 0, 1.], [0, 0, 0, 1]])
    geometries, created_arms = [], []

    class Arm(FakeArm):
        def __init__(self, *args):
            super().__init__()
            self.q = rest.copy()
            created_arms.append(self)
        def stop_walking(self): pass
        def close(self):
            if cleanup_failure:
                raise RuntimeError('close failed')

    class Camera:
        def __init__(self, *args): self.sequence = 0
        def poll(self):
            self.sequence += 1
            return SimpleNamespace(color_bgr=np.zeros((240, 320, 3), np.uint8), depth_m=lambda: depth,
                intrinsics=INTRINSICS, frame_id=self.sequence), time.monotonic()-.01
        def close(self): pass

    class Geometry:
        def __init__(self, runtime, kin, calibration):
            self.kin = kin
            geometries.append(self)
        def transform(self, q, imu):
            self.kin.update_fixed_base(np.array(q), imu)
            return transform.copy()

    class Operator:
        def __init__(self, *args): pass
        def read(self, *args, **kwargs): return 'yes'
        def __call__(self, *args, **kwargs): pytest.fail('unexpected pause')

    detector = SimpleNamespace(detect=lambda _: [SimpleNamespace(class_name='button', bbox=BBOX)])
    runtime = SimpleNamespace(module=lambda _: SimpleNamespace(perception=lambda _: SimpleNamespace(
        YoloDetector=lambda **kwargs: detector)))

    def execute(arm, camera, kin, calibration, args, *, motion, policy):
        assert policy.field and policy.fk_tolerance_m == .005 and policy.fk_abort_m == .010
        assert kin is not geometries[0].kin and kin.model is not geometries[0].kin.model
        observation = camera.observe()
        np.testing.assert_allclose(observation.target.face_xyz, [.35, -.25, 1.], atol=1e-5)
        tau = motion.gravity(arm.current_configuration(), velocity=np.zeros(7))
        assert len(tau) == 7 and max(abs(value) for value in tau) > 0
        arm.publish(rest[22:29], .1, right_tau=tau)
        return {'completed': True, 'motion_completed': True, 'home_completed': True,
                'arm_returned': True, 'release_completed': True, 'lamp_confirmed': True,
                'physical_press_success': None}

    monkeypatch.setattr(align_button, 'J1CameraRuntime', lambda _: runtime)
    monkeypatch.setattr(align_button, 'CalibratedHeadCamera', Geometry)
    monkeypatch.setattr(align_button, 'ArmSdk', Arm)
    monkeypatch.setattr(align_button, 'execute_alignment', execute)
    monkeypatch.setattr(localize_button, 'J1RgbdCamera', Camera)
    monkeypatch.setattr(field_motion, 'FieldOperator', Operator)
    output = tmp_path/'trial.json'
    code = align_button.main(['--profile', 'field', '--model', model, '--weights', 'unused',
        '--base-transform', str(cal), '--stroke', '.0015', '--gravity-scale', '.2', '--dynamics-feedforward',
        '--execute', '--calibrated', '--output', str(output)])
    assert code == (2 if cleanup_failure else 0)
    report = json.loads(output.read_text())
    assert report['calibration']['reference_waist_q_rad'] == calibration['reference_waist_q_rad']
    assert not np.allclose(report['monitor_references'][0]['waist_q_rad'], calibration['reference_waist_q_rad'])
    assert report['arm_commands'] and len(report['monitor_samples'][0]['motor_q_rad']) == 29
    assert report['rgbd_recording']['saved_frames'] >= 3 and report['measurements']
    assert report['applied_settings']['source_sha256'] and report['applied_settings']['model_sha256']
    assert created_arms[0].releases
    if cleanup_failure:
        assert not report['completed'] and not report['release_completed']
