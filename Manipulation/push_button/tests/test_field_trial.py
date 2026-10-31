"""実機の停止事例を使う回帰試験と、現場モードの模擬I/O試験。実機へは接続しない。"""
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'real')]
import bottle
from bottle import BottleFrame, BottleLocalizer, BottleSettings, BottleTarget
from field_motion import FieldArmMotion, FieldMonitor, FieldOperator
from push_bottle import build_argument_parser
from table_reference import estimate_table_transform
from trial_policy import ObservationUnavailable, TrialPause, TrialPolicy, resolve_trial_arguments, trial_exit_code
from trial_recording import TrialRecorder
from test_table_reference import table_depth


def field(**kwargs):
    return replace(TrialPolicy(profile='field', table_edge_tolerance=.015, table_x_edge='x_min',
        table_y_edge='y_min', position_stability_m=.008, waist_delta_deg=3., imu_delta_deg=3., record_rgbd=True), **kwargs)


def parse(extra, config=None):
    argv = ['--model', 'unused.xml', '--base-transform', 'unused.json', '--output', '/tmp/unused.json']
    if config is None:
        argv += ['--table-height', '.76']
    else:
        argv += ['--trial-config', str(config)]
    return resolve_trial_arguments(build_argument_parser(), argv+extra)


def test_field_defaults_and_explicit_cli_over_json_over_profile(tmp_path):
    config = tmp_path/'trial.json'
    config.write_text(json.dumps({'profile': 'field', 'table_height': .76, 'push_distance': .03,
                                 'table_x_edge': 'x_max', 'position_stability_m': .006}))
    args, policy = parse(['--push-distance=.04', '--table-x-edge', 'x_min', '--no-record-rgbd'], config)
    assert args.push_distance == .04 and args.table_height == .76
    assert not args.track_table and policy.field
    assert policy.table_x_edge == 'x_min' and policy.position_stability_m == .006
    assert not policy.record_rgbd and policy.table_edge_tolerance == .015
    args, policy = parse(['--profile', 'field'])
    assert args.push_distance == .02 and policy.position_stability_m == .008 and not args.track_table
    args, policy = parse([])
    assert args.push_distance == .05 and not args.track_table
    assert policy == TrialPolicy()


@pytest.mark.parametrize('profile', ['strict', 'field'])
@pytest.mark.parametrize('configured,flag,expected', [
    (True, None, True), (False, None, False),
    (None, '--track-table', True), (None, '--no-track-table', False),
    (True, '--no-track-table', False), (False, '--track-table', True),
])
def test_table_tracking_is_opt_in_with_cli_over_json(tmp_path, profile, configured, flag, expected):
    values = {'profile': profile, 'table_height': .76}
    if configured is not None:
        values['track_table'] = configured
    config = tmp_path/'trial.json'
    config.write_text(json.dumps(values))
    args, _ = parse([flag] if flag else [], config)
    assert args.track_table is expected


def test_field_example_defaults_to_no_table_tracking():
    args, policy = parse([], ROOT/'real'/'field_trial.example.json')
    assert policy.field and not args.track_table


@pytest.mark.parametrize('values', [
    {'unknown': 1}, {'record_rgbd': 'false'}, {'track_table': 'false'},
    {'position_stability_m': float('nan')}, {'table_x_edge': 'y_min'},
    {'table_bounds': [0, 1]}, {'motion_persistence_s': 0}, {'profile': 'other'},
])
def test_invalid_json_is_rejected_before_runtime_setup(tmp_path, values):
    config = tmp_path/'trial.json'
    config.write_text(json.dumps({'table_height': .76, **values}))
    with pytest.raises((ValueError, TypeError)):
        parse([], config)


def test_resolving_config_does_not_leak_parser_defaults(tmp_path):
    parser = build_argument_parser()
    common = ['--model', 'x', '--base-transform', 'x', '--output', 'x']
    config = tmp_path/'trial.json'
    config.write_text(json.dumps({'table_height': .76, 'profile': 'field'}))
    resolve_trial_arguments(parser, common+['--trial-config', str(config)])
    with pytest.raises(SystemExit):
        resolve_trial_arguments(parser, common)


def test_field_exit_is_processing_completion_and_does_not_invent_physical_success():
    report = {'completed': True, 'motion_completed': True, 'home_completed': True,
              'release_completed': True, 'visual_success': False, 'physical_push_success': None}
    assert trial_exit_code(report, field()) == 0
    assert trial_exit_code(report, TrialPolicy()) == 1
    for key in ('motion_completed', 'home_completed', 'release_completed'):
        assert trial_exit_code({**report, key: False}, field()) == 1
    assert report['visual_success'] is False and report['physical_push_success'] is None


def test_fixed_table_edge_does_not_follow_residual_ranking_or_fall_back(monkeypatch):
    import table_reference
    original = table_reference._edge

    def shifted(points, axis, side='min'):
        fitted, normal, span = original(points, axis, side)
        if axis == 0 and side == 'max':
            fitted = fitted.copy()
            fitted[:, 0] += .01
        return fitted, normal, span

    monkeypatch.setattr(table_reference, '_edge', shifted)
    details = {}
    estimate_table_transform(*table_depth(), opposite_edge_tolerance_m=.015,
                             selected_edges=('x_min', 'y_min'), diagnostics=details)
    assert details['selected_edges'] == ['x_min', 'y_min']

    def missing(points, axis, side='min'):
        if axis == 0 and side == 'min':
            raise ValueError('chosen edge occluded')
        return original(points, axis, side)

    monkeypatch.setattr(table_reference, '_edge', missing)
    with pytest.raises(ValueError, match='指定した台の基準辺'):
        estimate_table_transform(*table_depth(), selected_edges=('x_min', 'y_min'))


@pytest.mark.parametrize('edge,tolerance,stable,rejected', [(None, .004, 10, 34), ('x_min', .004, 44, 0),
                                                          ('x_min', .008, 44, 0)])
def test_recorded_retry_through_actual_localizer_stability(monkeypatch, edge, tolerance, stable, rejected):
    run = json.loads((ROOT/'LOG/bottle-lab-20261031/precontact_retry.json').read_text())
    references = {r['captured_at_s']: r for r in run['table_reference']}
    details = {r['captured_at_s']: r for r in run['table_diagnostics']}
    clock = SimpleNamespace(t=run['phases'][0]['time_s'])
    current = {}
    detector = SimpleNamespace(detect=lambda _: [SimpleNamespace(class_name='bottle',
        bbox=current['target']['bbox'], confidence=.9)])
    monkeypatch.setattr(bottle, 'estimate_bottle_target', lambda *a, **k: BottleTarget(**current['target']))
    loc = BottleLocalizer(None, detector, BottleSettings(.76, table_bounds=(.31, .61, -.36, .04)),
                          lambda: clock.t, position_stability_m=tolerance)
    loc.begin(clock.t)
    events = [(r['time_s'], None) for r in run['table_rejections']]
    events += [(r['captured_at_s']+r['age_after_fit_s'], r) for r in run['perception_frames']]
    sequences = []
    for stamp, frame in sorted(events, key=lambda r: r[0]):
        clock.t = stamp
        if frame is None:
            loc.samples.clear()
            continue
        target = dict(frame['targets'][0])
        if edge:
            capture = frame['captured_at_s']
            dx = details[capture]['edges'][edge]['offset_m']-references[capture]['base_offset_m'][0]
            target['center_xy'] = [target['center_xy'][0]+dx, target['center_xy'][1]]
            target['face_xyz'] = [target['face_xyz'][0]+dx, *target['face_xyz'][1:]]
        current['target'] = target
        obs = loc.consume(BottleFrame(np.zeros((1, 1, 3), np.uint8), np.ones((1, 1)), None,
                                     np.eye(4), frame['captured_at_s'], frame['sequence']))
        if obs:
            sequences.append(obs.sequence)
    assert len(sequences) == stable
    assert loc.rejections['フレーム間の位置または半径が不安定'] == rejected
    if edge is None:
        assert sequences == [r['sequence'] for r in run['measurements']]


def configuration(clock, *, pitch=0., leg=0., gyro=0.):
    q = np.zeros(29)
    q[14] = pitch
    return q, (0., 0., 0.), clock.t, leg, gyro


def test_transient_hold_waits_for_stability_but_persistent_motion_requires_operator():
    clock = SimpleNamespace(t=1.)
    monitor = FieldMonitor(field(), configuration(clock), now=lambda: clock.t)
    monitor.update(configuration(clock, leg=.174))
    assert monitor.pending.automatic
    clock.t += .02
    monitor.update(configuration(clock))
    assert not monitor.stable()
    clock.t += .301
    monitor.update(configuration(clock))
    assert monitor.stable()
    monitor.resume()
    assert monitor.pending is None
    monitor.update(configuration(clock, leg=.2))
    clock.t += .101
    monitor.update(configuration(clock, leg=.2))
    assert not monitor.pending.automatic
    with pytest.raises(TrialPause):
        monitor.resume()


def test_trial_reference_is_current_pose_and_rebase_does_not_change_calibration():
    clock = SimpleNamespace(t=1.)
    monitor = FieldMonitor(field(), configuration(clock, pitch=.5), now=lambda: clock.t)
    monitor.update(configuration(clock, pitch=.5+.02243))
    assert monitor.pending is None
    clock.t += .1
    monitor.update(configuration(clock, pitch=.5+math.radians(3.1)))
    assert monitor.pending.code == 'pose' and not monitor.pending.automatic
    clock.t += .301
    monitor.update(configuration(clock, pitch=.5+math.radians(3.1)))
    initial = monitor.references[0].copy()
    monitor.resume(rebase=True)
    assert monitor.pending is None and monitor.references[0] == initial
    assert len(monitor.references) == 2


@pytest.mark.parametrize('invalid', ['stale', 'future', 'nan', 'short', 'joint_limit'])
def test_invalid_state_is_fatal_even_while_paused(invalid):
    clock = SimpleNamespace(t=1.)
    monitor = FieldMonitor(field(), configuration(clock), now=lambda: clock.t)
    monitor.pause('waiting')
    q, imu, stamp, leg, gyro = configuration(clock)
    if invalid == 'stale': stamp -= .101
    if invalid == 'future': stamp += .01
    if invalid == 'nan': q[22] = np.nan
    if invalid == 'short': q = q[:-1]
    if invalid == 'joint_limit': q[22] = 100.
    with pytest.raises(RuntimeError) as caught:
        monitor.update((q, imu, stamp, leg, gyro))
    assert not isinstance(caught.value, TrialPause)


class FakeArm:
    def __init__(self):
        self.q = np.zeros(29)
        self.leg = 0.
        self.stale = False
        self.sent, self.releases = [], []

    def current_configuration(self):
        return self.q.copy(), (0., 0., 0.), time.monotonic()-(1. if self.stale else 0.), self.leg, 0.

    def current_arm(self):
        return self.q[15:29].tolist()

    def set_hold(self, q):
        pass

    def publish(self, goal, weight, **kwargs):
        self.sent.append((time.monotonic(), np.array(goal)))
        self.q[22:29] = goal

    def release(self):
        self.releases.append(time.monotonic())


def wait_until(predicate, timeout=1.):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() > deadline:
            pytest.fail('background condition timed out')
        time.sleep(.005)


def test_hold_keeps_publishing_and_rejects_new_goals_then_fault_releases():
    arm = FakeArm()
    monitor = FieldMonitor(field(), arm.current_configuration())
    motion = FieldArmMotion(arm, {}, monitor=monitor)
    try:
        motion.enabled, motion.weight = True, 1.
        wait_until(lambda: len(arm.sent) >= 2)
        arm.leg = .2
        wait_until(lambda: monitor.pending is not None)
        with pytest.raises(TrialPause):
            motion.stream(np.ones(7)*.1)
        held = motion.current_target()
        count = len(arm.sent)
        wait_until(lambda: len(arm.sent) >= count+3)
        assert all(np.allclose(goal, held) for _, goal in arm.sent[count:])
        arm.stale = True
        wait_until(lambda: motion.error is not None)
        with pytest.raises(RuntimeError, match='100ms'):
            motion.fatal_check()
        wait_until(lambda: bool(arm.releases))
    finally:
        motion.close()


def test_operator_input_wait_does_not_mask_a_fatal_disconnect():
    arm = FakeArm()
    motion = FieldArmMotion(arm, {}, monitor=FieldMonitor(field(), arm.current_configuration()))
    read_fd, write_fd = os.pipe()
    reader = os.fdopen(read_fd)
    try:
        # 改行の来ない入力を受けても、監視がreadline待ちで止まらないこと。
        os.write(write_fd, b'r')
        operator = FieldOperator(motion, SimpleNamespace(check=lambda: None), stdin=reader, output=lambda _: None)
        timer = threading.Timer(.05, lambda: setattr(arm, 'stale', True))
        timer.start()
        with pytest.raises(RuntimeError, match='100ms'):
            operator.read('waiting for an operator who does not answer')
        timer.join()
    finally:
        os.close(write_fd)
        reader.close()
        motion.close()


def test_operator_automatically_remeasures_a_short_motion_but_not_after_push():
    arm = FakeArm()
    policy = field(stable_hold_s=.1)
    monitor = FieldMonitor(policy, arm.current_configuration())
    motion = FieldArmMotion(arm, {}, monitor=monitor)
    read_fd, write_fd = os.pipe()
    reader = os.fdopen(read_fd)
    try:
        wait_until(monitor.stable)
        operator = FieldOperator(motion, SimpleNamespace(check=lambda: None), stdin=reader, output=lambda _: None)
        assert operator(TrialPause('short movement', code='body_motion', automatic=True), allow_resume=True) == 'resume'
        assert monitor.pending is None
        os.write(write_fd, b'r\nb\n')
        assert operator(TrialPause('push interrupted', automatic=True), allow_resume=False) == 'return'
        assert monitor.pending is None
        assert any(event['event'] == 'operator_action' and event['action'] == 'b' for event in monitor.events)
    finally:
        os.close(write_fd)
        reader.close()
        motion.close()


def test_move_interruption_freezes_the_command_and_next_move_starts_from_observed_pose():
    arm = FakeArm()
    monitor = FieldMonitor(field(stable_hold_s=.1), arm.current_configuration())
    motion = FieldArmMotion(arm, {}, monitor=monitor)
    try:
        motion.enabled, motion.weight = True, 1.
        timer = threading.Timer(.1, lambda: setattr(arm, 'leg', .2))
        timer.start()
        with pytest.raises(TrialPause):
            motion.move(np.full(7, .1), .5)
        timer.join()
        motion.hold(monitor.pending)
        held = motion.current_target().copy()
        count = len(arm.sent)
        wait_until(lambda: len(arm.sent) >= count+3)
        assert all(np.allclose(goal, held) for _, goal in arm.sent[count:])
        arm.leg = 0.
        wait_until(monitor.stable)
        monitor.resume(rebase=True)
        motion.move(held+.005, .1, settle_s=.02)
        assert np.allclose(motion.current_target(), held+.005)
        assert motion.error is None
    finally:
        motion.close()


def test_hold_respects_bridge_command_speed_when_the_arm_has_tracking_lag():
    class LagBridge(FakeArm):
        previous = None
        sent_at = None

        def publish(self, goal, weight, **kwargs):
            now = time.monotonic()
            goal = np.array(goal)
            if self.previous is not None and np.max(np.abs(goal-self.previous)) > .8*(now-self.sent_at)+.005:
                raise RuntimeError('SDK bridge command speed exceeded')
            self.previous, self.sent_at = goal.copy(), now
            self.sent.append((now, goal.copy()))
            self.q[22:29] = goal-.03

    arm = LagBridge()
    motion = FieldArmMotion(arm, {}, monitor=FieldMonitor(field(), arm.current_configuration()))
    try:
        motion.enabled, motion.weight = True, 1.
        motion.move(np.full(7, .05), .3, settle_s=.03)
        arm.leg = .2
        wait_until(lambda: motion.monitor.pending is not None)
        motion.hold(motion.monitor.pending)
        held_goal = motion.hold_goal.copy()
        wait_until(lambda: np.allclose(motion.current_target(), held_goal))
        assert motion.error is None
        assert np.allclose(held_goal, .02)
    finally:
        motion.close()


def test_yes_refreshes_both_arm_holds_and_trial_reference_without_publishing():
    class Arm(FakeArm):
        def set_hold(self, q): self.held = list(q)

    arm = Arm()
    monitor = FieldMonitor(field(), arm.current_configuration())
    motion = FieldArmMotion(arm, {}, monitor=monitor)
    try:
        arm.q[15:29] = .04
        arm.q[14] = .02
        motion.begin_trial()
        assert arm.held == pytest.approx([.04]*14)
        assert motion.home == pytest.approx([.04]*7)
        assert monitor.references[-1]['waist_q_rad'][2] == .02
        assert not arm.sent
    finally:
        motion.close()


def test_remeasure_after_pause_accepts_a_bottle_repositioned_by_the_operator():
    from bottle_perception import PerceptionEngine, PerceptionMailbox
    from test_bottle import Camera, SETTINGS
    camera = Camera()
    engine = PerceptionEngine(lambda: None, camera, SETTINGS, camera.now, recoverable=True)
    box = PerceptionMailbox(engine, camera.now, lambda _: None)
    box.reference = SimpleNamespace(center_xy=(.37, -.25), radius_m=.034)
    old_generation = box.generation
    box.reset_measurement_reference()
    box.request('measure')
    generation, request, _ = box.take_request()
    engine.configure(request)
    camera.x = .44
    result = None
    for _ in range(3):
        frame = camera.read(camera.t, camera.t+1.)
        result = engine.localizer.consume(frame)
    assert result is not None and result.target.center_xy[0] == pytest.approx(.44, abs=.001)
    box.publish(old_generation, result, None)
    assert box.latest is None
    box.error = RuntimeError('camera disconnected')
    box.reset_measurement_reference()
    with pytest.raises(RuntimeError, match='camera disconnected'):
        box.check()


def test_fresh_current_pose_is_used_for_a_return_after_pause(monkeypatch):
    from test_bottle_live_control import routine
    r, events, _ = routine(monkeypatch)
    checked = []
    actual = np.zeros(29)
    actual[22:29] = .02

    class Kin:
        qadr = np.arange(22, 29)
        data = SimpleNamespace(qpos=np.zeros(29))

        def update(self, q): self.data.qpos = q.copy()
        def tip_position(self): return np.array([.25, -.3, .9])
        def solve(self, xyz, normal, **kwargs): return SimpleNamespace(joint_angles=np.full(7, .03))
        def validate_path(self, goal, *args): checked.append(self.data.qpos.copy())

    class Motion:
        home = np.zeros(7)
        def check(self): pass
        def current_target(self): return actual[22:29].copy()
        def move(self, goal, *args, **kwargs): actual[22:29] = goal

    r.kin, r.motion = Kin(), Motion()
    r.arm_enabled = True
    r.last_face = np.array([.35, -.21, .84])
    r.refresh = lambda: r.kin.update(actual)
    r.move = bottle.BottlePushRoutine.move.__get__(r)
    r.return_home = bottle.BottlePushRoutine.return_home.__get__(r)
    r.field_return()
    assert r.report['home_completed']
    assert checked and np.allclose(checked[0][22:29], .02)
    assert np.allclose(actual[22:29], 0.)


def test_field_diagnostic_can_wait_during_return_without_restarting_approach(monkeypatch):
    from test_bottle_live_control import routine
    r, events, _ = routine(monkeypatch, diagnostic=True)
    attempts = []

    def return_current(**kwargs):
        attempts.append(True)
        if len(attempts) == 1:
            raise TrialPause('temporary pose change during return')
        r.report['home_completed'] = True

    r.field_return = return_current
    actions = []
    r.pause_handler = lambda pause, **kw: actions.append(kw) or 'return'
    report = r.execute()
    assert report['diagnostic_completed'] and report['home_completed']
    assert len(attempts) == 2 and actions == [{'allow_resume': False}]
    assert events.count('enable') == 1 and 'pushing' not in events


def test_real_field_entry_records_inputs_commands_frames_and_human_result(monkeypatch, tmp_path):
    import push_bottle
    import table_reference
    from kinematics import load_robot
    from test_bottle import cylinder_image, INTRINSICS, TRANSFORM
    model = os.environ.get('G1_MUJOCO_MODEL')
    if not model:
        pytest.skip('G1_MUJOCO_MODELを指定してください')
    _, rest = load_robot(Path(model), (.11, -.003, 0), .79)
    calibration = {'schema': 'g1-button-calibration-v2', 'camera_body': 'torso_link',
        'camera_offset_m': [0, 0, 0], 'camera_rpy_offset_rad': [0, 0, 0],
        'reference_waist_q_rad': (rest[12:15]+.04).tolist(), 'reference_imu_rpy_rad': [0, 0, 0],
        'tip_radius_m': .015, 'tip_offset_m': [.11, -.003, 0],
        'pelvis_height_m': .79, 'position_uncertainty_m': .001}
    cal = tmp_path/'calibration.json'
    cal.write_text(json.dumps(calibration))
    depth, bbox = cylinder_image(.33)

    class Arm(FakeArm):
        def __init__(self, *args):
            super().__init__()
            self.q = rest.copy()
        def stop_walking(self): pass

    class Camera:
        def __init__(self, *args): self.sequence = 0
        def poll(self):
            self.sequence += 1
            return SimpleNamespace(color_bgr=np.zeros((240, 320, 3), np.uint8), depth_m=lambda: depth,
                intrinsics=INTRINSICS, frame_id=self.sequence), time.monotonic()-.03
        def close(self): pass

    class Geometry:
        def __init__(self, runtime, kin, calibration): self.kin = kin
        def transform(self, q, imu):
            self.kin.update_fixed_base(np.array(q), imu)
            return TRANSFORM.copy()

    class Operator:
        def __init__(self, *args): self.answers = iter(['yes', 'y', 'n'])
        def read(self, *args, **kwargs): return next(self.answers)
        def __call__(self, *args, **kwargs): pytest.fail('unexpected pause')

    class Routine:
        def __init__(self, kin, motion, localizer, *args, **kwargs):
            self.motion, self.localizer = motion, localizer
            assert kwargs['pause_handler'] is not None
            assert kwargs['tracking_check'] is None
            assert kin.model.geom('push_table').id >= 0
        def execute(self):
            self.localizer.observe()
            # 入口の送信記録を模擬I/Oで通す。実機成功の判定は意図的にfalse。
            self.motion.arm.publish(rest[22:29], .1)
            self.motion.close()
            self.report = {'completed': True, 'motion_completed': True, 'home_completed': True,
                'release_completed': True, 'visual_success': False, 'physical_push_success': None}
            return self.report

    monkeypatch.setattr(push_bottle, 'J1CameraRuntime', lambda *_: object())
    monkeypatch.setattr(push_bottle, 'CalibratedHeadCamera', Geometry)
    monkeypatch.setattr(push_bottle, 'J1RgbdCamera', Camera)
    monkeypatch.setattr(push_bottle, 'make_bottle_detector', lambda *a, **k: SimpleNamespace(
        detect=lambda _: [SimpleNamespace(class_name='bottle', bbox=bbox)]))
    monkeypatch.setattr(push_bottle, 'ArmSdk', Arm)
    monkeypatch.setattr(push_bottle, 'BottlePushRoutine', Routine)
    monkeypatch.setattr(table_reference, 'TableReference', lambda *a, **k: pytest.fail('台認識を呼んではいけません'))
    monkeypatch.setattr(sys.modules[FieldOperator.__module__], 'FieldOperator', Operator)
    output = tmp_path/'field.json'
    code = push_bottle.main(['--profile', 'field', '--model', model,
        '--base-transform', str(cal), '--table-height', '.75', '--execute', '--calibrated', '--output', str(output)])
    assert code == 0
    report = json.loads(output.read_text())
    assert report['calibration']['reference_waist_q_rad'] == calibration['reference_waist_q_rad']
    assert not np.allclose(report['monitor_references'][0]['waist_q_rad'], calibration['reference_waist_q_rad'])
    assert report['applied_settings']['profile'] == 'field' and report['arm_commands']
    assert report['applied_settings']['arguments']['track_table'] is False
    assert len(report['monitor_samples'][0]['motor_q_rad']) == 29
    assert report['rgbd_recording']['saved_frames'] >= 3
    assert report['operator_assessment'] == {'moved': True, 'upright': False}
    assert report['visual_success'] is False and report['physical_push_success'] is None


def test_measurement_pause_restarts_with_new_measurement_and_only_enables_once(monkeypatch):
    from test_bottle_live_control import routine
    r, events, _ = routine(monkeypatch)
    original = r.localizer.observe
    first = [True]

    def observe():
        if first and first.pop():
            raise TrialPause('measurement timeout')
        return original()

    r.localizer.observe = observe
    actions = []
    r.pause_handler = lambda pause, **kw: actions.append(kw) or 'resume'
    report = r.execute()
    assert report['completed'] and report['home_completed']
    assert events.count('enable') == 1 and actions == [{'allow_resume': True}]


def test_interrupted_push_cannot_be_resumed_even_if_handler_requests_it(monkeypatch):
    from test_bottle_live_control import routine
    r, events, _ = routine(monkeypatch)
    attempts = []

    def interrupted(*args):
        attempts.append(True)
        raise TrialPause('pose changed during push')

    monkeypatch.setattr(bottle, 'run_continuous_normal', interrupted)
    actions = []
    r.pause_handler = lambda pause, **kw: actions.append(kw) or 'resume'
    report = r.execute()
    assert len(attempts) == 1 and actions == [{'allow_resume': False}]
    assert not report['motion_completed'] and not report['completed']
    assert events[-1] == 'release'


def test_fatal_during_push_does_not_send_recovery_commands(monkeypatch):
    from test_bottle_live_control import routine
    r, events, _ = routine(monkeypatch, bad_state=True)
    r.pause_handler = lambda *a, **k: pytest.fail('fatal faults cannot be resumed')
    report = r.execute()
    assert 'state disconnected' in report['error']
    assert 'home' not in events and 'retract' not in events and events[-1] == 'release'


def test_recording_round_trip_is_numeric_and_does_not_overwrite(tmp_path):
    path = tmp_path/'rgbd'
    recorder = TrialRecorder(path)
    frame = SimpleNamespace(frame_id=42, timestamp=123., color_bgr=np.zeros((3, 4, 3), np.uint8),
        depth_m=lambda: np.full((3, 4), .5), intrinsics=SimpleNamespace(fx=10., fy=11., cx=2., cy=1.))
    recorder.submit(frame, 1., motor_q_rad=np.zeros(29), imu_rpy_rad=[0, 0, 0])
    report = recorder.close()
    assert report['saved_frames'] == 1 and report['dropped_frames'] == 0 and not report['errors']
    with np.load(path/'000000042.npz', allow_pickle=False) as saved:
        assert saved['depth_m'].shape == (3, 4)
        metadata = json.loads(str(saved['metadata_json']))
        assert metadata['sequence'] == 42 and len(metadata['motor_q_rad']) == 29
        assert metadata['intrinsics']['fy'] == 11.
    with pytest.raises(FileExistsError):
        TrialRecorder(path)
