"""実際の3フレーム計測とFK収束を通し、段階接近の順序・再試行・中止を検査する。"""
from pathlib import Path
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import bottle
from alignment import SmoothNormalSettings
from bottle import BottleLocalizer, BottlePushRoutine, BottleSettings, STAGED_CLEARANCES_M
from test_bottle import Camera, target_at
from trial_policy import TrialPause


def staged_routine(monkeypatch, *, positions=(), pending=(), diagnostic=False, retries=2,
                   mode='staged', observe_fault=None):
    camera = Camera()
    events = []

    class Kin:
        data = SimpleNamespace(qpos=np.zeros(29))
        qadr = np.arange(22, 29)

        def __init__(self):
            self.tip = np.array([.1, -.25, .95])

        def tip_position(self): return self.tip.copy()
        def body_transform(self, _): return np.eye(4)
        def validate_path(self, *args): pass

    kin = Kin()

    class Localizer(BottleLocalizer):
        observation_index = 0

        def observe(self):
            self.observation_index += 1
            events.append(('observe', camera.t, self.last_sequence))
            if observe_fault is not None:
                observe_fault(self)
            if self.after_push:
                camera.x = (positions[-1] if positions else .37)+.02
            elif positions:
                camera.x = positions[min(self.observation_index-1, len(positions)-1)]
            return super().observe()

        def start_tracking(self, observation): events.append(('tracking', camera.t))
        def stop_tracking(self): events.append(('stop_tracking', camera.t))
        def latest_tracking(self, progress):
            return bottle.BottleObservation(self.previous, camera.t, self.last_sequence)
        def reset_measurement_reference(self): self.previous = None

    settings = BottleSettings(.75, push_distance=.02)
    localizer = Localizer(camera.read, camera, settings, camera.now)

    class Motion:
        release_error = None

        def __init__(self):
            self.target = np.r_[kin.tip, np.zeros(4)]
            self.home = self.target.copy()

        def current_target(self): return self.target.copy()
        def check(self): pass
        def enable(self): events.append(('enable', camera.t))
        def start_approach(self): pass
        def freeze_approach(self): pass
        def close(self): events.append(('release', camera.t))
        def wait(self, duration): camera.t += duration

        def stream(self, goal, **kwargs):
            self.target = np.array(goal)
            if localizer.observation_index not in pending:
                kin.tip = self.target[:3].copy()

    motion = Motion()

    class Filter:
        diagnostics = {}
        velocity = np.zeros(7)

        def __init__(self, *args): pass
        def next_goal(self, kin, xyz, normal, dt): return np.r_[xyz, np.zeros(4)]

    class Routine(BottlePushRoutine):
        def preview(self, observation):
            self.check_observation(observation)
            events.append(('preview', camera.t, observation.sequence))

        def move(self, xyz, face, duration=2., contact=False, *, settle_s=.3):
            events.append(('move', camera.t, float(face[0]-xyz[0]-self.radius), contact,
                           np.asarray(xyz).copy(), settle_s))
            kin.tip = np.asarray(xyz).copy()
            if localizer.observation_index in pending:
                kin.tip[1] += .004
            motion.target[:3] = kin.tip
            motion.wait(duration+settle_s)

        def return_home(self):
            events.append(('home', camera.t))
            self.report['home_completed'] = True

        def reverse_approach(self, face):
            events.append(('reverse', camera.t))
            self.return_home()

        def field_return(self, **kwargs): self.return_home()

    def normal(servo, now, wait, read_tip, send_goal, timeout, on_step):
        events.append(('push', camera.t, servo.penetration(kin.tip_position()), servo.stroke))
        return {'duration_s': 0.}

    monkeypatch.setattr(bottle, 'CartesianJointController', Filter)
    monkeypatch.setattr(bottle, 'run_continuous_normal', normal)
    r = Routine(kin, motion, localizer, lambda: None, camera.now, settings, .015,
                approach_mode=mode, approach_extra_retries=retries, live_tracking=True,
                stop_before_contact=diagnostic)
    return r, events, camera


def test_four_checkpoints_each_use_three_fresh_frames_and_final_push_starts_at_3cm(monkeypatch):
    r, events, camera = staged_routine(monkeypatch)
    report = r.execute()
    assert report['completed'] and report['visual_success'] and report['home_completed']
    rows = report['approach_steps']
    assert [row['clearance_m'] for row in rows] == list(STAGED_CLEARANCES_M)
    assert [row['sequence'] for row in rows] == [6, 9, 12, 15]
    assert report['approach_remeasurement_count'] == 4 and report['approach_extra_retry_count'] == 0
    assert all(row['accepted'] and row['fk_error_m'] <= .003 and row['duration_s'] > 0 for row in rows)
    # 計測開始後の3枚が必要。段階切替前に集めたフレームを混ぜない。
    assert all(capture['captured_at_s'] > started[1]
               for capture, started in zip(report['measurements'], [e for e in events if e[0] == 'observe']))
    for clearance in STAGED_CLEARANCES_M:
        alignment = [row for row in report['alignment_steps'] if row['clearance_m'] == clearance]
        assert len(alignment) >= 3 and all(np.linalg.norm(row['error_xyz_m']) <= .003 for row in alignment[-3:])
    assert all(event[5] == .6 for event in [e for e in events if e[0] == 'move'][1:9:2])
    push = next(event for event in events if event[0] == 'push')
    assert push[2:] == pytest.approx((-.03, .02))
    retract = next(event for event in events if event[0] == 'move' and event[3])
    assert retract[2] == pytest.approx(.05)
    assert events[-1][0] == 'release'


@pytest.mark.parametrize('positions,pending,reason', [
    ((.37, .38, .38), (), 'target_shift'),
    ((), (2,), 'fk_unsettled'),
])
def test_extra_measurement_stays_at_same_clearance_then_resumes(monkeypatch, positions, pending, reason):
    r, events, _ = staged_routine(monkeypatch, positions=positions, pending=pending)
    report = r.execute()
    assert report['completed'] and report['visual_success']
    rows = report['approach_steps']
    assert [row['clearance_m'] for row in rows] == [.10, .10, .07, .05, .03]
    assert rows[0]['retry_required'] == [reason] and not rows[0]['accepted']
    assert rows[1]['is_extra'] and rows[1]['retry_reason'] == [reason]
    assert rows[1]['accepted'] and report['approach_extra_retry_count'] == 1
    assert report['approach_remeasurement_count'] == 5


def test_extra_budget_is_shared_across_all_four_stages_and_exhaustion_holds(monkeypatch):
    r, events, _ = staged_routine(monkeypatch, pending=(2, 4, 6))
    pauses = []
    r.pause_handler = lambda pause, **kwargs: pauses.append((pause, kwargs)) or 'quit'
    report = r.execute()
    assert report['cancelled'] and not report['motion_completed']
    assert report['approach_extra_retry_count'] == 2
    assert [row['stage'] for row in report['approach_steps']] == [1, 1, 2, 2, 3]
    assert not report['approach_runs'][0]['completed']
    assert len(pauses) == 1 and pauses[0][0].code == 'alignment'
    assert pauses[0][1] == {'allow_resume': True}
    assert not any(event[0] in ('push', 'tracking') for event in events)


def test_zero_extra_budget_pauses_without_a_second_measurement(monkeypatch):
    r, _, _ = staged_routine(monkeypatch, pending=(2,), retries=0)
    r.pause_handler = lambda *args, **kwargs: 'quit'
    report = r.execute()
    assert report['approach_remeasurement_count'] == 1
    assert report['approach_extra_retry_count'] == 0 and not report['motion_completed']


def test_operator_resume_starts_new_four_stage_run_and_resets_only_its_retry_budget(monkeypatch):
    r, events, _ = staged_routine(monkeypatch, pending=(2, 3, 4, 6))
    pauses = []
    r.pause_handler = lambda pause, **kwargs: pauses.append(pause) or 'resume'
    report = r.execute()
    assert report['completed'] and report['visual_success']
    assert len(pauses) == 1
    assert sum(e[0] == 'enable' for e in events) == 1
    runs = report['approach_runs']
    assert [run['completed'] for run in runs] == [False, True]
    assert [run['extra_retry_count'] for run in runs] == [2, 1]
    assert report['approach_extra_retry_count'] == 3
    assert [row['stage'] for row in report['approach_steps'] if row['run'] == 2] == [1, 1, 2, 3, 4]


def test_fk_confirmation_requires_three_consecutive_samples(monkeypatch):
    r, _, _ = staged_routine(monkeypatch)
    face = np.array([.336, -.25, .83])
    goal = face-(r.radius+.07)*bottle.FORWARD
    errors = iter([0., 0., .004, 0., 0., 0.])

    def refresh():
        r.kin.tip = goal+np.array([0., next(errors), 0.])

    r.refresh = refresh
    r.settle_before_push(face, SmoothNormalSettings(), clearance=.07)
    assert len(r.report['alignment_steps']) == 6


def test_fk_error_above_8mm_is_fatal_instead_of_using_extra_budget(monkeypatch):
    r, events, _ = staged_routine(monkeypatch)
    r.pause_handler = lambda *args, **kwargs: pytest.fail('fatal FK error must not offer resume')

    def bad_alignment(*args, **kwargs):
        raise ValueError('ボトル手前のFK位置合わせ誤差が8mmを超えました')

    r.settle_before_push = bad_alignment
    report = r.execute()
    assert report['fatal_error'] and report['approach_extra_retry_count'] == 0
    assert len(report['approach_steps']) == 1 and not report['motion_completed']
    assert not any(event[0] == 'push' for event in events)


def test_diagnostic_finishes_all_stages_without_contact_or_tracking(monkeypatch):
    r, events, _ = staged_routine(monkeypatch, diagnostic=True)
    report = r.execute()
    assert report['diagnostic_completed'] and report['home_completed'] and report['release_completed']
    assert report['approach_remeasurement_count'] == 4
    assert not report['completed'] and not report['motion_completed'] and report['visual_success'] is None
    assert not any(event[0] in ('push', 'tracking') for event in events)
    assert not any(event[0] == 'move' and event[3] for event in events)


@pytest.mark.parametrize('fault', ['stale', 'lost', 'collision', 'state'])
def test_existing_faults_never_become_extra_alignment_retries(monkeypatch, fault):
    def fail(localizer):
        if localizer.observation_index == 2:
            if fault == 'stale':
                localizer.read_frame = lambda *args: bottle.replace(camera.read(*args), captured_at=camera.t-1.)
            else:
                raise {'lost': TrialPause('対象消失'), 'collision': ValueError('collision'),
                       'state': RuntimeError('state disconnected')}[fault]

    r, events, camera = staged_routine(monkeypatch, observe_fault=fail)
    r.pause_handler = lambda *args, **kwargs: 'quit'
    report = r.execute()
    assert not report['motion_completed'] and report['approach_extra_retry_count'] == 0
    assert len(report['approach_steps']) == 1
    assert not any(event[0] == 'push' for event in events)


def test_explicit_single_mode_keeps_one_remeasurement_and_5cm_start(monkeypatch):
    r, events, _ = staged_routine(monkeypatch, mode='single')
    report = r.execute()
    assert report['completed'] and report['visual_success']
    assert report['measurement_count'] == 3 and not report['approach_steps']
    assert report['approach_remeasurement_count'] == 1
    assert next(event for event in events if event[0] == 'push')[2] == pytest.approx(-.05)


def test_staged_preview_checks_all_checkpoint_paths_and_preserves_retract_distance():
    path = os.environ.get('G1_MUJOCO_MODEL')
    if not path:
        pytest.skip('G1_MUJOCO_MODELを指定してください')
    from kinematics import ArmKinematics, load_robot
    model, rest = load_robot(Path(path), (.11, -.003, 0), .79)
    kin = ArmKinematics(model)
    kin.update(rest)
    r = BottlePushRoutine(kin, None, None, lambda: None, lambda: 0., BottleSettings(.75, push_distance=.02),
                          .015, approach_mode='staged')
    report = r.preview(bottle.BottleObservation(target_at(.33), 0., 1))
    assert report['approach_clearances_m'] == list(STAGED_CLEARANCES_M)
    face = np.array(report['target']['face_xyz'])
    for name, clearance in [('transit', .10), ('approach_7cm', .07), ('approach_5cm', .05),
                            ('approach', .03), ('retract', .05)]:
        q = rest.copy()
        q[kin.qadr] = report['poses'][name]
        kin.update(q)
        assert kin.tip_position() == pytest.approx(face-(.015+clearance)*bottle.FORWARD, abs=.0001)
