"""Exercise new phase/feedback behavior with fakes; no robot or physics steps."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]),
                str(Path(__file__).resolve().parents[1]/'sim')]
import bottle
from alignment import ContinuousNormalController, SmoothNormalSettings, run_continuous_normal
from bottle import BottleObservation, BottlePushRoutine, BottleSettings
from test_bottle import target_at


def routine(monkeypatch, *, diagnostic=False, lost=False, bad_state=False,
            initial_age_limit=None, observation_age_s=0., preview_delay_s=0.):
    target = target_at(.37)
    clock = SimpleNamespace(t=1.)
    events, goals = [], []

    class Kin:
        data = SimpleNamespace(qpos=np.zeros(29))

        def __init__(self):
            self.tip = np.array(target.face_xyz)-[.065, 0., 0.]

        def tip_position(self):
            return self.tip.copy()

        def body_transform(self, name):
            return np.eye(4)

        def validate_path(self, *args):
            pass

    class Localizer:
        measurements, diagnostics, rejections = [], [], {}

        def observe(self):
            events.append('observe')
            if events.count('observe') == 3:
                assert 'home' in events
                moved = target_at(.42)
                return BottleObservation(moved, clock.t, 3)
            return BottleObservation(target, clock.t-observation_age_s, events.count('observe'))

        def start_tracking(self, observation):
            events.append('tracking')

        def latest_tracking(self, progress):
            if lost:
                raise RuntimeError('tracking lost')
            shifted = bottle.replace(target, center_xy=(.37, -.249))
            return BottleObservation(shifted, clock.t, 10)

        def stop_tracking(self):
            events.append('tracking_stopped')

        def expect_translation(self):
            events.append('expect_translation')

    class Motion:
        release_error = None

        def check(self):
            pass

        def enable(self):
            events.append('enable')

        def start_approach(self):
            pass

        def freeze_approach(self):
            pass

        def stream(self, goal, **kwargs):
            events.append('stream')

        def close(self):
            events.append('release')

        def wait(self, duration):
            clock.t += duration

    class Filter:
        diagnostics = {}
        velocity = np.zeros(7)

        def next_goal(self, kin, xyz, normal, dt):
            goals.append(xyz.copy())
            return np.zeros(7)

    class Routine(BottlePushRoutine):
        def preview(self, observation):
            self.check_observation(observation)
            clock.t += preview_delay_s
            return {}

        def move(self, xyz, *args, **kwargs):
            self.kin.tip = np.asarray(xyz)

        def settle_before_push(self, *args):
            return Filter()

        def reverse_approach(self, *args):
            events.append('reverse')
            self.report['home_completed'] = True

        def return_after_contact(self, *args):
            events.append('retract')
            self.report['retreat_completed'] = True

        def return_home(self):
            events.append('home')
            self.report['home_completed'] = True

    def refresh():
        if bad_state and 'pushing' in events:
            raise RuntimeError('state disconnected')

    def normal(servo, now, wait, read_tip, send_goal, timeout, on_step):
        events.append('pushing')
        read_tip()
        send_goal(servo.command_xyz, .02, 'contact')
        return {}

    monkeypatch.setattr(bottle, 'run_continuous_normal', normal)
    if initial_age_limit is not None:
        Routine.initial_plan_max_age_s = initial_age_limit
    r = Routine(Kin(), Motion(), Localizer(), refresh, lambda: clock.t, BottleSettings(.75), .015,
                live_tracking=True, stop_before_contact=diagnostic)
    return r, events, goals


def test_visual_feedback_restores_cylinder_line_and_verification_waits_for_home(monkeypatch):
    r, events, goals = routine(monkeypatch)
    report = r.execute()
    assert report['completed'] and report['visual_success'] and report['release_completed']
    assert events.index('home') < len(events)-1
    assert events.index('home') < events.index('expect_translation')
    assert -.25 < goals[0][1] <= -.245
    assert report['tracking_steps'][0]['sequence'] == 10
    assert events[-1] == 'release'


def test_approach_only_returns_without_tracking_or_contact_or_push_success(monkeypatch):
    r, events, goals = routine(monkeypatch, diagnostic=True)
    report = r.execute()
    assert report['diagnostic_completed'] and report['home_completed'] and report['release_completed']
    assert not report['completed'] and not report['motion_completed']
    assert report['visual_success'] is None
    assert 'reverse' in events and 'pushing' not in events and 'tracking' not in events
    assert not goals


@pytest.mark.parametrize('preview_delay_s', [.252, .302, .5])
def test_simulated_planning_delay_allows_up_to_075s_before_enabling(monkeypatch, preview_delay_s):
    from run_realistic_bottle import INITIAL_PLAN_MAX_AGE_S
    r, events, _ = routine(monkeypatch, initial_age_limit=INITIAL_PLAN_MAX_AGE_S,
                          observation_age_s=.25, preview_delay_s=preview_delay_s)
    report = r.execute()
    assert report['initial_plan_max_age_s'] == .75
    assert 'enable' in events and report['completed'] and report['visual_success']


def test_planning_delay_over_075s_releases_without_enabling_arm(monkeypatch):
    from run_realistic_bottle import INITIAL_PLAN_MAX_AGE_S
    r, events, goals = routine(monkeypatch, initial_age_limit=INITIAL_PLAN_MAX_AGE_S,
                              observation_age_s=.25, preview_delay_s=.501)
    report = r.execute()
    assert report['error'] == 'ボトル計測が古すぎます'
    assert 'enable' not in events and not goals and report['release_completed']


def test_real_default_still_rejects_measurement_aged_during_planning(monkeypatch):
    r, events, _ = routine(monkeypatch, observation_age_s=.25, preview_delay_s=.302)
    report = r.execute()
    assert report['initial_plan_max_age_s'] == .5
    assert report['error'] == 'ボトル計測が古すぎます' and 'enable' not in events


def test_extended_post_plan_limit_does_not_accept_an_already_stale_measurement(monkeypatch):
    from run_realistic_bottle import INITIAL_PLAN_MAX_AGE_S
    r, events, _ = routine(monkeypatch, initial_age_limit=INITIAL_PLAN_MAX_AGE_S, observation_age_s=.501)
    report = r.execute()
    assert report['error'] == 'ボトル計測が古すぎます' and 'enable' not in events


@pytest.mark.parametrize('bad_state', [False, True])
def test_loss_stops_push_and_only_healthy_state_allows_return(monkeypatch, bad_state):
    r, events, goals = routine(monkeypatch, lost=True, bad_state=bad_state)
    report = r.execute()
    assert not report['motion_completed'] and report['release_completed']
    assert ('home' in events) is (not bad_state)
    assert ('retract' in events) is (not bad_state)
    assert not goals


def test_control_rejects_a_slow_state_read_before_sending_any_goal():
    clock = SimpleNamespace(t=0.)
    servo = ContinuousNormalController([.3, -.25, .83], [1., 0., 0.], .015, .05,
        [.25, -.25, .83], SmoothNormalSettings(), travel_validator=bottle.validate_push_distance)
    sent = []

    def read():
        clock.t += .12
        return np.array([.25, -.25, .83])

    with pytest.raises(ValueError, match='状態取得が100ms'):
        run_continuous_normal(servo, lambda: clock.t, lambda _: None, read,
                              lambda *args: sent.append(args), 5., lambda _: None)
    assert not sent
