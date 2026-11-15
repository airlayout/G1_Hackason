"""Exercise the shared planner, walking gates and handoff with simulated inputs."""

from pathlib import Path
from types import SimpleNamespace
import json
import subprocess
import sys
import time

import numpy as np
import pytest


pytest.importorskip('mujoco')
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'real')]

import arm_bridge
from button_approach import ButtonApproach, plan_at_stops
from button_on_g1 import INSTALL, source_archive
from j1gen_camera import J1CameraRuntime
import kinematic_button_planner
from robust_perception import Observation
from robust_planner import SearchSettings
from robust_scene import SceneSnapshot
import robust_push


@pytest.fixture
def walking():
    runtime = J1CameraRuntime()
    config = runtime.module('config').load_config('approach.yaml')
    loco = runtime.module('loco')
    approach = ButtonApproach.__new__(ButtonApproach)
    approach.approach = runtime.module('approach')
    approach.cfg = config['approach']
    approach.stand = dict(min_dist_m=.30, max_dist_m=.50, center_side_m=-.14)
    approach.target_dist = .48
    approach.before_reverse = None
    approach.reverse_checked = False
    config['loco']['axes']['backward']['enabled'] = True
    backend = loco.FakeLoco()
    stillness = SimpleNamespace(is_still=backend.is_still, reset=lambda: None)
    approach.stillness = stillness
    approach.mover = loco.LocoMover(
        backend, stillness, config['loco'], go=True, sleep=backend.sleep, now=backend.now)
    return approach, backend


def measurement(approach, distance=.48, side=-.14, yaw=0.):
    normal = (np.cos(yaw), np.sin(yaw), 0.)
    call = SimpleNamespace(normal=normal, button=lambda _: SimpleNamespace(bbox=(20, 40, 30, 50)))
    return approach.approach.Measurement('buttons', distance, yaw, side, call=call)


def plan_result(status, *, seconds=7., scene=None):
    report = dict(search_status=status, total_wall_s=seconds, candidates_attempted=1,
                  candidates=[{'reason': 'joint limits'}])
    return SimpleNamespace(report=report, scene=scene,
                           candidate=SimpleNamespace(motion_metrics={}) if status == 'accepted' else None,
                           to_dict=lambda: report)


@pytest.mark.parametrize(('distance','side','yaw','expected_axis'), [
    (.58, -.14, 0., 'forward'),
    (.48, .02, 0., 'left'),
    (.48, -.32, 0., 'right'),
    (.48, -.14, .5, 'turn_left'),
    (.29, -.14, 0., 'backward'),
])
def test_corrects_and_replans_only_at_the_new_stop(walking, distance, side, yaw, expected_axis):
    approach, backend = walking
    observations = iter([measurement(approach, distance, side, yaw), measurement(approach)])
    accepted = plan_result('accepted')
    budgets = []

    def search(measured, budget):
        budgets.append(budget)
        return accepted if measured.distance_m == .48 and measured.side_m == -.14 else plan_result('candidates_exhausted')

    steps = []
    assert plan_at_stops(approach, lambda: next(observations), search, approach.walk,
                        steps=steps, budget_s=300.) is accepted
    assert steps[0]['pulse']['axis'] == expected_axis and steps[0]['stopped']
    assert steps[1]['decision'] == 'press' and len(backend.calls) == 1
    assert budgets == ([300.] if expected_axis in ('backward', 'turn_left') else [300., 293.])
    if expected_axis == 'backward':
        assert backend.calls[0][3] == .5


@pytest.mark.parametrize(('fault','reason','commands'), [
    ('too_close', '中止境界', 0),
    ('lost', '対象消失', 0),
    ('budget', '合計計画時間', 0),
    ('backward_did_not_help', '距離を確保', 1),
    ('still_moving', '脚が止まらない', 1),
    ('interrupt', 'interrupt', 1),
    ('preview', '', 0),
])
def test_approach_stops_before_enabling_the_arm(walking, monkeypatch, fault, reason, commands):
    approach, backend = walking
    distance = {'too_close': .26, 'backward_did_not_help': .29}.get(fault, .58)
    observed = measurement(approach, distance)
    if fault == 'preview':
        approach.mover.go = False
    elif fault == 'still_moving':
        backend.tail_s = 10.
    elif fault == 'interrupt':
        def interrupt(_):
            raise KeyboardInterrupt()

        monkeypatch.setattr(approach.mover, 'sleep', interrupt)
        original_stop = backend.stop
        stop_calls = []

        def stop():
            stop_calls.append(True)
            original_stop()
        monkeypatch.setattr(backend, 'stop', stop)

    def measure():
        if fault == 'lost':
            raise RuntimeError('対象消失')
        return observed

    def search(*_):
        return plan_result('budget_exhausted' if fault == 'budget' else 'candidates_exhausted')

    steps = []
    if fault == 'preview':
        assert plan_at_stops(approach, measure, search, approach.walk, steps=steps, budget_s=300.) is None
        assert steps[0]['pulse']['axis'] == 'forward'
    elif fault == 'interrupt':
        with pytest.raises(KeyboardInterrupt):
            plan_at_stops(approach, measure, search, approach.walk, steps=steps, budget_s=300.)
        assert stop_calls == [True]
    else:
        with pytest.raises(RuntimeError, match=reason):
            plan_at_stops(approach, measure, search, approach.walk, steps=steps, budget_s=300.)
    assert len(backend.calls) == commands


@pytest.mark.parametrize('outcome', ['press', 'corrected', 'drift', 'preview'])
def test_robust_entrypoint_uses_the_same_accepted_plan(walking, tmp_path, monkeypatch, outcome):
    approach, backend = walking
    pressing = outcome != 'preview'
    histories = []

    def measure(camera, configuration, history):
        histories.append(history)
        distance = .58 if outcome == 'corrected' and len(histories) == 1 else .48
        return measurement(approach, distance=distance)

    monkeypatch.setattr(approach, 'measure', measure)
    monkeypatch.setattr(approach, 'wait_still', lambda: None)
    monkeypatch.setattr('button_approach.ButtonApproach', lambda *a, **k: approach)
    rp = robust_push
    stamp = time.monotonic
    calibration = dict(tip_offset_m=(.1,0,0),tip_radius_m=.015,pelvis_height_m=.79,
                       position_uncertainty_m=.001,reference_waist_q_rad=[0]*3,reference_imu_rpy_rad=[0]*3)
    monkeypatch.setattr(rp, 'read_calibration', lambda *a, **k: (calibration, None))
    monkeypatch.setattr(rp, 'J1CameraRuntime', lambda *a: object())
    monkeypatch.setattr(rp, 'make_button_detector', lambda *a, **k: object())
    monkeypatch.setattr(rp, 'load_robot', lambda *a: (object(), np.zeros(29)))
    monkeypatch.setattr(rp, 'ArmKinematics', lambda *a: SimpleNamespace(update_fixed_base=lambda *a: None))
    monkeypatch.setattr(rp, 'CalibratedHeadCamera', lambda *a: object())
    monkeypatch.setattr(rp, 'J1RgbdCamera', lambda *a, **k: SimpleNamespace(close=lambda: None))
    monkeypatch.setattr(rp.time, 'sleep', lambda _: None)
    joints = np.zeros(29)
    arm = SimpleNamespace(current_configuration=lambda: (tuple(joints), (0.,)*3, stamp(), 0., 0.),
                          close=lambda: None, release=lambda: None, joint_feedback_state=lambda: {})
    monkeypatch.setattr(arm_bridge, 'ArmProxy', lambda _: arm)
    searches, motions, confirmations = [], [], []

    class Observer:
        frames = []
        rejections = {}

        def __init__(self, *_args, **_kwargs):
            pass

        def initial(self, *, timeout_s, after):
            confirmations.append(after)
            x = .38 if outcome == 'drift' and searches else .36
            return Observation(dict(face=(x,-.12,1.05),normal=(1.,0.,0.),radius=.035),
                               (0.,)*29,(0.,)*3,stamp(),3,100.,(1,2,3))

        def close(self):
            pass

    monkeypatch.setattr(rp, 'ContinuousObserver', Observer)

    def search(model, calibration, scene, settings, *, monitor):
        status = 'candidates_exhausted' if outcome == 'corrected' and not searches else 'accepted'
        candidate = plan_result(status, scene=scene)
        searches.append(candidate)
        return candidate

    def execute(plan, *a, **k):
        motions.append(plan)
        return dict(completed=True, motion_completed=True, lamp_early_return=True, release_completed=True)

    monkeypatch.setattr(kinematic_button_planner, 'search_kinematic', search)
    monkeypatch.setattr(rp, 'ArmMotion', lambda *a, **k: SimpleNamespace(
        upper_body_hold={}, command_publications=[], error=None, close=lambda: None))
    monkeypatch.setattr(rp, 'validate_button_plan', lambda *a, **k: None)
    monkeypatch.setattr(rp, 'execute', execute)
    monkeypatch.setattr(rp, 'collect_outcome_observations', lambda *a, **k: [])
    output = tmp_path/'result.json'
    args = ['--model','unused','--base-transform','unused','--stroke','.015','--arm-socket','unused',
            '--approach-button-set','wheelchair','--button-direction','down','--planning-scenarios','1',
            '--button-policy','collection','--button-feedback','joint','--button-press-mode','straight_trial',
            '--button-validation','kinematic','--output',str(output)]
    if pressing:
        args += ['--execute','--calibrated','--execute-verified-plan','--approach-confirmed']
    assert rp.main(args) == (2 if outcome == 'drift' else 0)
    if outcome == 'corrected':
        assert len(searches) == 2 and len(backend.calls) == 1
        assert len(histories) == 2 and histories[0] is not histories[1]
        assert len(confirmations) == 4
    else:
        assert len(searches) == 1 and backend.calls == []
        assert len(confirmations) == (2 if pressing else 1)
    assert len(motions) == (1 if outcome in ('press', 'corrected') else 0)
    if motions:
        assert motions[0] is searches[-1]
    result = json.loads(output.read_text())
    assert result['approach_steps'][-1]['decision'] == 'press'
    if outcome == 'drift':
        assert 'Target moved' in result['error']


@pytest.mark.parametrize(('failure', 'status'), [(ValueError, 'candidates_exhausted'), (TimeoutError, 'budget_exhausted')])
def test_kinematic_search_distinguishes_timeout_from_rejected_candidates(monkeypatch, failure, status):
    planner = kinematic_button_planner
    settings = SearchSettings(scenarios=1, button_policy='collection', button_feedback='joint', max_candidates=2)
    scene = SceneSnapshot('button',(.36,-.12,.98),(1.,0.,0.),.035,(0.,)*29,(0.,)*3,0.,
                          stroke=.015,button_policy='collection')
    kin = SimpleNamespace(data=SimpleNamespace(qpos=np.zeros(29)), update=lambda _: None)
    monkeypatch.setattr(planner, 'build_scene_model', lambda *a: kin)

    def reject(*_):
        raise failure('test candidate failure')

    monkeypatch.setattr(planner, 'generate_candidate', reject)
    plan = planner.search_kinematic('unused', {}, scene, settings)
    assert plan.candidate is None and plan.report['search_status'] == status
    assert plan.report['candidates_attempted'] == (2 if failure is ValueError else 1)


def test_approach_ssh_release_contains_j1_code_and_uses_runtime_assets(tmp_path):
    runtime = tmp_path/'runtime'
    base = runtime/'_local/robust_button'
    base.mkdir(parents=True)
    (base/'active_setup.json').write_text('{}')
    result = subprocess.run([sys.executable, '-c', INSTALL, str(runtime), 'test'],
                            input=source_archive(ROOT.parents[1], approach=True), capture_output=True, check=True)
    release = Path(json.loads(result.stdout)['release'])
    j1 = release/'Button_Press/J1-gen'
    assert (j1/'common/approach.py').is_file()
    assert (j1/'configs/approach.yaml').is_file()
    assert (j1/'weights').readlink() == runtime/'Button_Press/J1-gen/weights'
    assert (release/'_local').readlink() == runtime/'_local'
    assert (release/'Manipulation/push_button/real/run_robust_button_approach_press.sh').is_file()
