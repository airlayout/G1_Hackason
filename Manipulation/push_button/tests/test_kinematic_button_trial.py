"""Contracts for the opt-in kinematic-only real-robot prototype."""
from pathlib import Path
import argparse
import io
import json
import os
import subprocess
import sys
import tarfile
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('mujoco')
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'real')]
from robust_conditions import validate_button_plan
from robust_planner import SearchSettings
from robust_options import add_arguments, parse
from robust_scene import SceneSnapshot
import kinematic_button_planner as planner
import robust_session as session
from button_on_g1 import source_archive


def prototype_plan():
    settings = dict(button_policy='collection', button_feedback='joint', scenarios=1)
    trajectory = dict(candidate=0, q=[[0.]*7, [.01]*7], tcp=[[0.]*3, [.01]*3],
        phases=['approach', 'press'], press_start=0, press_end=1, retract_end=1,
        motion_metrics=dict(button_feedback=dict(version=1), button_tracking_settings=settings))
    return dict(schema='g1-kinematic-button-plan-v1', validation_method='kinematic',
        physical_simulation_performed=False, kinematic_checked=True, accepted=True,
        completed_rollouts=0, accepted_conditions=0, settings=settings,
        scene=dict(task='button', button_policy='collection', face=[.36,-.12,.98],
                   normal=[1.,0.,0.], stroke=.015), trajectory=trajectory,
        candidates=[dict(index=0, accepted=True, min_clearance_m=.003, min_joint_margin_rad=.025)])


def test_kinematic_contract_is_explicit_and_cannot_pass_physical_validation():
    plan = prototype_plan()
    validate_button_plan(plan, method='kinematic')
    with pytest.raises(ValueError, match='物理検証'):
        validate_button_plan(plan)
    plan['physical_simulation_performed'] = True
    with pytest.raises(ValueError):
        validate_button_plan(plan, method='kinematic')


def test_kinematic_contract_rejects_missing_or_unsafe_trajectory():
    plan = prototype_plan()
    plan['candidates'][0]['min_clearance_m'] = .002
    with pytest.raises(ValueError):
        validate_button_plan(plan, method='kinematic')
    plan = prototype_plan()
    plan['trajectory']['q'][0][0] = float('nan')
    with pytest.raises(ValueError):
        validate_button_plan(plan, method='kinematic')


def test_search_uses_static_candidate_without_running_physics(monkeypatch):
    settings = SearchSettings(scenarios=1, button_policy='collection', button_feedback='joint')
    scene = SceneSnapshot('button',(.36,-.12,.98),(1.,0.,0.),.035,(0.,)*29,(0.,)*3,0.,
                          stroke=.015,button_policy='collection')
    kin = SimpleNamespace(data=SimpleNamespace(qpos=np.zeros(29)), update=lambda _: None)
    candidate = SimpleNamespace(index=0, min_clearance_m=.01, min_joint_margin_rad=.1, duration_s=2.)
    calls = []
    monkeypatch.setattr(planner, 'build_scene_model', lambda *args: kin)
    monkeypatch.setattr(planner, 'generate_candidate', lambda *args: calls.append('static') or candidate)
    plan = planner.search_kinematic('unused', {}, scene, settings, monitor=lambda: calls.append('monitor'))
    assert plan.candidate is candidate and calls == ['monitor', 'static', 'monitor']
    assert plan.report['completed_rollouts'] == 0
    assert plan.report['physically_evaluated_candidates'] == 0
    assert plan.report['physical_simulation_performed'] is False


def test_launcher_requires_explicit_matching_validation_method():
    plan = prototype_plan()
    actual = dict(plan=plan, planning_completed=True)
    assert session.validate_preview(actual, None, .001, task='button', button_validation='kinematic')['validation_method'] == 'kinematic'
    with pytest.raises(ValueError):
        session.validate_preview(actual, None, .001, task='button')
    actual.update(completed=True, visual_success=True, execution=dict(motion_completed=True, release_completed=True))
    assert session.button_result_summary(actual, 'push', button_validation='kinematic')['completed']
    assert not session.button_result_summary(actual, 'push')['completed']


def test_trial_rejects_incompatible_kinematic_mode():
    calibration = dict(button_radius_m=.035, tip_radius_m=.015, position_uncertainty_m=.002)
    trial = dict(button_direction='up', button_validation='kinematic', button_policy='collection', button_feedback='joint')
    session.validate_button_trial(trial, calibration)
    assert trial['planning_scenarios'] == 1
    with pytest.raises(ValueError):
        session.validate_button_trial(dict(trial, button_policy='standard'), calibration)
    with pytest.raises(ValueError):
        session.validate_button_trial(dict(trial, button_feedback='off'), calibration)


def test_trial_config_passes_mode_to_child_and_g1_archive(tmp_path):
    trial = tmp_path/'trial.json'
    trial.write_text(json.dumps(dict(task='button', button_validation='kinematic',
        button_policy='collection', button_feedback='joint', planning_scenarios=1)))
    args = session.motion_arguments('push', trial, tmp_path/'result.json', socket_path='/tmp/test.sock', task='button')
    parser = argparse.ArgumentParser()
    parser.add_argument('--task')
    parser.add_argument('--output')
    parser.add_argument('--arm-socket')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--calibrated', action='store_true')
    parser.add_argument('--execute-verified-plan', action='store_true')
    add_arguments(parser)
    parsed, settings = parse(parser, args[1:])
    assert parsed.button_validation == 'kinematic' and settings.scenarios == 1
    with tarfile.open(fileobj=io.BytesIO(source_archive(ROOT.parents[1])), mode='r:gz') as archive:
        names = set(archive.getnames())
    assert 'Manipulation/push_button/kinematic_button_planner.py' in names
    assert 'Manipulation/push_button/real/run_kinematic_button_trial.sh' in names


def test_push_only_shell_forwards_requested_button_and_kinematic_mode(tmp_path):
    python = tmp_path/'python3'
    python.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
    python.chmod(0o755)
    shell = ROOT/'real/run_kinematic_button_trial.sh'
    result = subprocess.run(['bash', str(shell), '--button-direction', 'down', '--stroke-mm', '15'],
        env={**os.environ, 'PATH': str(tmp_path)+':'+os.environ['PATH']}, capture_output=True, text=True, check=True)
    args = result.stdout.splitlines()
    assert args.count('push') == 1
    assert args[args.index('--button-validation')+1] == 'kinematic'
    assert args[args.index('--button-direction')+1] == 'down'
    assert args[args.index('--stroke-mm')+1] == '15'


def test_standard_button_launcher_defaults_only_push_to_kinematic(tmp_path):
    python = tmp_path/'python3'
    python.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
    python.chmod(0o755)
    env = {**os.environ, 'PATH': str(tmp_path)+':'+os.environ['PATH']}
    shell = ROOT/'real/run_robust_button.sh'
    def launch(*args):
        return subprocess.run(['bash', str(shell), *args], env=env, capture_output=True,
                              text=True, check=True).stdout.splitlines()
    args = launch('--button-direction', 'down', 'push')
    assert args[args.index('--button-validation')+1] == 'kinematic'
    assert '--button-validation' not in launch('check')
    assert '--button-validation' not in launch('precontact')
    args = launch('push', '--button-validation', 'physical')
    choices = [args[i+1] for i, value in enumerate(args) if value == '--button-validation']
    assert choices == ['kinematic', 'physical']
