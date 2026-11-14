"""Button policy, condition-specific depth bounds and unchanged physical trajectories."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('mujoco')
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'real')]
from robust_options import add_arguments,parse
from robust_planner import SearchSettings,button_press_depth_bounds,_trial_failure_reasons,search,uncertainty_scenarios
from robust_policy import execution_policy
from robust_scene import SceneSnapshot
from robust_conditions import (BUTTON_CONDITION_NAMES,button_condition_positions,
                               validate_button_plan,validate_button_scenario_settings)
import robust_session as session


def button(**values):
    return replace(SceneSnapshot('button',(.36,-.12,.98),(1.,0.,0.),.035,(0.,)*29,(0.,)*3,0.,stroke=.015),**values)


def test_button_defaults_cli_json_precedence_and_bottle_compatibility(tmp_path):
    def parser():
        p=argparse.ArgumentParser();p.add_argument('--task',default='button');add_arguments(p);return p
    args,settings=parse(parser(),[])
    assert args.stroke==.015 and settings.budget_s==30 and settings.max_candidates==50
    assert settings.scenarios==4
    args,settings=parse(parser(),['--task','bottle'])
    assert args.stroke==.0015 and settings.budget_s==10 and settings.scenarios==3
    path=tmp_path/'trial.json'
    path.write_text(json.dumps(dict(task='button',stroke=.004,planning_budget_s=12)))
    args,settings=parse(parser(),['--trial-config',str(path),'--stroke','.006','--planning-budget-s','20'])
    assert args.stroke==.006 and settings.budget_s==20
    args,settings=parse(parser(),['--trial-config',str(path)])
    assert args.stroke==.004 and settings.budget_s==12


def test_launcher_fills_missing_defaults_but_rejects_explicit_null():
    calibration=dict(button_radius_m=.035,tip_radius_m=.015,position_uncertainty_m=.002)
    trial=dict(button_direction='up')
    session.validate_button_trial(trial,calibration)
    assert trial['stroke']==.015 and trial['planning_budget_s']==30 and trial['planning_max_candidates']==50
    assert trial['planning_scenarios']==4
    with pytest.raises(ValueError,match='stroke'):
        session.validate_button_trial(dict(trial,stroke=None),calibration)


def test_preview_and_session_separate_selected_conditions_from_total_trials():
    plan=valid_plan()
    plan.update(completed_rollouts=80,candidates_attempted=50,physically_evaluated_candidates=20,
        condition_trials_started=80)
    preview=session.validate_preview(dict(plan=plan,planning_completed=True),None,.001,task='button')
    assert preview['completed_conditions']==4
    summary=session.policy_summary(plan)
    assert summary['completed_conditions']==4 and summary['completed_rollouts']==80
    assert summary['candidates_attempted']==50 and summary['physically_evaluated_candidates']==20


def test_direct_button_entrypoint_logs_the_effective_defaults_before_connecting(tmp_path):
    from robust_push import main
    output=tmp_path/'result.json'
    assert main(['--task','button','--model','unused','--base-transform','unused','--output',str(output)])==2
    report=json.loads(output.read_text())
    assert report['arguments']['stroke']==.015 and report['settings']['budget_s']==30
    assert report['settings']['scenarios']==4
    assert report['error']=='Planning preview requires --state-json'


@pytest.mark.parametrize('offset,stroke,expected',[
    ((0.,0.,0.),.015,.0135),((-.003,0.,0.),.015,.0135),((.003,0.,0.),.015,.01175),
    ((.002,0.,0.),.015,.0135),((.004,0.,0.),.015,.0135),((.003,.0001,0.),.015,.0135),
    ((.003,0.,0.),.0015,.00135),((.003,0.,0.),.014,.0126)])
def test_only_the_15mm_button_with_exact_far_displacement_is_relaxed(offset,stroke,expected):
    lower,upper=button_press_depth_bounds(button(stroke=stroke),dict(index=99,position=offset))
    assert lower==pytest.approx(expected)
    assert upper==pytest.approx(stroke+min(.001,stroke/10))


def test_depth_bound_uses_panel_normal_and_not_scenario_number():
    scene=button(normal=(.96,.28,0.))
    assert button_press_depth_bounds(scene,dict(index=0,position=(.00288,.00084,0.)))[0]==.01175
    assert button_press_depth_bounds(scene,dict(index=2,position=(.003,0.,0.)))[0]==pytest.approx(.0135)
    assert button_press_depth_bounds(scene)[0]==pytest.approx(.0135)
    assert button_press_depth_bounds(replace(scene,task='bottle',base_height=.76),
        dict(position=(.00288,.00084,0.)))[0]==pytest.approx(.0135)


def failures(peak,**changes):
    metrics=dict(displacement=np.zeros(3),peak_press=peak,peak_force=6.,clearance=.02,
        tracking_max=.001,dynamic_joint_margin=.1,home_error=0.)
    metrics.update(changes)
    return _trial_failure_reasons(button(),condition=dict(position=(.003,0.,0.)),**metrics)


def test_far_depth_lower_and_upper_boundaries():
    assert failures(.01175)==[] and failures(.016)==[]
    assert failures(.0117499)==['press_depth']
    assert failures(.0160001)==['press_depth']


@pytest.mark.parametrize('change,reason',[(dict(peak_force=40.01),'press_force'),
    (dict(tracking_max=.01001),'tracking_error'),(dict(clearance=.00299),'obstacle_clearance'),
    (dict(dynamic_joint_margin=.01999),'joint_margin'),(dict(home_error=np.deg2rad(3)),'home_return')])
def test_far_depth_relaxation_never_removes_other_failure_reasons(change,reason):
    assert reason in failures(.012,**change)


def test_robot_policy_keeps_strict_defaults_and_uses_one_shared_collection_profile():
    strict=execution_policy();collection=execution_policy(True)
    assert strict.state_timeout_s==.1 and strict.leg_speed_rad_s==.15
    assert collection.state_timeout_s==.4 and collection.leg_speed_rad_s==1.0 and collection.gyro_rad_s==.3
    assert collection.record_deviations and collection.synchronous_stream


@pytest.mark.parametrize('age,leg,gyro,tilt',[(.401,0,0,0),(.0,1.001,0,0),(.0,0,.301,0),(.0,0,0,.151)])
def test_collection_motion_monitor_still_rejects_expiry_speed_and_non_upright_pose(monkeypatch,age,leg,gyro,tilt):
    import align_button
    monkeypatch.setattr(align_button.time,'monotonic',lambda:100.)
    config=([0.]*29,[tilt,0,0],100.-age,leg,gyro)
    with pytest.raises(RuntimeError):
        align_button.validate_stationary_pose(config,{},allow_pose_drift=True)


def test_synchronous_stream_publishes_each_checked_command_once_with_waist_torque(monkeypatch):
    import align_button
    clock=[100.];sent=[]
    monkeypatch.setattr(align_button,'time',SimpleNamespace(monotonic=lambda:clock[0]))
    class Arm:
        def configure_motion_limits(self,limits): return limits.to_dict()
        def current_arm(self): return (0.,)*14
        def set_hold(self,current): return dict(held=True)
        def current_configuration(self): return ([0.]*29,[0.]*3,clock[0],0.,0.)
        def publish(self,q,weight,**kwargs): sent.append((clock[0],tuple(q),kwargs))
        def release(self): pass
    motion=align_button.ArmMotion(Arm(),{},limits=SearchSettings().motion_limits,
        allow_pose_drift=True,synchronous_stream=True,waist_gravity=lambda config:[1,2,3])
    motion.wait=lambda dt:clock.__setitem__(0,clock[0]+dt)
    motion.weight=1.;motion.streaming=True
    for i in (1,2,3): motion.stream([i*.001,0,0,0,0,0,0],velocity=[.05,0,0,0,0,0,0])
    assert [row[1][0] for row in sent]==[.001,.002,.003]
    assert len(motion.command_publications)==3
    assert all(row[2]['waist_tau']==(1.,2.,3.) for row in sent)
    assert np.all(np.diff([row[0] for row in sent])>=.02-1e-10)


def test_15mm_button_first_valid_and_all_50_candidates_reproduce_the_experiment(tmp_path):
    from kinematics import load_robot
    model=ROOT.parents[1]/'_local/bottle_push/menagerie/unitree_g1/g1.xml'
    if not model.is_file(): pytest.skip('Menagerie asset not installed')
    _,q=load_robot(model,(.1,0,0),.79)
    scene=button(q=tuple(q),uncertainty_m=.001)
    calibration=dict(tip_offset_m=(.1,0,0),pelvis_height_m=.79)
    settings=SearchSettings(budget_s=30.,max_candidates=50,scenarios=4)
    first=search(model,calibration,scene,settings)
    assert first.candidate is not None and first.candidate.index==0
    assert first.report['candidates_attempted']==1 and first.report['condition_trials_started']==4
    assert first.report['accepted_conditions']==4 and first.report['first_accepted_s'] is not None
    validate_button_plan(first.to_dict())
    all_candidates=search(model,calibration,scene,settings,evaluate_all=True)
    report=all_candidates.to_dict()
    (tmp_path/'button-15mm-50-candidates.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    assert [row['index'] for row in report['candidates'] if row.get('accepted')]==[0,4,13,21,28,32,36]
    assert report['candidates_attempted']<=50 and report['physically_evaluated_candidates']<=20
    assert report['condition_trials_started']==report['physically_evaluated_candidates']*4
    assert report['completed_rollouts']+report['aborted_rollouts']==report['condition_trials_started']
    assert report['trajectory']['candidate']==0
    validate_button_plan(report)
    np.testing.assert_array_equal(first.candidate.q,all_candidates.candidate.q)
    assert first.candidate.motion_metrics['button_press']['reference_depth_m']==.020
    for row in report['candidates']:
        for i,trial in enumerate(row.get('trials',[])):
            if not trial.get('completed'): continue
            assert trial['press_depth_lower_m']==pytest.approx(.01175 if i==1 else .0135)
            assert trial['press_depth_upper_m']==.016


def valid_plan():
    scene=button()
    conditions=uncertainty_scenarios(SearchSettings(scenarios=4),scene)
    trials=[dict(completed=True,success=True,condition_name=c['name'],condition_position_m=c['position'])
        for c in conditions]
    return dict(accepted=True,accepted_conditions=4,completed_rollouts=4,scene=scene.to_dict(),
        conditions=conditions,trajectory=dict(candidate=0,q=[[0.]*7]),
        candidates=[dict(index=0,accepted=True,trials=trials)])


@pytest.mark.parametrize('normal',[(1.,0.,0.),(.96,.28,0.),(.96,0.,.28)])
def test_four_conditions_follow_the_panel_and_preserve_previous_random_samples(normal):
    scene=button(normal=normal)
    settings=SearchSettings(scenarios=4,seed=123)
    conditions=uncertainty_scenarios(settings,scene)
    assert conditions==uncertainty_scenarios(settings,scene)
    assert tuple(c['name'] for c in conditions)==BUTTON_CONDITION_NAMES
    positions=np.array([c['position'] for c in conditions])
    np.testing.assert_allclose(positions[1],.003*np.array(normal),atol=1e-12)
    np.testing.assert_allclose(positions[2],-positions[3],atol=1e-12)
    assert np.linalg.norm(positions[2])==pytest.approx(.003)
    assert positions[2]@normal==pytest.approx(0.,abs=1e-12) and positions[2][2]==0
    assert positions[2][1]>0
    old=uncertainty_scenarios(replace(settings,scenarios=5),scene)
    for current,prior in zip(conditions,[old[i] for i in (0,2,3,4)]):
        for key in ('angles','tracking','mass','friction'): assert current[key]==prior[key]
    random=uncertainty_scenarios(settings,scene,full_uncertainty=True)
    assert len(random)==4 and all('name' not in row for row in random)
    bottle=replace(scene,task='bottle',base_height=.76)
    old_bottle=uncertainty_scenarios(replace(settings,scenarios=3),bottle)
    assert len(old_bottle)==3 and all('name' not in row for row in old_bottle)
    np.testing.assert_allclose(old_bottle[1]['position'],-.003*np.array(normal))


@pytest.mark.parametrize('count,span',[(3,.003),(4,.002),(4,.004)])
def test_nonstandard_button_conditions_are_rejected(count,span):
    with pytest.raises(ValueError,match='4条件'):
        validate_button_scenario_settings(count,span)


@pytest.mark.parametrize('verified',[False,True])
def test_old_three_condition_button_execution_is_rejected_before_preparation(tmp_path,monkeypatch,verified):
    import robust_push
    monkeypatch.setattr(robust_push,'read_calibration',lambda *a,**k:pytest.fail('Old plan reached preparation'))
    output=tmp_path/'rejected.json'
    args=['--task','button','--model','unused','--base-transform','unused','--execute','--calibrated',
        '--planning-scenarios','3','--output',str(output)]
    if verified: args.append('--execute-verified-plan')
    assert robust_push.main(args)==2
    assert '4' in json.loads(output.read_text())['error']


@pytest.mark.parametrize('fault',['legacy','names','position','trial_position','incomplete','failed','missing_trial'])
def test_four_condition_plan_validation_rejects_stale_or_incomplete_results(fault):
    plan=valid_plan()
    if fault=='legacy': plan['accepted_conditions']=3
    if fault=='names': plan['conditions'][2]['name']='near_3mm'
    if fault=='position': plan['conditions'][2]['position']=[-.003,0,0]
    if fault=='trial_position': plan['candidates'][0]['trials'][2]['condition_position_m']=[-.003,0,0]
    if fault=='incomplete': plan['candidates'][0]['trials'][2]['completed']=False
    if fault=='failed': plan['candidates'][0]['trials'][2]['success']=False
    if fault=='missing_trial': plan['candidates'][0]['trials'].pop()
    with pytest.raises(ValueError,match='4条件'):
        validate_button_plan(plan)
    summary=session.button_result_summary(dict(completed=True,visual_success=True,plan=plan,
        execution=dict(motion_completed=True,release_completed=True,home_completed=True)),'push')
    assert summary['completed'] is False and summary['plan_validation_error']
