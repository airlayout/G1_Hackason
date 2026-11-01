"""Three-condition planning, contact exceptions and task outcome checks; no SDK IO."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robust_scene import SceneSnapshot, build_scene_model, contact_metrics
from robust_options import add_arguments, parse
from robust_planner import (Candidate, SearchSettings, uncertainty_scenarios, physical_trial,
                            generate_candidate, _trial_failure_reasons)
from kinematics import load_robot
from robust_execution import execute


def scene(task='bottle', **kwargs):
    return SceneSnapshot(task,(.326,-.12,.84),(1.,0.,0.),.034,(0.,)*29,(0.,)*3,0.,
                         base_height=.76 if task == 'bottle' else None,**kwargs)


def test_span_is_along_push_normal_and_does_not_rewrite_calibration():
    s = replace(scene(uncertainty_m=.015),normal=(np.cos(.1),np.sin(.1),0.))
    settings = SearchSettings(seed=3,position_span_m=.004)
    rows = uncertainty_scenarios(settings,s)
    np.testing.assert_allclose(rows[1]['position'],-.004*np.array(s.normal))
    np.testing.assert_allclose(rows[2]['position'],.004*np.array(s.normal))
    assert s.uncertainty_m == .015
    full = uncertainty_scenarios(settings,s,full_uncertainty=True,include_corners=False)
    assert max(abs(v) for row in full[1:] for v in row['position']) > .004
    for a,b in zip(rows,full):
        assert {k:v for k,v in a.items() if k != 'position'} == {k:v for k,v in b.items() if k != 'position'}


def test_new_settings_json_cli_precedence_and_validation(tmp_path):
    path = tmp_path/'trial.json'
    path.write_text(json.dumps(dict(planning_position_span_m=.006,bottle_feedback='relative')))
    parser = argparse.ArgumentParser(); add_arguments(parser)
    _, settings = parse(parser,['--trial-config',str(path),'--planning-position-span-m','.003',
                               '--bottle-feedback','off'])
    assert settings.position_span_m == .003 and settings.bottle_feedback == 'off'
    for value in (-.001,float('nan'),float('inf'),.021):
        with pytest.raises(ValueError): SearchSettings(position_span_m=value)
    with pytest.raises(ValueError): SearchSettings(bottle_feedback='unknown')
    # A conservative calibration bound is no longer a geometric pre-rejection.
    assert scene(uncertainty_m=.02).uncertainty_m == .02


def test_feedback_cannot_be_enabled_after_verifying_an_unmodified_reference():
    plan = SimpleNamespace(scene=scene(),candidate=SimpleNamespace(motion_metrics={}))
    with pytest.raises(RuntimeError,match='feedback mode differs'):
        execute(plan,'unused',{},None,lambda:pytest.fail('No hardware IO'),lambda:None,
                SearchSettings(bottle_feedback='relative'))


def test_robust_calibration_keeps_measured_uncertainty_without_legacy_margin_gate(tmp_path):
    sys.path.insert(0,str(ROOT/'real'))
    from align_button import read_calibration
    payload = dict(schema='g1-button-calibration-v1',T_base_optical=np.eye(4).tolist(),
        camera_body='torso_link',reference_waist_q_rad=[0,0,0],reference_imu_rpy_rad=[0,0,0],
        tip_radius_m=.015,button_radius_m=.025,position_uncertainty_m=.015)
    path = tmp_path/'calibration.json'; path.write_text(json.dumps(payload))
    with pytest.raises(ValueError): read_calibration(path)
    calibration,_ = read_calibration(path,require_target_margin=False)
    assert calibration['position_uncertainty_m'] == .015
    payload['button_radius_m'] = .014; path.write_text(json.dumps(payload))
    with pytest.raises(ValueError): read_calibration(path,require_target_margin=False)


@pytest.mark.parametrize('change,reason', [
    ({'clearance':.00299},'obstacle_clearance'),
    ({'tracking_max':.01001},'tracking_error'),
    ({'dynamic_joint_margin':.01999},'joint_margin'),
    ({'home_error':np.deg2rad(2.01)},'home_return'),
    ({'displacement':(.039,0,0)},'push_distance'),
    ({'displacement':(.061,0,0)},'push_distance'),
    ({'displacement':(.05,.01001,0)},'push_lateral'),
])
def test_bottle_failure_criteria_remain_enforced(change,reason):
    metrics = dict(displacement=(.05,0,0),peak_press=0.,peak_force=0.,clearance=.003,
                   tracking_max=.01,dynamic_joint_margin=.02,home_error=np.deg2rad(2))
    assert _trial_failure_reasons(scene(),**metrics) == []
    assert _trial_failure_reasons(scene(),**dict(metrics,**change)) == [reason]


@pytest.mark.parametrize('press,force,reason', [(.001,6.,'press_depth'),(.0017,6.,'press_depth'),
                                              (.0015,40.01,'press_force')])
def test_button_press_depth_overtravel_and_force_remain_enforced(press,force,reason):
    metrics = dict(displacement=(0,0,0),peak_press=press,peak_force=force,clearance=.003,
                   tracking_max=.005,dynamic_joint_margin=.03,home_error=0.)
    assert _trial_failure_reasons(scene('button'),**metrics) == [reason]


@pytest.mark.parametrize('collision', ['tip','forearm','wall'])
def test_contact_exception_is_only_for_tip_and_selected_target(collision):
    forearm_position = '0 0 0' if collision == 'forearm' else '0 .2 0'
    wall_position = '-.049 0 .7' if collision == 'wall' else '2 0 .7'
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <geom name="robust_target_geom" type="sphere" size=".035" pos="0 0 .7"/>
      <geom name="robust_wall" type="box" size=".1 .1 .1" pos="{wall_position}"/>
      <body pos="-.049 0 .7"><freejoint/>
        <geom name="tip" type="sphere" size=".015"/>
        <geom name="forearm" type="sphere" size=".015" pos="{forearm_position}"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model); mujoco.mj_forward(model,data)
    tip,forearm = model.geom('tip').id,model.geom('forearm').id
    kin = SimpleNamespace(model=model,data=data,tip_id=tip,arm_geoms={tip,forearm})
    with pytest.raises(ValueError,match='Unexpected arm contact'):
        contact_metrics(kin,scene(),contact=False)
    if collision == 'tip':
        assert contact_metrics(kin,scene(),contact=True) >= .003
    else:
        with pytest.raises(ValueError,match='Unexpected arm contact'):
            contact_metrics(kin,scene(),contact=True)


@pytest.fixture
def model_path():
    path = ROOT.parents[1]/'_local/bottle_push/menagerie/unitree_g1/g1.xml'
    if not path.exists(): pytest.skip('Menagerie asset not installed')
    return path


def test_saved_live_bottle_trajectory_passes_all_three_conditions_even_with_early_contact(model_path):
    root = ROOT/'LOG/robust-lab-20261101'
    saved = json.loads((root/'live/diagnostic_20261101_112438/result.json').read_text())
    s = SceneSnapshot(**saved['plan']['scene'])
    kin = build_scene_model(model_path,saved['calibration'],s)
    metadata = json.loads((root/'performance/baseline_candidate.json').read_text())
    with np.load(root/'performance/baseline_candidate.npz') as arrays:
        candidate = Candidate(**metadata,**{k:arrays[k] for k in ('q','tcp','tau')})
    # Contact permission must not depend on these scheduled phase labels.
    candidate.phases = ['approach']*len(candidate.q)
    settings = SearchSettings()
    for condition,expected_mm in zip(uncertainty_scenarios(settings,s),(47.38,50.36,44.33)):
        result = physical_trial(kin,s,candidate,condition,time.perf_counter()+30)
        assert result['completed'] and result['success'], result
        assert result['displacement_m'][0]*1000 == pytest.approx(expected_mm,abs=.1)
        assert result['min_clearance_m'] >= .003 and result['tracking_max_m'] <= .01
    assert s.uncertainty_m == .015
    assert not physical_trial(kin,s,candidate,condition,time.perf_counter()-1)['completed']


@pytest.mark.parametrize('height', [1.05,.98])
def test_button_three_conditions_complete_with_valid_depth_force_and_clearance(model_path,height):
    calibration = dict(tip_offset_m=(.1,0,0),pelvis_height_m=.79)
    _,q = load_robot(model_path,(.1,0,0),.79)
    s = replace(scene('button',uncertainty_m=.001),face=(.36,-.12,height),radius=.035,q=tuple(q))
    kin = build_scene_model(model_path,calibration,s)
    settings = SearchSettings()
    candidate = generate_candidate(kin,s,0,(0,0,0),(0,0),settings,time.perf_counter()+30)
    for condition in uncertainty_scenarios(settings,s):
        trial = physical_trial(kin,s,candidate,condition,time.perf_counter()+30)
        assert trial['completed'] and trial['success'], trial
        assert .9*s.stroke <= trial['peak_press_m'] <= .00165
        assert trial['peak_force_n'] <= 40. and trial['min_clearance_m'] >= .003
