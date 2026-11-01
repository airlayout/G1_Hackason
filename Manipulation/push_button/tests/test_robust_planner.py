"""Robust opt-in contracts and real B geometry; no hardware connection."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('mujoco')
sys.path[:0] = [str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[1]/'real')]
from robust_scene import SceneSnapshot, build_scene_model, estimate_bottle, contact_metrics
from robust_perception import Observation, ObservationWindow, CorrectionGate, LatestFrameSampler, ContinuousObserver
from robust_planner import SearchSettings, search, uncertainty_scenarios
from robust_options import add_arguments, parse, selected
from robust_execution import validate_start, execute
from kinematics import load_robot


def scene(**kwargs):
    return SceneSnapshot('bottle',(.326,-.12,.84),(1.,0.,0.),.034,(0.,)*29,(0.,0.,0.),1.,
                         base_height=.76,**kwargs)


def observation(s=None, *, frame=1, stamp=1., offset=(0,0,0)):
    s = s or scene()
    g = dict(face=tuple(np.array(s.face)+offset),normal=s.normal,radius=s.radius,base_height=s.base_height)
    return Observation(g,s.q,s.imu,stamp,frame)


def test_window_requires_three_unique_fresh_frames_and_rejects_identity_jump():
    w = ObservationWindow()
    assert w.consume(observation(frame=1,stamp=1),1) is None
    with pytest.raises(ValueError,match='duplicate'):
        w.consume(observation(frame=1,stamp=1),1)
    for i in (2,3):
        stamp = 1.+(i-1)*.1
        assert w.consume(observation(frame=i,stamp=stamp),stamp) is None
    assert w.consume(observation(frame=4,stamp=1.3),1.3) is not None
    with pytest.raises(ValueError,match='identity'):
        w.consume(observation(frame=5,stamp=1.4,offset=(0,.04,0)),1.4)
    with pytest.raises(ValueError,match='Stale'):
        w.consume(observation(frame=6,stamp=6),7)


def test_static_target_read_keeps_original_time_and_does_not_hide_missing_data_or_errors():
    import threading
    observer = ContinuousObserver.__new__(ContinuousObserver)
    observer.lock = threading.Lock()
    observer.error = None
    observer.now = lambda: 100.
    observer.value = observation(stamp=1.)
    with pytest.raises(RuntimeError, match='fresh'):
        observer.latest()
    held = observer.latest(allow_stale=True)
    assert held is observer.value and held.captured_at == 1.
    observer.value = None
    with pytest.raises(RuntimeError, match='observation'):
        observer.latest(allow_stale=True)
    observer.value = held
    observer.error = ValueError('camera failed')
    with pytest.raises(RuntimeError, match='camera failed'):
        observer.latest(allow_stale=True)


@pytest.mark.parametrize('immediate',[False,True])
def test_static_target_cannot_be_used_for_a_full_push(tmp_path,immediate):
    from robust_push import main
    output = tmp_path/'trial.json'
    code = main(['--task','bottle','--model','unused','--base-transform','unused',
        '--weights','unused','--static-target','--output',str(output)]+
        (['--execute','--calibrated','--execute-after-plan'] if immediate else []))
    assert code == 2
    assert '--stop-before-contact' in json.loads(output.read_text())['error']


def test_five_hz_frame_jitter_does_not_drop_every_other_image():
    sampler = LatestFrameSampler()
    frames = [('first', .85), ('second', 1.045), ('third', 1.249)]
    assert sampler.consume(frames[0], 1.) is frames[0]
    assert sampler.consume(frames[1], 1.195) is None
    # Reuse the original capture time, even when no new packet arrives at the tick.
    assert sampler.consume(None, 1.205) is frames[1]
    assert sampler.consume(frames[2], 1.399) is None
    assert sampler.consume(None, 1.410) is frames[2]
    assert sampler.consume(None, 1.7) is None
    assert sampler.consume(('older', 1.8), 1.8) == ('older', 1.8)
    assert sampler.consume(('pending', 1.9), 1.9) is None
    assert sampler.consume(('newest', 1.99), 1.99) is None
    assert sampler.consume(None, 2.01) == ('newest', 1.99)


def test_small_jitter_does_not_retarget_and_consistent_shift_is_bounded():
    s = scene(); gate = CorrectionGate(s)
    np.testing.assert_array_equal(gate.update(observation(offset=(0,.0005,0)),0),[0,0,0])
    out = gate.update(observation(frame=2,offset=(0,.003,0)),0)
    assert 0 < out[1] <= .0005
    np.testing.assert_array_equal(gate.update(observation(frame=3,offset=(0,.004,0)),0,freeze=True),out)
    with pytest.raises(RuntimeError,match='envelope'):
        gate.update(observation(frame=4,offset=(0,.01,0)),0)
    with pytest.raises(RuntimeError,match='press direction'):
        gate.update(observation(frame=5,offset=(.02,0,0)),0,freeze=True)


def test_snapshot_age_is_not_execution_expiry_but_movement_invalidates():
    s = scene()
    config = (s.q,s.imu,100.,0.,0.)
    validate_start(s,config,observation(stamp=100))
    changed = np.array(s.q); changed[22] += .02
    with pytest.raises(RuntimeError,match='moved'):
        validate_start(s,(changed,s.imu,100.,0.,0.),observation(stamp=100))
    observed = observation()
    lifted = replace(observed,geometry=dict(observed.geometry,object_height=s.object_height+.02))
    with pytest.raises(RuntimeError,match='vertical'):
        validate_start(s,config,lifted)
    with pytest.raises(RuntimeError,match='vertical'):
        CorrectionGate(s).update(lifted,0)


def test_missing_bottom_and_transparent_depth_reject_without_table_fallback():
    from vision import CameraIntrinsics
    with pytest.raises(ValueError,match='depth'):
        estimate_bottle(np.zeros((100,100)),(20,20,80,90),CameraIntrinsics(100,100,50,50),np.eye(4))
    with pytest.raises(ValueError,match='bottom'):
        replace(scene(),base_height=None)


@pytest.mark.parametrize('centre_y', [-.215, .215])
@pytest.mark.parametrize('narrow_neck', [False, True])
def test_measured_bottom_separates_oblique_bottle_from_support(centre_y, narrow_neck):
    from vision import CameraIntrinsics
    # Independent ray intersections: the projected cylinder is diagonal, leaving
    # more than half its axis-aligned detector box occupied by the support plane.
    v, u = np.mgrid[:360, :640]
    intrinsics = CameraIntrinsics(454., 454., 319., 191.)
    angle = .83
    transform = np.array([[0., -np.sin(angle), np.cos(angle), .06],
                          [-1., 0., 0., 0.],
                          [0., -np.cos(angle), -np.sin(angle), 1.24],
                          [0., 0., 0., 1.]])
    directions = np.stack(((u-319)/454., (v-191)/454., np.ones_like(u)), axis=-1)
    directions = directions @ transform[:3, :3].T
    origin = transform[:3, 3]
    a = np.sum(directions[:, :, :2]**2, axis=-1)
    b = 2*((origin[0]-.4)*directions[:, :, 0]+(origin[1]-centre_y)*directions[:, :, 1])
    support = (.72-origin[2])/directions[:, :, 2]
    support[support <= 0] = np.inf
    depth = np.minimum(support, 1.5)
    visible = np.zeros_like(depth, dtype=bool)
    surfaces = [(.034, .72, .88), (.012, .88, .94)] if narrow_neck else [(.034, .72, .94)]
    for radius, lower, upper in surfaces:
        c = (origin[0]-.4)**2+(origin[1]-centre_y)**2-radius**2
        discriminant = b*b-4*a*c
        distance = (-b-np.sqrt(np.maximum(discriminant, 0)))/(2*a)
        height = origin[2]+distance*directions[:, :, 2]
        hit = ((discriminant > 0) & (distance > 0) & (height > lower)
               & (height < upper) & (distance < depth))
        depth[hit] = distance[hit]
        visible |= hit
    yy, xx = np.where(visible)
    bbox = (xx.min(), yy.min(), xx.max()+1, yy.max()+1)
    x0, y0, x1, y1 = bbox
    assert np.mean(~visible[y0:y1, x0:x1]) > .5
    observed = estimate_bottle(depth, bbox, intrinsics, transform, base_height=.72)
    assert observed['face'] == pytest.approx((.366, centre_y, .8), abs=.001)
    assert observed['radius'] == pytest.approx(.034, abs=.001)
    assert observed['object_height'] == pytest.approx(.22, abs=.01)
    # A measured support height alone never provides evidence of a bottle.
    with pytest.raises(ValueError):
        estimate_bottle(np.minimum(support, 1.5), bbox, intrinsics, transform, base_height=.72)
    with pytest.raises(ValueError):
        estimate_bottle(np.full_like(depth, .37), bbox, intrinsics, transform, base_height=.72)


def test_trial_config_cli_precedence_and_opt_in(tmp_path):
    path = tmp_path/'trial.json'
    path.write_text(json.dumps(dict(motion_planner='robust',planning_budget_s=6,planning_scenarios=2)))
    parser = argparse.ArgumentParser(); add_arguments(parser)
    args, settings = parse(parser,['--trial-config',str(path),'--planning-budget-s','3'])
    assert selected(['--trial-config',str(path)])
    assert not selected(['--trial-config',str(path),'--motion-planner','legacy'])
    assert args.planning_budget_s == settings.budget_s == 3 and settings.scenarios == 2
    assert not selected([])
    with pytest.raises(ValueError):
        SearchSettings(budget_s=float('nan'))
    with pytest.raises(ValueError,match='integers'):
        SearchSettings(scenarios=1.5)


def test_uncertainty_is_deterministic_nominal_first_and_bias_fixed_per_trial():
    rows = uncertainty_scenarios(SearchSettings(seed=123),scene())
    assert rows == uncertainty_scenarios(SearchSettings(seed=123),scene())
    assert rows[0]['position'] == [0,0,0]
    assert rows[1]['position'] == [-.003,0,0]
    assert rows[2]['position'] == [.003,0,0]
    assert len(rows) == 3


@pytest.fixture
def robot():
    path = Path(__file__).resolve().parents[3]/'_local/bottle_push/menagerie/unitree_g1/g1.xml'
    if not path.exists():
        pytest.skip('Menagerie asset not installed')
    cal = dict(tip_offset_m=(.1,0,0),pelvis_height_m=.79)
    _,q = load_robot(path,(.1,0,0),.79)
    return path,cal,replace(scene(),q=tuple(q))


def test_b_wall_is_fixed_low_front_is_free_but_back_is_blocked(robot):
    import mujoco
    path,cal,s = robot
    kin = build_scene_model(path,cal,s)
    wall = kin.model.geom('robust_wall').id
    wall_pos = ' '.join(map(str,kin.model.geom_pos[wall]))
    wall_size = ' '.join(map(str,kin.model.geom_size[wall]))
    # Probe the actual generated wall, independently of the robot's starting pose.
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <geom name="wall" type="box" pos="{wall_pos}" size="{wall_size}"/>
      <body name="probe"><freejoint/><geom name="tip" type="sphere" size=".015"/></body>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    for position,blocked in [((s.wall_x-.05,0,.60),False),((s.wall_x+.05,0,.60),True),
                             ((s.wall_x+.05,0,.84),False)]:
        data.qpos[:3] = position; mujoco.mj_forward(model,data)
        dist = mujoco.mj_geomDistance(model,data,0,1,1.,None)
        assert (dist < 0) == blocked
    frozen = kin.model.geom_pos[wall].copy()
    adr = kin.model.joint('robust_object_joint').qposadr[0]
    kin.data.qpos[adr] += .05; mujoco.mj_forward(kin.model,kin.data)
    np.testing.assert_array_equal(kin.model.geom_pos[wall],frozen)


def test_absolute_imu_yaw_is_not_floor_frame_heading(robot):
    path,cal,s = robot
    kin = build_scene_model(path,cal,replace(s,imu=(0.,0.,1.2)))
    np.testing.assert_allclose(kin.model.body_quat[kin.model.body('pelvis').id],[1.,0.,0.,0.])


def test_deadline_returns_no_plan_and_never_commands_hardware(robot):
    path,cal,s = robot
    plan = search(path,cal,s,SearchSettings(budget_s=.00001,scenarios=1))
    assert plan.candidate is None and plan.report['completed_rollouts'] == 0
    with pytest.raises(RuntimeError,match='No fully verified'):
        execute(plan,path,cal,SimpleNamespace(),lambda:pytest.fail('no IO'),lambda:None,SearchSettings())


def test_real_robust_parser_is_importable_without_sdk():
    import robust_push
    assert callable(robust_push.main)


@pytest.mark.parametrize('incomplete,expired', [(False,False),(True,False),(False,True)])
def test_search_stops_at_first_complete_success_and_caps_workers(monkeypatch,incomplete,expired):
    import robust_planner as planner
    from concurrent.futures import Future
    dummy = SimpleNamespace(data=SimpleNamespace(qpos=np.zeros(29)),update=lambda q:None)
    monkeypatch.setattr(planner,'build_scene_model',lambda *a:dummy)
    monkeypatch.setattr(planner,'generate_candidate',lambda kin,s,index,*a:SimpleNamespace(
        index=index,min_joint_margin_rad=.1,duration_s=100-index))
    clock = [0.]
    monkeypatch.setattr(planner.time,'perf_counter',lambda:clock[0])
    class Pool:
        def __init__(self,**kwargs): assert kwargs['max_workers'] == 2
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def submit(self,fn,s,c,condition,deadline):
            if expired:
                clock[0] = 11.
            result = Future()
            result.set_result(dict(completed=not incomplete,success=True,min_clearance_m=.004,
                                   target_margin_m=-.01,min_joint_margin_rad=.1))
            return result
    monkeypatch.setattr(planner,'ProcessPoolExecutor',Pool)
    result = planner.search('unused',{},scene(),SearchSettings(max_candidates=3,scenarios=2,max_rollouts=6))
    if incomplete or expired:
        assert result.candidate is None
    else:
        assert result.candidate.index == 0
        assert len(result.report['candidates']) == 1
        assert result.report['completed_rollouts'] == 2
        assert result.report['runtime']['workers'] == 2
    assert not result.report['target_1000_met']


def test_preview_uses_no_arm_or_execute_path(monkeypatch,tmp_path):
    import robust_push as rp
    calibration = dict(tip_offset_m=(.1,0,0),tip_radius_m=.015,pelvis_height_m=.79,
                       position_uncertainty_m=.001,reference_imu_rpy_rad=[0,0,0])
    monkeypatch.setattr(rp,'read_calibration',lambda *a,**k:(calibration,None))
    class Detector:
        def __init__(self,**kwargs): pass
        def detect(self,image): return []
    runtime = SimpleNamespace(module=lambda n:SimpleNamespace(perception=lambda n:SimpleNamespace(YoloDetector=Detector)))
    monkeypatch.setattr(rp,'J1CameraRuntime',lambda *a:runtime)
    monkeypatch.setattr(rp,'load_robot',lambda *a:(object(),np.zeros(29)))
    monkeypatch.setattr(rp,'ArmKinematics',lambda m:SimpleNamespace(update_fixed_base=lambda *a:None))
    monkeypatch.setattr(rp,'CalibratedHeadCamera',lambda *a:object())
    monkeypatch.setattr(rp,'J1RgbdCamera',lambda *a:SimpleNamespace(close=lambda:None))
    class Observer:
        frames,rejections = [],{}
        def __init__(self,*a): pass
        def initial(self): return observation()
        def close(self): pass
    monkeypatch.setattr(rp,'ContinuousObserver',Observer)
    monkeypatch.setattr(rp,'search',lambda *a,**k:SimpleNamespace(candidate=object(),to_dict=lambda:dict(accepted=True)))
    monkeypatch.setattr(rp,'ArmMotion',lambda *a,**k:pytest.fail('preview opened arm'))
    state = tmp_path/'state.json'
    state.write_text(json.dumps(dict(motor_q_rad=[0]*29,imu_rpy_rad=[0]*3)))
    output = tmp_path/'result.json'
    assert rp.main(['--task','bottle','--model','unused','--base-transform','unused','--weights','unused',
                    '--state-json',str(state),'--output',str(output)]) == 0
    data = json.loads(output.read_text())
    assert data['planning_completed'] and not data['completed'] and data['physical_push_success'] is None
