"""Feedback invariants, policy boundaries and launch integration without hardware."""
import io
import json
from dataclasses import asdict, replace
from pathlib import Path
import socket
import subprocess
import sys
import tarfile
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT),str(ROOT/'real')]
from button_tracking import (TorqueIntegral, BIAS_LIMIT, BIAS_SLEW, TORQUE_LIMIT,
                             ButtonTrackingController, controller_metadata)
from robust_scene import SceneSnapshot, build_scene_model, update_robot
from robust_planner import Candidate, SearchSettings, uncertainty_scenarios
from robust_conditions import validate_button_plan, validate_button_scenario_settings
from robust_perception import Observation, ObservationWindow
from kinematics import load_robot
from test_robust_button import launcher
import robust_session as session
import button_on_g1


def test_integral_is_slewed_bounded_and_does_not_integrate_cached_state():
    integral = TorqueIntegral()
    kwargs = dict(depth=-.1,stroke=.015)
    integral.update(np.ones(7),1.,np.zeros(7),**kwargs)
    integral.update(np.ones(7),1.02,np.zeros(7),**kwargs)
    np.testing.assert_allclose(integral.bias,BIAS_SLEW*.02)
    previous = integral.bias.copy()
    integral.update(np.ones(7),1.02,np.zeros(7),**kwargs)
    np.testing.assert_equal(integral.bias,previous)
    for i in range(1000):
        torque = integral.update(np.ones(7),1.04+.02*i,np.zeros(7),**kwargs)
    np.testing.assert_allclose(integral.bias,BIAS_LIMIT)
    assert np.all(np.abs(torque) <= TORQUE_LIMIT)


def test_torque_saturation_does_not_wind_up_the_integral():
    integral = TorqueIntegral()
    for i in range(50):
        integral.update(np.ones(7),i*.02,TORQUE_LIMIT,depth=-.1,stroke=.015)
    np.testing.assert_array_equal(integral.bias,np.zeros(7))
    integral.update(-np.ones(7),1.,TORQUE_LIMIT,depth=-.1,stroke=.015)
    assert np.all(integral.bias < 0)


def test_contact_integral_is_unwound_during_retreat():
    integral = TorqueIntegral()
    integral.bias[:] = .5
    integral.baseline = np.zeros(7)
    integral.stamp = 0.
    integral.update(np.zeros(7),.02,np.zeros(7),depth=.015,stroke=.015,retreat=True)
    for i in range(200):
        integral.update(np.zeros(7),.04+.02*i,np.zeros(7),depth=-.01,stroke=.015,retreat=True)
    np.testing.assert_allclose(integral.bias,np.zeros(7),atol=1e-10)


@pytest.fixture
def controller_fixture():
    model = ROOT.parents[1]/'_local/bottle_push/menagerie/unitree_g1/g1.xml'
    if not model.exists():
        pytest.skip('Menagerie model not installed')
    cal = dict(tip_offset_m=(.1,0,0),pelvis_height_m=.79)
    _,q = load_robot(model,(.1,0,0),.79)
    scene = SceneSnapshot('button',(.4,-.12,1.05),(1.,0.,0.),.035,tuple(q),(0.,)*3,0.)
    kin = build_scene_model(model,cal,scene)
    qs = np.tile(q[22:29],(10,1))
    qs[:,0] += np.arange(10)*.001
    tcp=[]
    for angles in qs:
        pose = kin.data.qpos.copy(); pose[kin.qadr]=angles; kin.update(pose)
        tcp.append(kin.tip_position().copy())
    update_robot(kin,q,scene.imu)
    settings=SearchSettings(button_feedback='joint')
    c=Candidate(0,qs,np.asarray(tcp),['approach']*10,4,5,9,.2,.1,.05,np.zeros((10,7)),
        dict(button_feedback=controller_metadata()))
    return kin,scene,c,settings,cal,model


def test_body_compensation_reaches_the_world_target(controller_fixture):
    kin,scene,c,settings,*_=controller_fixture
    controller=ButtonTrackingController(kin,scene,c,settings)
    q=np.asarray(scene.q).copy(); q[14]+=.025
    pose=update_robot(kin,q,(.01,0.,0.))
    before=np.linalg.norm(kin.tip_position()-c.tcp[0])
    assert before > .002
    corrected=controller._body_corrected_goal(pose,c.q[0],c.tcp[0])
    pose[kin.qadr]=corrected; kin.update(pose)
    assert np.linalg.norm(kin.tip_position()-c.tcp[0]) < .0005


def test_recovery_reverses_only_the_path_already_traversed(controller_fixture):
    kin,scene,c,settings,*_=controller_fixture
    controller=ButtonTrackingController(kin,scene,c,settings)
    controller.index=3
    controller.recover(10.,'tracking timeout')
    np.testing.assert_array_equal(controller.qs,c.q[:4][::-1])
    assert controller.retreat and controller.error=='tracking timeout'
    controller.recover(11.,'second timeout')
    assert controller.recovery_started==10.


def test_lamp_return_reverses_only_the_traversed_path_without_error(controller_fixture):
    kin,scene,c,settings,*_=controller_fixture
    controller=ButtonTrackingController(kin,scene,c,settings)
    controller.index=5
    assert controller.return_on_lamp(10.)
    np.testing.assert_array_equal(controller.qs,c.q[:6][::-1])
    assert controller.retreat and controller.error is None and controller.index == 0
    assert not controller.return_on_lamp(11.)


def test_lamp_return_advances_despite_tracking_error_with_joint_limits(controller_fixture):
    kin,scene,c,settings,*_=controller_fixture
    def make_controller():
        controller=ButtonTrackingController(kin,scene,c,settings)
        controller.index=5
        controller.start_from_command(c.q[5])
        assert controller.return_on_lamp(2.)
        return controller
    accelerated=make_controller()
    measured=np.asarray(scene.q).copy(); measured[22:29]=c.q[5]
    row=accelerated.step(measured,scene.imu,2.,2.)
    assert row['index']==1
    assert np.max(np.abs(row['velocity'])) <= settings.motion_limits.joint_speed_rad_s
    stalled=make_controller()
    displaced=measured.copy(); displaced[22]+=.08
    assert stalled.step(displaced,scene.imu,2.,2.)['index']==1


def test_lamp_return_uses_precomputed_shortcut_and_stays_continuous(controller_fixture):
    kin,scene,c,settings,*_=controller_fixture
    c.motion_metrics['lamp_return_plan'] = dict(version=1,nodes=[0,3],edges=[None,
        dict(to=0,kind='shortcut',q=[c.q[3].tolist(),c.q[1].tolist(),c.q[0].tolist()],
             tcp=[c.tcp[3].tolist(),c.tcp[1].tolist(),c.tcp[0].tolist()])])
    controller=ButtonTrackingController(kin,scene,c,settings)
    controller.index=5
    assert controller.return_on_lamp(2.)
    np.testing.assert_array_equal(controller.qs, c.q[[5,4,3,1,0]])
    np.testing.assert_array_equal(controller.tcps, c.tcp[[5,4,3,1,0]])
    assert controller.lamp_return_shortcut


def test_return_optimizer_keeps_reference_when_shortcuts_collide(controller_fixture,monkeypatch):
    from button_return import optimize_lamp_returns,build_lamp_return
    kin,scene,c,settings,*_=controller_fixture
    attempts=[]
    def reject(*args,**kwargs):
        attempts.append(1)
        raise ValueError('collision')
    monkeypatch.setattr(kin,'validate_path',reject)
    c.q=np.repeat(c.q,20,axis=0)
    c.tcp=np.repeat(c.tcp,20,axis=0)
    c.press_end=len(c.q)-1
    plan=optimize_lamp_returns(kin,scene,c.q,c.tcp,c.press_end,settings.motion_limits,time.perf_counter()+10)
    c.motion_metrics['lamp_return_plan']=plan
    q,tcp,used=build_lamp_return(c,c.press_end)
    np.testing.assert_array_equal(q,c.q[:c.press_end+1][::-1])
    np.testing.assert_array_equal(tcp,c.tcp[:c.press_end+1][::-1])
    assert attempts and not used


def test_lamp_monitor_accepts_one_new_bright_frame_and_rejects_stale():
    from button_feedback import PressLampMonitor
    monitor=PressLampMonitor(50.,30.,10.)
    def observation(frame,stamp,level):
        return SimpleNamespace(frame_id=frame,captured_at=stamp,lamp=level)
    assert not monitor.observe(observation(1,9.9,150.),10.)
    assert not monitor.observe(observation(2,10.1,75.),10.2)
    assert not monitor.observe(observation(3,10.2,150.),11.)
    assert not monitor.observe(observation(3,10.2,150.),11.)
    assert monitor.observe(observation(4,11.1,80.),11.2)
    assert [v['frame_id'] for v in monitor.frames] == [4]


def test_image_lamp_monitor_requires_matching_detection_but_not_depth():
    from button_feedback import ImageLampTracker, PressLampMonitor
    tracker=ImageLampTracker()
    tracker.seed((10,10,30,30))
    dark=np.full((40,40,3),40,dtype=np.uint8)
    detection=SimpleNamespace(bbox=(10,10,30,30))
    tracker.observe(dark,[detection],1.,1)
    assert tracker.latest().lamp == 40
    image=dark.copy()
    image[10:30,10:30]=[230,190,80]
    tracker.observe(image,[],2.,2)
    assert tracker.latest().frame_id == 1
    tracker.observe(image,[SimpleNamespace(bbox=(30,10,39,30))],2.,2)
    assert tracker.latest().frame_id == 1
    tracker.observe(image,[detection],2.,2)
    observed=tracker.latest()
    assert observed.frame_id == 2 and observed.lamp > 40
    # Colour alone is insufficient; compare to the initial brightness.
    monitor=PressLampMonitor(observed.lamp,30.,1.5)
    assert not monitor.observe(observed,2.1)
    assert PressLampMonitor(40,30.,1.5).observe(observed,2.1)


def test_tracking_error_holds_the_reference_and_resumes_after_correction(controller_fixture):
    kin,scene,c,settings,*_=controller_fixture
    controller=ButtonTrackingController(kin,scene,c,settings)
    measured=np.asarray(scene.q).copy(); measured[22]+=.08
    first=controller.step(measured,scene.imu,0.,2.)
    second=controller.step(measured,scene.imu,.02,2.02)
    assert first['reference_held'] and second['index']==0
    assert np.linalg.norm(second['torque_bias']) > 0.
    resumed=controller.step(scene.q,scene.imu,.04,2.04)
    assert resumed['index'] > 0


@pytest.mark.parametrize(('command_clearance', 'guard_holds', 'index'), [
    (.002, 1, 0),
    (.003, 0, 1),
])
def test_next_command_enforces_obstacle_clearance(
        controller_fixture, monkeypatch, command_clearance, guard_holds, index):
    kin, scene, candidate, settings, *_ = controller_fixture
    controller = ButtonTrackingController(kin, scene, candidate, settings)
    clearances = iter([.05, command_clearance] + [.003] * guard_holds)
    monkeypatch.setattr('button_tracking.contact_metrics', lambda *args, **kwargs: next(clearances))

    result = controller.step(scene.q, scene.imu, 0., 2.)

    assert result['guard_holds'] == guard_holds
    assert result['index'] == index
    assert next(clearances, None) is None


def test_initial_correction_preserves_the_last_published_command(controller_fixture):
    kin,scene,c,settings,*_=controller_fixture
    controller=ButtonTrackingController(kin,scene,c,settings)
    hold=c.q[0].copy(); hold[0]+=.05
    controller.start_from_command(hold)
    row=controller.step(scene.q,scene.imu,0.,0.)
    assert np.max(np.abs(row['q']-hold)) <= settings.joint_acceleration_rad_s2*.02**2+1e-12


def test_feedback_cannot_execute_a_plan_checked_with_another_controller(controller_fixture):
    kin,scene,c,settings,*_=controller_fixture
    c.motion_metrics['button_feedback']['bias_limit_nm'][0]=100.
    with pytest.raises(ValueError,match='verified plan'):
        ButtonTrackingController(kin,scene,c,settings)


def test_collection_keeps_calibration_uncertainty_and_allows_equal_radii():
    cal=dict(button_radius_m=.015,tip_radius_m=.015,position_uncertainty_m=.015)
    trial=dict(button_direction='down',button_policy='collection',button_feedback='joint')
    session.validate_button_trial(trial,cal)
    assert cal['position_uncertainty_m']==.015 and trial['planning_scenarios']==1
    with pytest.raises(ValueError):
        session.validate_button_trial(dict(trial,button_policy='standard'),cal)
    validate_button_scenario_settings(1,.003,'collection')
    with pytest.raises(ValueError):
        validate_button_scenario_settings(1,.003)


def test_collection_requires_distinct_images_even_when_age_is_relaxed():
    window=ObservationWindow(allow_stale=True)
    geometry=dict(face=(.4,-.12,1.),normal=(1.,0.,0.),radius=.015)
    for index in range(3):
        last=Observation(geometry,(0.,)*29,(0.,)*3,index*.2,index)
        result=window.consume(last,100.)
    assert result.frame_ids==(0,1,2)
    with pytest.raises(ValueError,match='duplicate'):
        window.consume(last,100.)


def test_collection_observer_delivers_new_stable_position_after_a_large_shift():
    window=ObservationWindow(allow_stale=True,allow_target_motion=True)
    geometry=dict(face=(.4,-.12,1.),normal=(1.,0.,0.),radius=.015)
    for index in range(3):
        window.consume(Observation(geometry,(0.,)*29,(0.,)*3,index*.2,index),100.)
    moved=dict(geometry,face=(.43,-.04,.98))
    for index in range(3,6):
        result=window.consume(Observation(moved,(0.,)*29,(0.,)*3,index*.2,index),100.)
    assert result is not None
    np.testing.assert_allclose(result.geometry['face'],moved['face'])
    assert result.frame_ids==(3,4,5)


@pytest.mark.parametrize('record_deviations',[False,True])
def test_collection_target_movement_never_aborts_planning(record_deviations):
    from robust_execution import validate_start
    scene=SceneSnapshot('button',(.4,-.12,1.),(1.,0.,0.),.015,(0.,)*29,(0.,)*3,0.,
        button_policy='collection',tip_radius=.015,uncertainty_m=.015)
    observed=Observation(dict(face=(.43,-.04,.98),normal=scene.normal),scene.q,scene.imu,0.,1)
    config=(scene.q,scene.imu,0.,0.,0.)
    warnings=validate_start(scene,config,observed,record_deviations=record_deviations)
    assert warnings and 'Target moved' in warnings[0]
    with pytest.raises(RuntimeError,match='Target moved'):
        validate_start(replace(scene,radius=.035,button_policy='standard'),config,observed,
            record_deviations=record_deviations)


def test_collection_plan_requires_matching_feedback_physics_and_policy():
    from test_robust_button_policy import valid_plan
    plan=valid_plan()
    settings=SearchSettings(button_feedback='joint',button_policy='collection',scenarios=1)
    plan['settings']=asdict(settings)
    plan['scene']['button_policy']='collection'
    plan['accepted_conditions']=1
    plan['conditions']=plan['conditions'][:1]
    selected=next(c for c in plan['candidates'] if c['index']==plan['trajectory']['candidate'])
    selected['trials']=selected['trials'][:1]
    selected['trials'][0]['button_feedback']='joint'
    plan['trajectory']['motion_metrics']=dict(button_feedback=controller_metadata(),button_tracking_settings=asdict(settings))
    validate_button_plan(plan)
    plan['trajectory'].update(press_start=10,press_end=20)
    plan['trajectory']['motion_metrics']['lamp_return_plan']={'version':1}
    with pytest.raises(ValueError,match='帰還経路の物理検証'):
        validate_button_plan(plan)
    selected['trials'][0]['lamp_return_trials']=[dict(completed=True,success=True,
        lamp_return_triggered=True,trigger_index=i) for i in (10,20)]
    validate_button_plan(plan)
    selected['trials'][0]['lamp_return_trials'][0]['trigger_index']=11
    with pytest.raises(ValueError,match='検証位置'):
        validate_button_plan(plan)
    selected['trials'][0]['lamp_return_trials'][0]['trigger_index']=10
    selected['trials'][0].pop('button_feedback')
    with pytest.raises(ValueError,match='rollout'):
        validate_button_plan(plan)


def test_child_timeout_includes_both_planning_and_feedback_budget():
    assert session.button_child_timeout(dict(button_feedback='joint',planning_budget_s=120,button_feedback_timeout_s=120))==330.


def test_launcher_propagates_feedback_and_keeps_original_calibration(launcher,monkeypatch):
    from test_robust_button_policy import valid_plan
    calibration=dict(button_radius_m=.015,tip_radius_m=.015,position_uncertainty_m=.015)
    launcher.calibration.write_text(json.dumps(calibration))
    seen=[]
    def execute(command,log,env,*,input_text=None,timeout=100):
        seen.append((command,timeout))
        if '--execute' in command:
            trial=json.loads(Path(command[command.index('--trial-config')+1]).read_text())
            assert trial['button_feedback']=='joint' and trial['button_policy']=='collection'
            assert trial['planning_scenarios']==1 and trial['planning_budget_s']==120
            assert json.loads(Path(trial['base_transform']).read_text())==calibration
            assert command[command.index('--planning-scenarios')+1]=='1'
            settings=asdict(SearchSettings(button_feedback='joint',button_policy='collection',scenarios=1,budget_s=120))
            plan=valid_plan(); plan['settings']=settings;plan['scene']['button_policy']='collection'
            plan['accepted_conditions']=1;plan['conditions']=plan['conditions'][:1]
            plan['candidates'][0]['trials']=plan['candidates'][0]['trials'][:1]
            plan['candidates'][0]['trials'][0]['button_feedback']='joint'
            plan['trajectory']['motion_metrics']=dict(button_feedback=controller_metadata(),button_tracking_settings=settings)
            result=dict(completed=True,visual_success=True,plan=plan,
                execution=dict(motion_completed=True,release_completed=True,home_completed=True))
            Path(command[command.index('--output')+1]).write_text(json.dumps(result))
        return 0
    monkeypatch.setattr(session,'execute_child',execute)
    assert session.main(launcher.args('push')+['--button-feedback','joint','--button-policy','collection',
        '--planning-budget-s','120'])==0
    assert next(timeout for command,timeout in seen if '--execute' in command)==330.
    assert json.loads(launcher.calibration.read_text())==calibration


def test_delayed_outcome_still_requires_three_distinct_post_press_images():
    from robust_push import collect_outcome_observations
    geometry=dict(face=(.4,-.1,1.),normal=(1.,0.,0.),radius=.015)
    def obs(index,stamp): return Observation(geometry,(0.,)*29,(0.,)*3,stamp,index,100.)
    values=iter([obs(1,1.),obs(2,1.2),obs(3,1.4),obs(4,1.6)])
    observer=SimpleNamespace(latest=lambda **kwargs:next(values))
    results=collect_outcome_observations(observer,allow_stale=True,after=1.,now=lambda:100.,sleep=lambda dt:None)
    assert [v.frame_id for v in results]==[2,3,4]


def test_g1_onnx_detector_warms_up_without_torch(tmp_path):
    from button_detector import make_button_detector
    weight=tmp_path/'button.onnx';weight.touch()
    calls=[]
    class Backend:
        names={0:'up',1:'down'}
        device='cpu'
        def __init__(self,**kwargs): calls.append(kwargs)
        def detect(self,image): return [SimpleNamespace(class_name='down')]
    runtime=SimpleNamespace(module=lambda _:SimpleNamespace(perception=lambda name:
        SimpleNamespace(OnnxYoloDetector=Backend) if name=='detector.onnx_yolo' else pytest.fail('Torch loaded')))
    detector=make_button_detector(weight,runtime=runtime,direction='down')
    assert detector.detect(None)[0].class_name=='down' and calls[0]['num_threads']==2


def test_deploy_only_transfers_code_without_starting_the_robot(monkeypatch):
    calls=[]
    def upload(command,**kwargs):
        calls.append(command)
        with tarfile.open(fileobj=io.BytesIO(kwargs['input']),mode='r:gz') as archive:
            names=archive.getnames()
        assert 'Manipulation/push_button/button_tracking.py' in names
        assert not any('_local' in n or 'calibration' in n or '/tests/' in n for n in names)
        return SimpleNamespace(returncode=0,stdout=b'{"release":"/g1/release","runtime":"/g1/runtime"}',stderr=b'')
    monkeypatch.setattr(button_on_g1.subprocess,'run',upload)
    monkeypatch.setattr(button_on_g1.subprocess,'call',lambda *a,**k:pytest.fail('Motion launched during deploy-only'))
    assert button_on_g1.run_on_g1(ROOT.parents[1],dict(ssh_host='g1',remote_runtime_root='/g1/runtime'),
        ['push'],deploy_only=True)==0
    assert len(calls)==1


def test_remote_launch_uses_new_code_and_remote_setup(monkeypatch):
    monkeypatch.setattr(button_on_g1.subprocess,'run',lambda *a,**k:SimpleNamespace(returncode=0,
        stdout=b'{"release":"/g1/new-code","runtime":"/g1/runtime"}',stderr=b''))
    calls=[]
    monkeypatch.setattr(button_on_g1.subprocess,'call',lambda args:calls.append(args) or 0)
    button_on_g1.run_on_g1(ROOT.parents[1],dict(ssh_host='g1',remote_runtime_root='/g1/runtime'),
        ['--button-feedback','joint','push','--setup','/local/setup.json','--button-direction','down'])
    command=calls[0][-1]
    assert '/g1/new-code/Manipulation/push_button/real/robust_session.py' in command
    assert '/g1/runtime/_local/robust_button/active_setup.json' in command
    assert '/local/setup.json' not in command and 'run_remote.sh' not in command


def test_code_installer_preserves_existing_calibration_and_uses_separate_release(tmp_path):
    runtime=tmp_path/'runtime'; base=runtime/'_local/robust_button'
    base.mkdir(parents=True)
    (base/'active_setup.json').write_text('{}')
    (base/'calibration.json').write_text('{"position_uncertainty_m":0.015}')
    before=(base/'calibration.json').read_bytes()
    result=subprocess.run([sys.executable,'-c',button_on_g1.INSTALL,str(runtime),'test-release'],
        input=button_on_g1.source_archive(ROOT.parents[1]),capture_output=True,check=True)
    release=Path(json.loads(result.stdout)['release'])
    assert (release/'Manipulation/push_button/button_tracking.py').is_file()
    assert (release/'_local').is_symlink() and (base/'calibration.json').read_bytes()==before


@pytest.mark.parametrize('failure',[KeyboardInterrupt(),RuntimeError('state lost')])
def test_feedback_errors_preserve_partial_report_and_release(monkeypatch,controller_fixture,failure):
    import button_tracking_runtime as runtime
    import robust_execution
    kin,scene,c,settings,cal,model=controller_fixture
    closed=[]
    class Controller:
        def __init__(self,*a,**k):
            self.done=False; self.tcps=c.tcp; self.index=0; self.normal=np.asarray(scene.normal)
            self.retreat=False; self.guard_holds=0; self.integral=TorqueIntegral()
        def step(self,*a): raise failure
        def start_from_command(self,*a): pass
    monkeypatch.setattr(runtime,'ButtonTrackingController',Controller)
    monkeypatch.setattr(robust_execution,'validate_start',lambda *a,**k:[])
    motion=SimpleNamespace(enable=lambda:None,start_approach=lambda:None,
        close=lambda:closed.append(True),release_error=None)
    q=list(scene.q)
    config=lambda:(q,scene.imu,10.,0.,0.)
    observe=lambda:Observation(dict(face=scene.face,normal=scene.normal),scene.q,scene.imu,10.,1)
    result=runtime.execute_feedback(SimpleNamespace(scene=scene,candidate=c),model,cal,motion,
        config,observe,settings,now=lambda:10.)
    assert result['release_completed'] and not result['completed'] and closed
    assert result.get('interrupted',False)==isinstance(failure,KeyboardInterrupt)


def test_feedback_runtime_records_large_target_changes_and_keeps_sending(monkeypatch,controller_fixture):
    import button_tracking_runtime as runtime
    kin,scene,c,settings,cal,model=controller_fixture
    scene=replace(scene,button_policy='collection')
    settings=replace(settings,button_policy='collection')
    clock=[0.];offsets=[];sent=[];closed=[];frames=[0]
    class Controller:
        def __init__(self,*a,**k):
            self.done=False;self.tcps=c.tcp;self.index=0;self.normal=np.asarray(scene.normal)
            self.retreat=False;self.guard_holds=0;self.integral=TorqueIntegral();self.error=None
        def start_from_command(self,*a): pass
        def step(self,q,imu,stamp,elapsed,offset):
            offsets.append(offset.copy());self.done=len(offsets)==3
            return dict(q=c.q[0],velocity=np.zeros(7),tau=np.zeros(7),phase='approach',index=0,
                reference_held=False,torque_bias=np.zeros(7),actual_depth_m=-.1,reference_depth_m=-.1,
                guard_holds=0,error_m=0.,measured_tip=c.tcp[0],target=c.tcp[0])
    monkeypatch.setattr(runtime,'ButtonTrackingController',Controller)
    motion=SimpleNamespace(enable=lambda:None,start_approach=lambda:None,
        current_target=lambda:c.q[0],stream=lambda *a,**k:sent.append(a[0]),
        wait=lambda duration:clock.__setitem__(0,clock[0]+duration),
        close=lambda:closed.append(True),release_error=None)
    config=lambda:(scene.q,scene.imu,clock[0],0.,0.)
    def observe():
        frames[0]+=1
        return Observation(dict(face=tuple(np.asarray(scene.face)+[.03,.07,0.]),normal=scene.normal),
            scene.q,scene.imu,clock[0],frames[0])
    result=runtime.execute_feedback(SimpleNamespace(scene=scene,candidate=c),model,cal,motion,
        config,observe,settings,now=lambda:clock[0],record_deviations=True)
    assert result['completed'] and result['release_completed'] and len(sent)==3 and closed
    assert result['target_position_stop_enabled'] is False
    assert len(result['target_position_changes'])==3 and all(v['correction_clamped'] for v in result['target_position_changes'])
    assert all(np.linalg.norm(value)<=.005 for value in offsets)


def test_feedback_runtime_returns_on_first_new_lit_frame(monkeypatch,controller_fixture):
    import button_tracking_runtime as runtime
    import robust_execution
    _,scene,c,settings,cal,model=controller_fixture
    clock=[0.]; frames=[0]; actions=[]
    class Controller:
        def __init__(self,*args,**kwargs):
            self.done=False; self.qs=c.q; self.tcps=c.tcp; self.index=0; self.normal=np.asarray(scene.normal)
            self.retreat=False; self.guard_holds=0; self.integral=TorqueIntegral(); self.error=None
            self.press_end=c.press_end
        def start_from_command(self,*args): pass
        def return_on_lamp(self,elapsed):
            actions.append('lamp_return'); self.retreat=True; return True
        def step(self,q,imu,stamp,elapsed,offset):
            actions.append('retreat' if self.retreat else 'forward')
            self.done=self.retreat
            return dict(q=c.q[0],velocity=np.zeros(7),tau=np.zeros(7),phase='test',index=self.index,
                reference_held=False,torque_bias=np.zeros(7),actual_depth_m=-.1,reference_depth_m=-.1,
                guard_holds=0,error_m=0.,measured_tip=c.tcp[0],target=c.tcp[0])
    monkeypatch.setattr(runtime,'ButtonTrackingController',Controller)
    monkeypatch.setattr(robust_execution,'validate_start',lambda *a,**k:[])
    motion=SimpleNamespace(enable=lambda:None,start_approach=lambda:None,
        current_target=lambda:c.q[0],stream=lambda *a,**k:None,
        wait=lambda duration:clock.__setitem__(0,clock[0]+duration),
        close=lambda:None,release_error=None)
    config=lambda:(scene.q,scene.imu,clock[0],0.,0.)
    def observe():
        # Simulate 3D position estimation having stopped updating.
        return Observation(dict(face=scene.face,normal=scene.normal),scene.q,scene.imu,0.,1,20.)
    def observe_lamp():
        frames[0]+=1
        return SimpleNamespace(captured_at=clock[0],frame_id=frames[0],lamp=100.,blue_lit=False)
    result=runtime.execute_feedback(SimpleNamespace(scene=scene,candidate=c),model,cal,motion,
        config,observe,settings,now=lambda:clock[0],lamp_baseline=50.,observe_lamp=observe_lamp)
    assert 'error' not in result, result.get('error')
    assert actions == ['forward','lamp_return','retreat']
    assert result['lamp_early_return']['frames'][0]['level'] == 100.
    assert result['completed'] and result['release_completed']
    json.dumps(result)


def test_feedback_rpc_is_read_only_and_keeps_legacy_state_tuple():
    from test_arm_bridge import connection
    proxy,arm,errors,thread=connection()
    arm.joint_feedback_state=lambda:dict(received_at_s=1.,q=[.1]*7,dq=[.2]*7,tau_est=[.3]*7)
    try:
        assert proxy.joint_feedback_state()['tau_est']==[.3]*7
        assert len(proxy.current_configuration())==5 and arm.writes==0
    finally:
        proxy.socket.close(); thread.join(timeout=1.)


def test_shared_controller_completes_a_physical_press_and_retreat(controller_fixture):
    from robust_planner import generate_candidate, physical_trial
    kin,scene,_,_,cal,model=controller_fixture
    recorded=json.loads((ROOT/'tests/data/button_tracking_nov03.json').read_text())
    scene=SceneSnapshot(**recorded['scene'])
    cal=recorded['calibration']
    kin=build_scene_model(model,cal,scene)
    settings=SearchSettings(button_feedback='joint',button_policy='collection',scenarios=1,budget_s=90)
    deadline=time.perf_counter()+90.
    candidate=generate_candidate(kin,scene,0,(0.,0.,0.),(0.,0.),settings,deadline)
    assert candidate.motion_metrics['button_press']['reference_depth_m']==.015
    condition=uncertainty_scenarios(settings,scene)[0]
    condition['torque_effectiveness']=.5
    result=physical_trial(kin,scene,candidate,condition,deadline)
    assert result['completed'] and result['success'],result
    assert .0135 <= result['peak_press_m'] <= .016
    assert result['peak_force_n'] <= 40. and result['home_error_rad'] < np.deg2rad(2.)
    assert result['button_feedback']=='joint' and result['guard_holds']==0


def test_early_lamp_return_completes_in_physics(controller_fixture, monkeypatch):
    from robust_planner import generate_candidate, physical_trial
    _,_,_,_,_,model=controller_fixture
    recorded=json.loads((ROOT/'tests/data/button_tracking_nov03.json').read_text())
    scene=SceneSnapshot(**recorded['scene'])
    kin=build_scene_model(model,recorded['calibration'],scene)
    settings=SearchSettings(button_feedback='joint',button_policy='collection',scenarios=1,budget_s=90)
    deadline=time.perf_counter()+90.
    candidate=generate_candidate(kin,scene,0,(0.,0.,0.),(0.,0.),settings,deadline)
    original=ButtonTrackingController.step
    triggered=[]
    def step(controller,q,imu,stamp,elapsed,offset=None):
        if not controller.retreat and controller.index >= candidate.press_start+10:
            triggered.append(controller.index)
            controller.return_on_lamp(elapsed)
        return original(controller,q,imu,stamp,elapsed,offset)
    monkeypatch.setattr(ButtonTrackingController,'step',step)
    condition=uncertainty_scenarios(settings,scene)[0]
    result=physical_trial(kin,scene,candidate,condition,deadline)
    assert triggered and result['completed'] and result['success'],result
    assert result['home_error_rad'] < np.deg2rad(2.)
