"""Elevator launcher, occluded contact and outcome contracts; no hardware IO."""
from contextlib import contextmanager
from dataclasses import replace
import fcntl
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('mujoco')
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'real'),str(ROOT)]
import robust_session as session
import robust_execution as execution
from robust_push import button_lamp_outcome
from robust_perception import Observation
from robust_scene import SceneSnapshot
from robust_planner import SearchSettings,uncertainty_scenarios


def verified_plan(stroke=.002,face=(.36,-.12,1.05),normal=(1.,0.,0.)):
    scene=SceneSnapshot('button',tuple(face),tuple(normal),.035,(0.,)*29,(0.,)*3,0.,stroke=stroke)
    conditions=uncertainty_scenarios(SearchSettings(scenarios=4),scene)
    trials=[dict(completed=True,success=True,condition_name=c['name'],condition_position_m=c['position'])
        for c in conditions]
    return dict(accepted=True,accepted_conditions=4,completed_rollouts=4,scene=scene.to_dict(),
        conditions=conditions,trajectory=dict(candidate=0,q=[[0.]*7]),
        candidates=[dict(index=0,accepted=True,trials=trials)])


@pytest.fixture
def launcher(tmp_path,monkeypatch):
    monkeypatch.setattr(session,'ROOT',tmp_path)
    real=tmp_path/'Manipulation/push_button/real'; real.mkdir(parents=True)
    monkeypatch.setattr(session,'REAL',real)
    model,weights=tmp_path/'model.xml',tmp_path/'weights.pt'
    model.touch(); weights.touch()
    calibration=tmp_path/'calibration.json'
    calibration.write_text(json.dumps(dict(button_radius_m=.035,tip_radius_m=.015,position_uncertainty_m=.002)))
    trial=tmp_path/'trial.json'
    values=dict(task='button',stroke=.0015,button_direction='up',model=str(model),weights=str(weights),
        base_transform=str(calibration),network_interface='lo',planning_scenarios=3)
    trial.write_text(json.dumps(values))
    setup=tmp_path/'setup.json'
    setup.write_text(json.dumps(dict(sdk_python=sys.executable,vision_python=sys.executable,
        trial_config=str(trial),ssh_host='unused',remote_camera_root='unused',remote_sdk_python='unused')))
    calls=[]
    monkeypatch.setattr(session,'ensure_camera',lambda *a:calls.append('camera') or {})
    @contextmanager
    def bridge(*a):
        calls.append('bridge')
        yield Path('/tmp/fake_button.sock')
    monkeypatch.setattr(session,'sdk_bridge',bridge)
    def fake_execute(command,log,env,*,input_text=None):
        calls.append(command)
        log.write_text('mock child')
        if '--task' in command:
            assert command[command.index('--task')+1]=='button'
            assert not {'--execute-after-plan','--static-target','--allow-stale-observation'} & set(command)
            copied=json.loads(Path(command[command.index('--trial-config')+1]).read_text())
            assert copied['stroke']==.002 and copied['button_direction']=='down'
            assert copied['planning_scenarios']==4
            assert command[command.index('--planning-scenarios')+1]=='4'
            if '--execute' in command:
                assert '--execute-verified-plan' in command and input_text is None
                result=dict(completed=True,visual_success=True,plan=verified_plan(),
                    lamp_check=dict(status='lit'),execution=dict(motion_completed=True,
                        diagnostic_completed='--stop-before-contact' in command,home_completed=True,release_completed=True))
            else:
                result=dict(planning_completed=True,plan=verified_plan())
            Path(command[command.index('--output')+1]).write_text(json.dumps(result))
        return 0
    monkeypatch.setattr(session,'execute_child',fake_execute)
    return SimpleNamespace(root=tmp_path,setup=setup,trial=trial,calibration=calibration,calls=calls,execute=fake_execute,
        args=lambda mode:['--task','button',mode,'--setup',str(setup),'--stroke-mm','2','--button-direction','down'])


@pytest.mark.parametrize('mode',['check','precontact','push'])
def test_button_launcher_uses_measured_overrides_without_touching_originals(launcher,mode):
    assert session.main(launcher.args(mode))==0
    commands=[v for v in launcher.calls if isinstance(v,list)]
    assert len([c for c in commands if '--execute' in c])==(0 if mode=='check' else 1)
    assert json.loads(launcher.trial.read_text())['stroke']==.0015
    assert json.loads(launcher.trial.read_text())['planning_scenarios']==3
    run=next((launcher.root/'_local/robust_button/runs').iterdir())
    assert json.loads((run/'session.json').read_text())['completed']


@pytest.mark.parametrize('bad', ['stroke','radius','target','bottle_mode'])
def test_bad_button_configuration_never_opens_camera_or_sdk(launcher,bad):
    trial=json.loads(launcher.trial.read_text())
    if bad=='stroke': trial['stroke']=None
    if bad=='target': trial['button_direction']=None
    if bad=='bottle_mode': trial['allow_stale_observation']=True
    if bad=='radius':
        calibration=json.loads(launcher.calibration.read_text()); calibration['button_radius_m']=.016
        launcher.calibration.write_text(json.dumps(calibration))
    launcher.trial.write_text(json.dumps(trial))
    assert session.main(['--task','button','push','--setup',str(launcher.setup)])==2
    assert launcher.calls==[]


@pytest.mark.parametrize('failed_phase',['state','plan'])
def test_preparation_failure_never_sends_motion(launcher,monkeypatch,failed_phase):
    def fail(command,*args,**kwargs):
        if (failed_phase=='state' and 'check_robust_readiness.py' in command[1]) or (
                failed_phase=='plan' and '--task' in command):
            launcher.calls.append(command)
            return 2
        return launcher.execute(command,*args,**kwargs)
    monkeypatch.setattr(session,'execute_child',fail)
    mode='push' if failed_phase=='state' else 'check'
    assert session.main(launcher.args(mode))==2
    assert not any(isinstance(c,list) and '--execute' in c for c in launcher.calls)


def test_bottle_and_button_share_robot_lock(launcher):
    lock=launcher.root/'_local/robust_push/runs/robot.lock'
    lock.parent.mkdir(parents=True)
    with lock.open('a') as held:
        fcntl.flock(held,fcntl.LOCK_EX|fcntl.LOCK_NB)
        assert session.main(launcher.args('push'))==2
    assert launcher.calls==[]


@pytest.mark.parametrize('visual,status',[(True,'lit'),(False,'no_rise'),(None,'unknown')])
def test_lamp_failure_keeps_motion_result_and_never_retries(launcher,monkeypatch,visual,status):
    def child(command,*args,**kwargs):
        code=launcher.execute(command,*args,**kwargs)
        if '--execute' in command:
            path=Path(command[command.index('--output')+1]); result=json.loads(path.read_text())
            result.update(visual_success=visual,lamp_check=dict(status=status))
            path.write_text(json.dumps(result))
            return 0 if visual is True else 1
        return code
    monkeypatch.setattr(session,'execute_child',child)
    assert session.main(launcher.args('push'))==(0 if visual is True else 1)
    commands=[c for c in launcher.calls if isinstance(c,list) and '--execute' in c]
    assert len(commands)==1
    run=next((launcher.root/'_local/robust_button/runs').iterdir())
    report=json.loads((run/'session.json').read_text())
    assert report['motion_completed'] and report['release_completed'] and report['home_completed']
    assert report['completed']==(visual is True) and report['visual_success'] is visual


@pytest.mark.parametrize('levels,expected', [([140,150,160],True),([120,125,129],False),
    ([140,120,140],None),([140,None,150],None),([140,150],None),([float('nan'),140,150],None)])
def test_lamp_requires_three_consistent_finite_measurements(levels,expected):
    assert button_lamp_outcome(100,[SimpleNamespace(lamp=v) for v in levels],30) is expected


def test_home_warning_can_complete_but_release_failure_cannot():
    actual=dict(completed=True,visual_success=True,execution=dict(motion_completed=True,
        home_completed=True,release_completed=True),plan=verified_plan())
    changed=dict(actual,execution=dict(actual['execution'],home_completed=False))
    summary=session.button_result_summary(changed,'push')
    assert summary['completed'] and summary['home_completed'] is False
    changed=dict(actual,execution=dict(actual['execution'],release_completed=False))
    assert not session.button_result_summary(changed,'push')['completed']


@pytest.fixture
def contact_run(monkeypatch):
    s=SceneSnapshot('button',(.36,-.12,1.05),(1.,0.,0.),.035,(0.,)*29,(0.,)*3,100.)
    depths=np.r_[[-.007,-.006],np.linspace(-.005,.0015,10),np.linspace(0.,-.05,12),[-.06,-.08]]
    c=SimpleNamespace(q=np.zeros((len(depths),7)),tcp=np.array(s.face)+np.c_[depths-s.tip_radius,
        np.zeros((len(depths),2))],press_start=2,press_end=11,retract_end=23,
        tau=np.zeros((len(depths),7)),waist_tau=np.zeros((len(depths),3)),
        phases=['approach']*2+['contact']*4+['press']*6+['retract']*12+['home']*2)
    state=SimpleNamespace(step=0,time=100.,fault=None,released=False,commands=[],strict=[],held=[])
    kin=SimpleNamespace(model=SimpleNamespace(nv=7,geom=lambda n:SimpleNamespace(id=0)),
        data=SimpleNamespace(qpos=np.zeros(29)),qadr=np.arange(7),dadr=np.arange(7),site_id=0,
        update=lambda p:None,tip_position=lambda:c.tcp[min(state.step,len(depths)-1)].copy()+
            ([.011,0,0] if state.fault=='fk' and state.step==10 else [0,0,0]))
    def path(*a,**k):
        if state.fault=='collision' and state.step==10: raise ValueError('Unexpected arm contact')
    kin.validate_path=path
    monkeypatch.setattr(execution,'build_scene_model',lambda *a:kin)
    monkeypatch.setattr(execution,'update_robot',lambda *a:np.zeros(29))
    monkeypatch.setattr(execution,'contact_metrics',lambda *a,**k:.05)
    monkeypatch.setattr(execution,'reference_torque',lambda *a:np.zeros(7))
    monkeypatch.setattr(execution.mujoco,'mj_jacSite',lambda *a:None)
    def config():
        q=np.array(s.q); age=leg=gyro=0.
        if state.step==10:
            age={'state':.2,'delayed_state':.399,'expired_state':.401}.get(state.fault,0.)
            leg=.501 if state.fault=='leg_speed' else 0.
            gyro=.301 if state.fault=='gyro_speed' else 0.
            if state.fault=='pose': q[0]=.03; q[12]=.06
        if state.fault=='start_pose': q[22]=.02
        if state.fault=='home' and state.step>=len(c.q): q[22]=.05
        return q,s.imu,state.time-age,leg,gyro
    def observation(stamp,frame):
        face=np.array(s.face)+([0,.02,0] if state.fault=='moved_target' and state.step==10 else [0,0,0])
        return Observation(dict(face=face,normal=s.normal,radius=s.radius),s.q,s.imu,stamp,frame)
    def fresh():
        state.strict.append(state.step)
        return observation(100. if state.fault=='after_retreat' and state.step==24 else state.time,state.step+1)
    def held():
        state.held.append(state.step)
        if state.fault=='camera' and state.step==10: raise RuntimeError('Perception stopped: camera failed')
        stamp=state.time if state.fault=='moved_target' else 100.14
        return observation(stamp,state.step+1 if state.fault=='moved_target' else 3)
    class Motion:
        release_error=None
        streaming=False
        def enable(self): pass
        def start_approach(self): pass
        def stream(self,*a,**k): self.streaming=True; state.commands.append(state.step)
        def wait(self,seconds):
            # Deliberately slower than the reference period, but below the
            # existing 100ms update guard. Contact has no elapsed-time expiry.
            delay=.399 if state.fault=='delayed_update' else .401 if state.fault=='expired_update' else .07
            state.time+=max(seconds,delay)
            if self.streaming: state.step+=1
        def finish_stream(self): self.streaming=False
        def close(self):
            state.released=True
            if state.fault=='release': self.release_error='release failed'
    def run(fault=None,*,record_deviations=False):
        state.fault=fault
        report=execution.execute(SimpleNamespace(scene=s,candidate=c),'unused',{},Motion(),config,fresh,
            SearchSettings(),now=lambda:state.time,observe_held=held,record_deviations=record_deviations)
        return report,state,c
    return run


def test_occluded_contact_finishes_even_when_reference_time_is_exceeded(contact_run):
    report,state,c=contact_run()
    assert report['completed'] and report['release_completed'] and state.released
    assert report['trajectory_s']>len(c.q)*execution.PERIOD*3
    assert report['button_fixed_target']['interval_timeout_s'] is None
    assert report['button_fixed_target']['stale_observation_commands']>0
    assert set(state.held)==set(range(3,24))
    assert 24 in state.strict and len(report['commands'])==len(c.q)


@pytest.mark.parametrize('fault,reason',[('state','Stale'),('camera','camera failed'),
    ('collision','contact'),('moved_target','envelope'),('after_retreat','fresh')])
def test_occlusion_never_hides_state_camera_collision_or_target_failure(contact_run,fault,reason):
    report,state,_=contact_run(fault)
    assert not report['completed'] and reason in report['error']
    assert report['release_completed'] and state.released


@pytest.mark.parametrize('fault',['delayed_state','delayed_update','start_pose','pose','fk','home'])
def test_button_collection_policy_records_deviations_and_keeps_occluded_contact(contact_run,fault):
    report,state,_=contact_run(fault,record_deviations=True)
    assert report['completed'] and report['release_completed'] and state.released
    assert report['execution_policy']['state_timeout_s']==.4
    assert report['button_fixed_target']['stale_observation_commands']>0
    if fault in ('start_pose','pose','fk','home'): assert report['warnings']
    if fault=='home':
        assert report['home_completed'] is False and report['home_max_error_rad']==pytest.approx(.05)


@pytest.mark.parametrize('fault',['expired_state','expired_update','leg_speed','gyro_speed',
    'camera','collision','moved_target','after_retreat','release'])
def test_button_collection_policy_still_stops_on_hard_failures(contact_run,fault):
    report,state,_=contact_run(fault,record_deviations=True)
    assert not report['completed'] and state.released
    assert report.get('error') or report.get('release_error')


def test_button_shell_entrypoint_cannot_switch_to_bottle(tmp_path):
    # Capture Python argv instead of importing or connecting to any robot code.
    python=tmp_path/'python3'
    python.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
    python.chmod(0o755)
    import os
    result=subprocess.run(['bash',str(ROOT/'real/run_robust_button_press.sh'),'--task','bottle',
        '--button-direction','down'],capture_output=True,text=True,
        env=dict(os.environ,PATH=str(tmp_path)+os.pathsep+os.environ['PATH']),check=True)
    args=result.stdout.splitlines()
    assert args[1]=='push' and args[-2:]==['--task','button']


@pytest.mark.parametrize('levels,diagnostic,expected', [([140,145,150],False,0),
    ([110,115,120],False,1),([140,110,145],False,1),([],False,1),([],True,0)])
@pytest.mark.parametrize('legacy_plan',[False,True])
def test_full_button_entrypoint_keeps_monitoring_and_records_unknown_outcomes(
        tmp_path,monkeypatch,levels,diagnostic,expected,legacy_plan):
    import time
    import arm_bridge
    import robust_push as rp
    calibration=dict(tip_offset_m=(.1,0,0),tip_radius_m=.015,pelvis_height_m=.79,
        position_uncertainty_m=.001,reference_waist_q_rad=[0]*3,reference_imu_rpy_rad=[0]*3)
    checks=[]
    def read_calibration(*a,**kwargs):
        checks.append(kwargs)
        return calibration,None
    monkeypatch.setattr(rp,'read_calibration',read_calibration)
    monkeypatch.setattr(rp,'J1CameraRuntime',lambda *a:object())
    monkeypatch.setattr(rp,'make_button_detector',lambda *a,**k:object())
    monkeypatch.setattr(rp,'load_robot',lambda *a:(object(),np.zeros(29)))
    monkeypatch.setattr(rp,'ArmKinematics',lambda *a:SimpleNamespace(update_fixed_base=lambda *a:None))
    monkeypatch.setattr(rp,'CalibratedHeadCamera',lambda *a:object())
    monkeypatch.setattr(rp,'J1RgbdCamera',lambda *a:SimpleNamespace(close=lambda:None))
    arm=SimpleNamespace(stop_walking=lambda:None,release=lambda:None,close=lambda:None,
        current_configuration=lambda:([0.]*29,[0.]*3,time.monotonic(),0.,0.))
    monkeypatch.setattr(arm_bridge,'ArmProxy',lambda *a:arm)
    initial=Observation(dict(face=(.36,-.12,1.05),normal=(1.,0.,0.),radius=.035),
        (0.,)*29,(0.,)*3,time.monotonic(),3,100.,(1,2,3))
    class Observer:
        frames=[]
        rejections={}
        def __init__(self,*a): pass
        def initial(self): return initial
        def latest(self,**kwargs): return initial
        def close(self): pass
    monkeypatch.setattr(rp,'ContinuousObserver',Observer)
    monkeypatch.setattr(rp.time,'sleep',lambda dt:None)
    def search(model,calibration,scene,settings,*,monitor):
        monitor()
        assert settings.scenarios==4
        plan=verified_plan(stroke=scene.stroke,face=scene.face,normal=scene.normal)
        if legacy_plan: plan['accepted_conditions']=3
        return SimpleNamespace(scene=scene,candidate=object(),
            to_dict=lambda:plan)
    monkeypatch.setattr(rp,'search',search)
    motion=SimpleNamespace(upper_body_hold={},command_publications=[],error=None,close=lambda:None)
    def arm_motion(*a,**kwargs):
        assert not legacy_plan, 'Legacy three-condition plan must not enable the arm'
        assert kwargs['allow_pose_drift'] is True and kwargs['synchronous_stream'] is True
        return motion
    monkeypatch.setattr(rp,'ArmMotion',arm_motion)
    calls=[]
    def execute(*a,**kwargs):
        calls.append(kwargs)
        assert kwargs['record_deviations'] is True and not kwargs['allow_stale_observation']
        assert (kwargs['observe_held'] is None)==diagnostic
        return dict(completed=True,motion_completed=not diagnostic,diagnostic_completed=diagnostic,
            home_completed=True,release_completed=True)
    monkeypatch.setattr(rp,'execute',execute)
    monkeypatch.setattr(rp,'collect_outcome_observations',lambda *a:[replace(initial,frame_id=i+4,
        lamp=level) for i,level in enumerate(levels)])
    output=tmp_path/'result.json'
    args=['--task','button','--model','unused','--base-transform','unused','--stroke','.0015',
        '--execute','--calibrated','--execute-verified-plan','--arm-socket','unused','--output',str(output)]
    if diagnostic: args.append('--stop-before-contact')
    assert rp.main(args)==(2 if legacy_plan else expected)
    result=json.loads(output.read_text())
    if legacy_plan:
        assert not calls and '4条件' in result['error']
        return
    assert len(calls)==1 and result['execution']['release_completed']
    assert checks[0]['require_target_margin']
    if not diagnostic:
        assert result['lamp_check']['status']==('lit' if expected==0 else 'unknown' if len(levels)!=3 or levels==[140,110,145] else 'no_rise')
        assert result['motion_completed'] and result['physical_press_success'] is None


@pytest.mark.parametrize('height,x,yaw',[(.98,.35,-3),(.98,.36,0),(.98,.37,3),
    (1.05,.35,-3),(1.05,.36,0),(1.05,.37,3)])
def test_button_four_physical_conditions_at_two_heights_and_shifted_panels(height,x,yaw):
    from kinematics import load_robot
    from robust_planner import search
    model=ROOT.parents[1]/'_local/bottle_push/menagerie/unitree_g1/g1.xml'
    if not model.is_file(): pytest.skip('Menagerie asset not installed')
    calibration=dict(tip_offset_m=(.1,0,0),pelvis_height_m=.79)
    _,q=load_robot(model,(.1,0,0),.79)
    normal=(float(np.cos(np.radians(yaw))),float(np.sin(np.radians(yaw))),0.)
    scene=SceneSnapshot('button',(x,-.12,height),normal,.035,tuple(q),(0.,)*3,0.,uncertainty_m=.001)
    settings=SearchSettings(scenarios=4,budget_s=30.)
    plan=search(model,calibration,scene,settings)
    assert plan.candidate is not None,plan.report
    candidate=plan.candidate
    start,end=execution.button_fixed_interval(scene,candidate)
    assert start<=candidate.press_start and end>=candidate.retract_end
    trials=plan.report['candidates'][-1]['trials']
    assert len(trials)==4
    for trial in trials:
        assert trial['completed'] and trial['success'],trial
        assert .9*scene.stroke<=trial['peak_press_m']<=.00165
        assert trial['peak_force_n']<=40 and trial['min_clearance_m']>=.003
