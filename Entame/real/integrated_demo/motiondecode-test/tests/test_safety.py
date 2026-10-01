import json
import inspect
import subprocess
import sys
import threading
import time
from collections import deque
from types import SimpleNamespace as NS
import numpy as np
import pytest
from common import (ROOT, NAMES, ARMS, ARM_NAMES, ARM_INDICES, read_motion,
                    read_arms, read_table, validate_arm_pose, digest, smoothstep,
                    retime_arm_window)
from play_g1_arms import validate_clip, prepare, transition, transition_duration, run_phase
from robot_transport import (arm_action_ownership_reasons, decode_arm_action_state,
                             make_command, ReadOnlyState, LOWSTATE_FRESHNESS_S,
                             RecoverableMotionEnvelopeExceeded, validate_state)

def artifact():return read_arms(ROOT/'output/selected_arms.csv')

def test_official_mapping_and_reordered_csv(tmp_path):
    assert ARM_INDICES==list(range(15,29))
    assert ARMS[0]['sdk_name']=='LeftShoulderPitch'
    assert ARMS[7]['sdk_name']=='RightShoulderPitch'
    source=ROOT/'data/motions/SII_Direction_Pointing_00001.csv'
    canonical,header=read_motion(source)
    raw=np.loadtxt(source,delimiter=',',skiprows=1)
    permutation=np.random.default_rng(42).permutation(36)
    p=tmp_path/'reordered.csv'
    np.savetxt(p,raw[:4,permutation],delimiter=',',comments='',header=','.join(header[i] for i in permutation))
    decoded,_=read_motion(p)
    np.testing.assert_allclose(decoded,canonical[:4])

@pytest.mark.parametrize('value',[np.nan,np.inf,-np.inf])
def test_nonfinite_motion_rejected(tmp_path,value):
    a,h=read_motion(ROOT/'data/motions/SII_Direction_Pointing_00001.csv');a=a[:3].copy();a[1,25]=value
    p=tmp_path/'bad.csv';np.savetxt(p,a,delimiter=',',header=','.join(h),comments='')
    with pytest.raises(ValueError,match='NaN / Inf'):read_motion(p)

def test_bad_quaternion_and_duplicate_headers(tmp_path):
    a,h=read_motion(ROOT/'data/motions/SII_Direction_Pointing_00001.csv');a=a[:3].copy();a[0,3:7]=0
    p=tmp_path/'bad.csv';np.savetxt(p,a,delimiter=',',header=','.join(h),comments='')
    with pytest.raises(ValueError,match='quaternion'):read_motion(p)
    p.write_text('x,x\n0,0\n0,0\n')
    with pytest.raises(ValueError,match='Duplicate'):read_table(p)

def test_output_tampering_is_rejected(tmp_path):
    source=ROOT/'output/selected_arms.csv';p=tmp_path/'motion.csv'
    p.write_bytes(source.read_bytes()+b'\n')
    p.with_suffix('.json').write_text(source.with_suffix('.json').read_text())
    with pytest.raises(ValueError):read_arms(p)

@pytest.mark.parametrize('speed',[0,-1,1,np.nan,np.inf])
def test_invalid_speed_never_reaches_transport(speed):
    t,q,m=artifact()
    with pytest.raises(ValueError):validate_clip(t,q,m,speed,2)

def test_hard_limits_and_excess_motion_rejected():
    t,q,m=artifact();q=q.copy();q[1,7]=20
    with pytest.raises(ValueError):validate_arm_pose(q)
    q=artifact()[1].copy();q[1,7]+=.6
    with pytest.raises(ValueError):validate_clip(t,q,m,.4,2)

def test_actual_artifact_meets_speed_and_acceleration_limits():
    t,q,m=artifact();v,a=validate_clip(t,q,m,.4,2)
    assert abs(v).max()<.25 and abs(a).max()<1
    assert m['source_fps'] is None and m['timing_mode']=='explicit_retiming'

def test_shared_retime_is_explicit_and_endpoint_safe():
    source,_=read_motion(ROOT/'data/motions/SII_Direction_Pointing_00001.csv')
    raw=source[180:271,7:][:,ARM_INDICES]
    t,q=retime_arm_window(raw,2.0,11,50)
    assert len(t)==len(q)==101 and t[0]==0 and t[-1]==2
    assert np.max(np.abs(np.diff(q,axis=0)))<.15
    # Quintic phase makes the sampled endpoint steps much smaller than an
    # ordinary linear phase and is important to the acceleration gate.
    assert np.max(np.abs(q[1]-q[0]))<np.max(np.abs(q[len(q)//2+1]-q[len(q)//2]))

def fake_factory():
    return NS(motor_cmd=[NS(q=0.,dq=0.,tau=0.,kp=0.,kd=0.,mode=0) for _ in range(35)],crc=0)

def test_no_leg_waist_or_left_arm_active_commands():
    q=artifact()[1][0]
    cmd=make_command(fake_factory,NS(Crc=lambda _:123),q,.7)
    for i,motor in enumerate(cmd.motor_cmd):
        if 22<=i<=28:
            assert motor.q==q[i-15] and motor.kp==60 and motor.kd==1.5
        elif i==29:assert motor.q==.7 and motor.kp==0
        else:assert all(x==0 for x in vars(motor).values())
    assert cmd.crc==123

@pytest.mark.parametrize('flags',[[],['--dry-run']])
def test_dry_run_without_socket_or_sdk(flags):
    code='''import socket,runpy,sys
def forbidden(*a,**k):raise AssertionError('network forbidden in dry-run')
socket.socket=forbidden
sys.path.insert(0,'scripts')
sys.argv=['scripts/play_g1_arms.py','output/selected_arms.csv']+FLAGS
try:runpy.run_path('scripts/play_g1_arms.py',run_name='__main__')
except SystemExit as e:assert e.code==0
assert not any(m.startswith(('unitree_sdk2py','cyclonedds')) for m in sys.modules)
'''
    code=code.replace('FLAGS',repr(flags))
    r=subprocess.run([sys.executable,'-c',code],cwd=ROOT,capture_output=True,text=True)
    assert r.returncode==0,r.stderr

def test_transitions_start_at_current_pose_and_return_with_limited_speed():
    q=artifact()[1][0];start=q.copy();start[7]+=.8
    duration=transition_duration(start,q,2)
    path=transition(start,q,duration)
    np.testing.assert_allclose(path[0],start);np.testing.assert_allclose(path[-1],q)
    assert abs(np.diff(path,axis=0)/(duration/(len(path)-1))).max()<=.25
    assert duration>=2

def test_saved_current_pose_is_blocked():
    t,q,_=artifact();snapshot=json.loads((ROOT/'tests/fixtures/blocked_pose.json').read_text())
    _,_,r=prepare(t,q,snapshot,2,.4)
    assert not r['clearance']['passed']

@pytest.mark.parametrize(('raw','status'),[
    ('{"holding": false, "id": 0, "name": ""}','IDLE'),
    ('{"holding": true, "id": 23, "name": "right hand up"}','ACTIVE'),
    ('{"id": 0}','UNKNOWN'),
    ('not json','UNKNOWN'),
])
def test_arm_action_state_decodes_only_observed_schema(raw,status):
    assert decode_arm_action_state(raw)['status']==status


def test_runtime_allows_unknown_arm_action_but_never_fresh_active():
    assert arm_action_ownership_reasons('UNKNOWN',allow_unknown=True)==[]
    assert arm_action_ownership_reasons('UNKNOWN',allow_unknown=False)
    assert arm_action_ownership_reasons('ACTIVE',allow_unknown=True)==[
        'fresh Arm Action is explicitly non-IDLE']

def test_stale_state_rejected():
    r=object.__new__(ReadOnlyState);r.lock=threading.Lock();r.latest=(time.monotonic()-1,object())
    r.lowstate_reader_error=None
    with pytest.raises(RuntimeError,match='stale'):r.get()


def safety_state(*, roll=0., pitch=0., gyro=(0., 0., 0.)):
    return NS(
        mode_machine=2, mode_pr=0, tick=1,
        imu_state=NS(rpy=(roll, pitch, 0.), gyroscope=gyro),
        motor_state=[NS(q=0., dq=0., motorstate=0, temperature=(25,))
                     for _ in range(29)],
    )


def test_first_test_tilt_and_gyro_limits_remain_default():
    validate_state(safety_state(roll=.20, gyro=(.50, 0., 0.)))
    with pytest.raises(RuntimeError, match=r"limit=0.20rad.*limit=0.50rad/s"):
        validate_state(safety_state(roll=.201))
    with pytest.raises(RuntimeError, match=r"limit=0.20rad.*limit=0.50rad/s"):
        validate_state(safety_state(gyro=(.501, 0., 0.)))


def test_hackathon_tilt_and_gyro_limits_are_bounded_opt_in():
    validate_state(
        safety_state(roll=.30, gyro=(.80, 0., 0.)),
        hackathon_suspended_mode=True,
    )
    with pytest.raises(RecoverableMotionEnvelopeExceeded,
                       match=r"limit=0.30rad.*limit=0.80rad/s"):
        validate_state(safety_state(pitch=.301), hackathon_suspended_mode=True)
    with pytest.raises(RuntimeError, match=r"limit=0.30rad.*limit=0.80rad/s"):
        validate_state(
            safety_state(gyro=(0., 0., .801)), hackathon_suspended_mode=True
        )


def bare_lowstate_reader():
    reader=object.__new__(ReadOnlyState)
    reader.lock=threading.Lock();reader.history=deque(maxlen=1000);reader.latest=None
    reader.motion_history=deque(maxlen=5000);reader.motion_sequence=0
    reader.last_tick=None;reader.lowstate_reader_error=None;reader.lowstate_closing=False
    return reader


def lowstate(tick):
    motors=[NS(q=float(i),dq=float(-i)) for i in range(29)]
    return NS(tick=tick,motor_state=motors,sample_info=NS(valid_data=True))


def test_direct_lowstate_callback_updates_latest_sequence_and_ignores_frozen_tick():
    reader=bare_lowstate_reader()
    invalid=lowstate(9);invalid.sample_info.valid_data=False
    source=NS(take=lambda count:[invalid,lowstate(10),lowstate(10),lowstate(11)])
    reader._direct_lowstate_available(source)
    assert reader.latest[1].tick==11
    assert reader.last_tick==11
    assert reader.motion_sequence==2
    assert [row['sequence'] for row in reader.motion_history]==[1,2]


def test_direct_lowstate_callback_failure_is_fail_closed():
    reader=bare_lowstate_reader()
    def failed(_):raise RuntimeError('take failed')
    reader._direct_lowstate_available(NS(take=failed))
    with pytest.raises(RuntimeError,match='reader failed'):
        reader.get()


def test_direct_lowstate_none_callback_is_ignored_only_during_cleanup():
    reader=bare_lowstate_reader();reader.lowstate_closing=True
    reader._direct_lowstate_available(None)
    assert reader.lowstate_reader_error is None

    reader=bare_lowstate_reader()
    reader._direct_lowstate_available(None)
    assert 'outside cleanup' in reader.lowstate_reader_error
    with pytest.raises(RuntimeError,match='reader failed'):
        reader.get()


def test_lowstate_freshness_threshold_and_default_backend_are_unchanged():
    assert LOWSTATE_FRESHNESS_S==.15
    assert inspect.signature(ReadOnlyState.get).parameters['max_age'].default==.15
    assert inspect.signature(ReadOnlyState).parameters['lowstate_backend'].default=='unitree'


def test_unitree_lowstate_backend_still_initializes_channel_subscriber():
    reader=bare_lowstate_reader();reader.lowstate_backend='unitree'
    calls=[]
    class Subscriber:
        def Init(self,callback,queue_length):calls.append(('init',callback,queue_length))
    def subscriber(topic,state_type):
        calls.append(('subscriber',topic,state_type));return Subscriber()
    reader._start_lowstate_reader(
        subscriber,'LowState',lambda *args:pytest.fail('direct reader used'),
        None,None)
    assert calls[0]==('subscriber','rt/lowstate','LowState')
    assert calls[1][0]=='init' and calls[1][2]==10


def test_direct_lowstate_backend_constructs_only_topic_listener_and_datareader():
    reader=bare_lowstate_reader();reader.lowstate_backend='cyclonedds'
    reader.discovery_participant='participant';calls=[]
    def topic(*args):calls.append(('topic',args));return 'topic'
    def listener(**kwargs):calls.append(('listener',kwargs));return 'listener'
    def data_reader(*args):calls.append(('data_reader',args));return 'reader'
    reader._start_lowstate_reader(
        lambda *args:pytest.fail('ChannelSubscriber used'),
        'LowState',data_reader,topic,listener)
    assert reader.reader=='reader'
    assert [name for name,_ in calls]==['topic','listener','data_reader']

@pytest.mark.parametrize('fault',[KeyboardInterrupt,RuntimeError])
def test_interrupt_cleanup_and_normal_release(monkeypatch,fault):
    import robot_transport as rt
    import play_g1_arms as player
    t,q,_=artifact();neutral=q[0]
    state=NS(motor_state=[NS(q=0) for _ in range(35)])
    summary={'arm_q':neutral.tolist(),'all_q':[0]*15+neutral.tolist()}
    calls=[]
    class Reader:
        arm_packets=0
        def __init__(self,*a):pass
        def wait(self,*a):return state
        def get(self):return state
        def require_stationary(self):pass
        def check_exclusive(self,**kwargs):pass
        def close(self):calls.append('reader_close')
    class Writer:
        def __init__(self,*a):calls.append('writer_created')
        def release_now(self):calls.append('release_zero')
        def close(self):calls.append('writer_close')
    monkeypatch.setattr(rt,'ReadOnlyState',Reader);monkeypatch.setattr(rt,'ArmWriter',Writer)
    monkeypatch.setattr(rt,'validate_state',lambda *a,**k:neutral)
    monkeypatch.setattr(rt,'state_summary',lambda *a:summary)
    monkeypatch.setattr(player,'check_feedback',lambda *a,**k:None)
    monkeypatch.setattr(player,'prepare',lambda *a:(q,neutral,{
        'passed':True,'failure_reasons':[],'clearance':{'passed':True},
        'entry_s':2,'playback_s':5,'return_s':2}))
    monkeypatch.setattr(sys,'stdin',NS(isatty=lambda:True))
    monkeypatch.setattr('builtins.input',lambda *a:'ARMS')
    monkeypatch.setattr(player,'write_json',lambda *a:None)
    monkeypatch.setattr(player.time,'sleep',lambda *a:None)
    def interrupted(*a,**k):raise fault()
    monkeypatch.setattr(player,'run_phase',interrupted)
    args=NS(network_interface='fake',blend=2,speed=.4)
    with pytest.raises(fault):player.execute(args,t,q)
    assert calls==['writer_created','release_zero','release_zero','release_zero','writer_close','reader_close']
    calls.clear();weights=[]
    def completed(*a,**k):weights.append((a[6](0),a[6](1)))
    monkeypatch.setattr(player,'run_phase',completed)
    player.execute(args,t,q)
    assert weights==[(0,1),(1,1),(1,1),(1,1),(1,0)]
    assert 'release_zero' not in calls and calls[-2:]==['writer_close','reader_close']
