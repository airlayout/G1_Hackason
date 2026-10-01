import json
from types import SimpleNamespace as NS
import numpy as np
import pytest
from common import ROOT
from run_real_reaction import (make_hold_initialized_command, build_paths, evaluate_paths,
                               evaluate_hold_target, prerequisites, enforce_collision_assessment,
                               CollisionWorker, InitializedWriter, Runtime,
                               velocity_observations,wait_for_fresh_idle_ownership,
                               first_collision_occurrences)


def snapshot():
    return json.loads((ROOT/'tests/fixtures/blocked_pose.json').read_text())


def factory():
    return NS(motor_cmd=[NS(q=0.,dq=0.,tau=0.,kp=0.,kd=0.,mode=0) for _ in range(35)],crc=0)


def test_all_upper_targets_initialized_before_weight_ramp():
    q=np.asarray(snapshot()['all_q'])
    for weight in (0.,.004,.5,1.):
        cmd=make_hold_initialized_command(factory,NS(Crc=lambda _:123),q,weight)
        assert cmd.mode_pr==0 and cmd.mode_machine==0
        assert all(m.mode==1 for m in cmd.motor_cmd)
        for i in range(29):
            m=cmd.motor_cmd[i]
            assert m.q==q[i] and m.dq==m.tau==0
            assert m.kp==(60 if i>=12 else 0)
        assert cmd.motor_cmd[29].q==weight and cmd.crc==123


def test_official_style_acquire_command_is_full_weight_fresh_q():
    q=np.asarray(snapshot()['all_q'])
    cmd=make_hold_initialized_command(factory,NS(Crc=lambda _:123),q,1.)
    assert cmd.motor_cmd[29].q==1.
    for i in range(29):assert cmd.motor_cmd[i].q==q[i]


def test_scaled_path_is_q0_relative_and_preserves_full_times():
    s=snapshot();paths,full,d=build_paths(s,.25);arm0=np.asarray(s['arm_q'])
    assert d['clip']==3. and d['entry']>=2 and d['return']>=2
    for phase in paths:
        np.testing.assert_allclose(paths[phase],arm0+.25*(full[phase]-arm0))
        np.testing.assert_allclose(paths[phase][:,:7],np.tile(arm0[:7],(len(paths[phase]),1)))
    np.testing.assert_allclose(paths['entry'][0],arm0)
    np.testing.assert_allclose(paths['return'][-1],arm0)


def test_half_path_is_q0_relative_and_preserves_full_times():
    s=snapshot();paths,full,d=build_paths(s,.5);arm0=np.asarray(s['arm_q'])
    assert d['clip']==3. and d['entry']>=2 and d['return']>=2
    for phase in paths:
        np.testing.assert_allclose(paths[phase],arm0+.5*(full[phase]-arm0))
        np.testing.assert_allclose(paths[phase][:,:7],np.tile(arm0[:7],(len(paths[phase]),1)))
    np.testing.assert_allclose(paths['entry'][0],arm0)
    np.testing.assert_allclose(paths['return'][-1],arm0)


def test_upper_body_path_uses_waist_and_both_arms_without_legs():
    s=snapshot();paths,full,d=build_paths(s,.25,'upper-body');q0=np.asarray(s['all_q'])[12:]
    assert all(path.shape[1]==17 for path in paths.values())
    np.testing.assert_allclose(paths['entry'][0],q0)
    np.testing.assert_allclose(paths['return'][-1],q0)
    assert np.max(abs(full['clip'][:,:3]-q0[:3]))>1e-3
    assert np.max(abs(full['clip'][:,3:10]-q0[3:10]))>1e-3
    assert np.max(abs(full['clip'][:,10:]-q0[10:]))>1e-3
    for phase in paths:
        np.testing.assert_allclose(paths[phase],q0+.25*(full[phase]-q0))


def test_practical_upper_profiles_scale_waist_separately():
    s=snapshot();q0=np.asarray(s['all_q'])[12:]
    arms_only,full,_=build_paths(s,.5,'upper-body',0.)
    mixed,_,_=build_paths(s,.5,'upper-body',.25)
    for phase in full:
        np.testing.assert_allclose(arms_only[phase][:,:3],
                                   np.tile(q0[:3],(len(arms_only[phase]),1)))
        np.testing.assert_allclose(arms_only[phase][:,3:],
                                   q0[3:]+.5*(full[phase][:,3:]-q0[3:]))
        np.testing.assert_allclose(mixed[phase][:,:3],
                                   q0[:3]+.25*(full[phase][:,:3]-q0[:3]))
        np.testing.assert_allclose(mixed[phase][:,3:],arms_only[phase][:,3:])


def test_hold_never_has_reaction_targets():
    s=snapshot();paths,_,d=build_paths(s,0.)
    for path in paths.values():np.testing.assert_array_equal(path,np.tile(s['arm_q'],(len(path),1)))
    assert evaluate_paths(paths,d,s,hold=True)['passed']


def test_fast_hold_recheck_does_not_load_reaction_trajectory():
    report=evaluate_hold_target(snapshot())
    assert report['passed'] and report['max_delta_rad']==0
    assert report['checks']['all_controlled_targets_equal_fresh_q0']
    assert not report['checks']['reaction_trajectory_loaded']


@pytest.mark.parametrize('stage',['scaled','half','full'])
def test_no_automatic_stage_advance(stage):
    args=NS(stage=stage,execute=True,confirm_site_ready=True,confirm_full_real_robot=False,
            previous_stage_result=None,confirm_previous_stage_observed=False,
            hold_baseline_worsening_log_only=False)
    with pytest.raises(ValueError):prerequisites(args)
    args.confirm_full_real_robot=True
    with pytest.raises(ValueError):prerequisites(args)


def test_failed_hold_blocks_scaled_even_with_flags(tmp_path):
    p=tmp_path/'stage_result.json';p.write_text(json.dumps({'stage':'hold','status':'FAIL','executed':True}))
    args=NS(stage='scaled',execute=True,confirm_site_ready=True,confirm_full_real_robot=False,
            previous_stage_result=p,confirm_previous_stage_observed=True,
            hold_baseline_worsening_log_only=False)
    with pytest.raises(ValueError,match='did not PASS'):prerequisites(args)


def test_upper_half_requires_upper_scaled_result(tmp_path):
    p=tmp_path/'stage_result.json'
    p.write_text(json.dumps({'stage':'scaled','status':'PASS','executed':True,
                             'ownership_released':True,'motion_layout':'right-arm'}))
    args=NS(stage='half',motion_layout='upper-body',execute=True,confirm_site_ready=True,
            confirm_full_real_robot=False,previous_stage_result=p,
            confirm_previous_stage_observed=True,hold_baseline_worsening_log_only=False)
    with pytest.raises(ValueError,match='layout does not match'):prerequisites(args)


def test_baseline_log_only_policy_is_hold_only():
    args=NS(stage='scaled',execute=False,confirm_site_ready=False,
            confirm_full_real_robot=False,previous_stage_result=None,
            confirm_previous_stage_observed=False,hold_baseline_worsening_log_only=True)
    with pytest.raises(ValueError,match='valid only for HOLD'):
        prerequisites(args)


def test_hold_baseline_log_only_suppresses_only_legacy_relative_stop():
    assessment={'hard_safety':{'passed':True,'reasons':[]},
                'baseline_relative':{'legacy_threshold_exceeded_pairs':['existing pair']}}
    enforce_collision_assessment(assessment,True)
    with pytest.raises(RuntimeError,match='baseline-relative'):
        enforce_collision_assessment(assessment,False)
    assessment['hard_safety']={'passed':False,'reasons':['new penetration']}
    with pytest.raises(RuntimeError,match='new penetration'):
        enforce_collision_assessment(assessment,True)


def test_collision_worker_fault_latches_across_later_safe_sample(monkeypatch):
    import run_real_reaction as real
    q=np.asarray(snapshot()['all_q'])
    state=NS(tick=1,motor_state=[NS(q=value) for value in q])
    reader=NS(get=lambda:state)
    values=iter([{'pair':-.001},{'pair':.02}])
    monkeypatch.setattr(real,'measured_pose_clearances',
                        lambda *args:(next(values),{'fk_s':.001,'collision_s':.01}))
    worker=CollisionWorker(reader,None,None,None,None,{'pair':.01})
    first=worker.evaluate_once()
    assert first['new_penetration']==['pair']
    assert first['new_pair_distances_m']=={'pair':-.001}
    assert first_collision_occurrences([first],'new_penetration')[0]['phase']=='PREFLIGHT'
    assert worker.snapshot(1.)['latched_collision_fault']
    worker.evaluate_once()
    cached=worker.snapshot(1.)
    assert cached['latched_collision_fault'] and not cached['collision_safe']
    assert cached['latest']['new_penetration']==[]


def test_lowstate_velocity_windows_are_separate_observations():
    history=[]
    for k in range(101):
        q=np.zeros(29);q[12]=2.*k/1000
        dq=np.zeros(29);dq[12]=2.1
        history.append({'sequence':k+1,'monotonic_s':k/1000,'tick':k,
                        'q':q.tolist(),'dq':dq.tolist()})
    rows,last=velocity_observations(history,1)
    assert last==101
    assert rows[-1]['raw_velocity_max_rad_s']==pytest.approx(2.)
    assert rows[-1]['lowstate_dq_max_rad_s']==pytest.approx(2.1)
    for window in ('20ms','50ms','100ms'):
        assert rows[-1][window+'_velocity_max_rad_s']==pytest.approx(2.)
    assert rows[-1]['100ms_interval_s']==pytest.approx(.1)


def test_post_benchmark_wait_requires_fresh_idle_and_never_waits_on_conflict(monkeypatch):
    unknown={'passed':False,'reasons':['Arm Action is not confirmed idle'],
             'arm_action':{'status':'UNKNOWN'}}
    idle={'passed':True,'reasons':[],'arm_action':{'status':'IDLE'}}
    states=[unknown,idle]
    monkeypatch.setattr('run_real_reaction.time.sleep',lambda _:None)
    assert wait_for_fresh_idle_ownership(NS(ownership_snapshot=lambda:states.pop(0))) is idle
    conflict={'passed':False,'reasons':['unknown external rt/arm_sdk writer exists'],
              'arm_action':{'status':'IDLE'}}
    with pytest.raises(RuntimeError,match='external'):
        wait_for_fresh_idle_ownership(NS(ownership_snapshot=lambda:conflict))


def test_runtime_sample_uses_collision_cache_without_synchronous_fk(monkeypatch):
    import threading
    import time
    import run_real_reaction as real
    q=np.asarray(snapshot()['all_q'])
    state=NS(tick=1,motor_state=[NS(q=value,dq=.30 if i==12 else 0.)
                                 for i,value in enumerate(q)],
             imu_state=NS(rpy=[.01,.02,.03]))
    monkeypatch.setattr(real,'validate_state',lambda *args,**kwargs:None)
    monkeypatch.setattr(real,'validate_upper',lambda *args,**kwargs:None)
    monkeypatch.setattr(real,'measured_pose_clearances',
                        lambda *args:(_ for _ in ()).throw(AssertionError('synchronous FK forbidden')))
    relative={'existing_pairs':{},'legacy_threshold_m':1e-6,
              'legacy_threshold_exceeded_pairs':[],
              'calibrated_threshold_applied':False,'stop_policy_changed':False}
    cached={'collision_safe':True,'last_collision_check_time':time.monotonic(),
            'last_state_tick':1,'result_age_s':.01,'stale':False,'stale_limit_s':.3,
            'latched_collision_fault':False,'latched_reasons':[],
            'latest':{'new_penetration':[],'new_dangerous_pair':[],
                      'baseline_relative_values':relative,'minimum_clearance_m':.02}}
    runtime=object.__new__(Runtime)
    sample_time=time.monotonic()
    history=[{'sequence':1,'monotonic_s':sample_time-.001,'tick':0,
              'q':q.tolist(),'dq':np.zeros(29).tolist()},
             {'sequence':2,'monotonic_s':sample_time,'tick':1,
              'q':q.tolist(),'dq':np.zeros(29).tolist()}]
    runtime.reader=NS(get=lambda:state,lock=threading.Lock(),latest=(sample_time,state),
                      motion_history_snapshot=lambda *_args:history)
    runtime.writer=NS(weight=0.);runtime.q0=q;runtime.previous_target=q.copy()
    runtime.ownership_watchdog=NS(snapshot=lambda:{'ownership_safe':True,'reasons':[],
                                                   'last_successful_check_age_s':.01,
                                                   'latched_fault':False})
    runtime.collision_worker=NS(snapshot=lambda stale:cached)
    runtime.collision_stale_s=.3;runtime.baseline={'pair':.02}
    runtime.rows=[];runtime.ownership_rows=[];runtime.stage='hold'
    runtime.baseline_worsening_log_only=True
    runtime.last_velocity_sequence=1;runtime.velocity_records=[]
    runtime.vw=NS(writerow=lambda row:None)
    runtime.mw=NS(writerow=lambda row:None)
    runtime.sw=NS(writerow=lambda row:None)
    runtime.lw=NS(writerow=lambda row:None)
    measured=runtime.sample('hold',q)
    np.testing.assert_array_equal(measured,q)
    assert runtime.rows[-1]['hard_safety']['passed']
    assert runtime.rows[-1]['hard_safety']['lowstate_dq_max_rad_s']==pytest.approx(.30)
    assert not runtime.rows[-1]['hard_safety']['velocity_hard_stop_applied']
    state.motor_state[4].q+=.031
    with pytest.raises(RuntimeError,match='unexpected leg movement'):
        runtime.sample('clip',q)
    assert runtime.rows[-1]['leg_q0_deviation_rad']==pytest.approx(.031)
    runtime.hackathon_suspended_mode=True
    runtime.previous_target=q.copy();runtime.previous_target[12]+=.11
    runtime.sample('clip',q)
    assert runtime.rows[-1]['hard_safety']['passed']
    assert 'tracking error >0.1 rad' in runtime.rows[-1]['soft_warnings']
    assert any('leg q0 deviation' in x for x in runtime.rows[-1]['soft_warnings'])


def test_invalid_pose_never_creates_publisher():
    calls=[]
    def publisher(*args):calls.append('publisher');raise AssertionError('must not be called')
    reader=NS(sdk=(None,None,publisher,None,None,factory,lambda:NS(Crc=lambda _:0)))
    with pytest.raises(ValueError):InitializedWriter(reader,np.zeros(3))
    assert calls==[]


def test_abort_holds_latest_measured_not_starting_pose(monkeypatch):
    import run_real_reaction as real
    monkeypatch.setattr(real.time,'sleep',lambda _:None)
    q=np.asarray(snapshot()['all_q']);q[22]+=.01
    r=object.__new__(Runtime)
    r.reader=NS(get=lambda:NS(motor_state=[NS(q=x) for x in q]))
    r.writer=NS(weight=.008)
    sent=[];r.send=lambda phase,target,weight:sent.append((target.copy(),weight))
    r.sample=lambda *a,**k:q
    r.abort_release()
    assert sent[-1][1]==0.
    for target,_ in sent:np.testing.assert_array_equal(target,q)
    assert all(a[1]>=b[1] for a,b in zip(sent,sent[1:]))


def test_stale_abort_releases_zero_gains_with_initialized_angles(monkeypatch):
    import run_real_reaction as real
    monkeypatch.setattr(real.time,'sleep',lambda _:None)
    def stale():raise RuntimeError('stale')
    messages=[];q=np.asarray(snapshot()['all_q'])
    r=object.__new__(Runtime);r.reader=NS(get=stale)
    r.writer=NS(weight=.2,factory=factory,crc=NS(Crc=lambda _:0),last_q=q,
                publisher=NS(Write=lambda cmd:messages.append(cmd)),zero_sent=False)
    r.cw=NS(writerow=lambda row:None)
    r.abort_release()
    assert len(messages)==3 and r.writer.zero_sent
    for cmd in messages:
        assert cmd.motor_cmd[29].q==0
        for i,m in enumerate(cmd.motor_cmd[:29]):
            assert m.kp==m.kd==0 and m.q==q[i]


def test_prevalidated_envelope_abort_reaches_weight_zero(monkeypatch):
    import run_real_reaction as real
    from robot_transport import RecoverableMotionEnvelopeExceeded
    monkeypatch.setattr(real.time, 'sleep', lambda _: None)
    q=np.asarray(snapshot()['all_q'])
    r=object.__new__(Runtime)
    r.prevalidated_runtime=True;r.release_duration_s=.10;r.q0=q.copy()
    r.reader=NS(get=lambda:NS(motor_state=[NS(q=x) for x in q]))
    r.writer=NS(weight=.4)
    sent=[]
    def send(phase,target,weight):
        sent.append((phase,weight));r.writer.weight=weight
    r.send=send
    r.sample=lambda *a,**k: (_ for _ in ()).throw(
        RecoverableMotionEnvelopeExceeded(
            tilt_peak=.05,tilt_limit=.30,gyro_peak=1.48,gyro_limit=.80))
    r.abort_release()
    assert r.writer.weight == 0.
    assert sent[-1] == ('abort_measured_hold_release', 0.)
