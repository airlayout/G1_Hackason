#!/usr/bin/env python3
"""One independent attended G1 stage; defaults to receive-only preflight.

Reuses the existing DDS reader, quintic retiming and baseline-aware FK checker.
Never invokes RPC, navigation, lowcmd, or an automatic following stage.
"""
import argparse
import csv
import gc
import json
from pathlib import Path
import signal
import threading
import time
import traceback

import numpy as np
from common import (ROOT, JOINTS, NAMES, ARM_INDICES, read_motion, retime_arm_window, smoothstep,
                    digest, write_json, output_path, interpolate, validate_arm_pose,
                    acquire_file_lock)
from play_g1_arms import ACTIVE, DT, MAX_SPEED, MAX_ACCEL, transition, transition_duration
from scene import (make_scene, set_arms, clearance_pairs, pair_clearances,
                   check_baseline_contact_path, measured_pose_clearances)
from robot_transport import (ARM_ACTION_STALE_S, ReadOnlyState,
                             RecoverableMotionEnvelopeExceeded, state_summary,
                             validate_state)
from inspect_arm_sdk_publishers import local_process_scan, interface_ipv4
from safety_monitor import (OwnershipWatchdog, assess_collision_clearances,
                            joint_limit_violations)
from hold_core import command_fields, run_periodic_loop
from reaction_profiles import build_named_paths, load_trajectory_profile

SOURCE=ROOT/'data/motions/EESB_Frustration_00001.csv'
SOURCE_HASH='3c9c6397c7523fa80fc65995f2d03aa2fe9d50bbb7d91da546db896d8eb0f662'
CONTROLLED=list(range(12,29))  # official arm7: waist(3) + both arms(14)
RIGHT=list(range(22,29))
RAMP_S=5.0
RELEASE_S=2.0
HOLD_S=.75
FAST_ACQUIRE_S=.15
FAST_HOLD_S=.10
FAST_RELEASE_S=.30
FAST_POST_RELEASE_S=.10
TRACKING_LIMIT=.10
HOLD_DEVIATION_LIMIT=.03
FINAL_Q_TOLERANCE=.03
CLEARANCE=.015
BASELINE_TOLERANCE=1e-6  # unchanged existing numerical tolerance, NOT a noise allowance
VELOCITY_WINDOWS_S=(.02,.05,.10)


def _vector_peak(values,indices=CONTROLLED):
    values=np.asarray(values,dtype=float);selected=np.asarray(indices,dtype=int)
    local=int(np.argmax(np.abs(values[selected])));joint=int(selected[local])
    return float(abs(values[joint])),joint,NAMES[joint]


def velocity_observations(history,after_sequence,indices=CONTROLLED):
    """Characterize every new LowState sample without creating a stop limit.

    Raw velocity uses adjacent callback reception times. Window velocities use
    the newest earlier sample at or before each requested cutoff, so their
    reported interval is always at least 20/50/100 ms.
    """
    if len(history)<2:return [],after_sequence
    times=np.asarray([row['monotonic_s'] for row in history],dtype=float)
    observations=[];last=after_sequence
    for i in range(1,len(history)):
        current=history[i];sequence=int(current['sequence'])
        if sequence<=after_sequence:continue
        previous=history[i-1];dt=float(times[i]-times[i-1])
        if dt<=0:continue
        q=np.asarray(current['q']);raw=(q-np.asarray(previous['q']))/dt
        raw_peak,raw_joint,raw_name=_vector_peak(raw,indices)
        dq_peak,dq_joint,dq_name=_vector_peak(current['dq'],indices)
        record={'sequence':sequence,'monotonic_s':float(times[i]),'tick':int(current['tick']),
                'tick_delta':int(current['tick'])-int(previous['tick']),
                'raw_interval_s':dt,'raw_velocity_max_rad_s':raw_peak,
                'raw_velocity_joint_index':raw_joint,'raw_velocity_joint':raw_name,
                'lowstate_dq_max_rad_s':dq_peak,'lowstate_dq_joint_index':dq_joint,
                'lowstate_dq_joint':dq_name}
        for window in VELOCITY_WINDOWS_S:
            j=int(np.searchsorted(times[:i],times[i]-window,side='right')-1)
            prefix=f'{int(window*1000)}ms'
            if j<0:
                record[f'{prefix}_interval_s']=None
                record[f'{prefix}_velocity_max_rad_s']=None
                record[f'{prefix}_velocity_joint_index']=None
                record[f'{prefix}_velocity_joint']=None
                continue
            elapsed=float(times[i]-times[j])
            velocity=(q-np.asarray(history[j]['q']))/elapsed
            peak,joint,name=_vector_peak(velocity,indices)
            record[f'{prefix}_interval_s']=elapsed
            record[f'{prefix}_velocity_max_rad_s']=peak
            record[f'{prefix}_velocity_joint_index']=joint
            record[f'{prefix}_velocity_joint']=name
        observations.append(record);last=max(last,sequence)
    return observations,last


def enforce_collision_assessment(assessment, baseline_worsening_log_only=False):
    hard=assessment['hard_safety'];relative=assessment['baseline_relative']
    if not hard['passed']:
        raise RuntimeError('Runtime hard safety: '+'; '.join(hard['reasons']))
    if (relative['legacy_threshold_exceeded_pairs']
            and not baseline_worsening_log_only):
        raise RuntimeError('Runtime baseline-relative violation: '+
                           ', '.join(relative['legacy_threshold_exceeded_pairs']))


def validate_upper(q):
    q=np.asarray(q)
    if q.shape!=(29,) or not np.isfinite(q).all():
        raise ValueError('Need 29 finite measured/target joint angles')
    for i in CONTROLLED:
        if not JOINTS[i]['lower']+.03<=q[i]<=JOINTS[i]['upper']-.03:
            raise ValueError(f'Upper-body joint limit + margin: {NAMES[i]}')


def make_hold_initialized_command(factory,crc,q,weight,mode_machine=0):
    validate_upper(q)
    fields=command_fields(q,weight,0,CONTROLLED)
    cmd=factory()
    cmd.mode_pr=0;cmd.mode_machine=int(mode_machine)
    for motor in cmd.motor_cmd:motor.mode=1
    # Every physical joint target is initialized. Legs retain zero gains;
    # the 17 official arm_sdk joints hold q0 except the right reaction arm.
    for i in range(29):
        m=cmd.motor_cmd[i]
        m.q=fields['q'][i];m.dq=fields['dq'][i];m.tau=fields['tau'][i]
        m.kp=fields['kp'][i];m.kd=fields['kd'][i]
    cmd.motor_cmd[29].q=fields['weight']
    cmd.crc=crc.Crc(cmd)
    return cmd


class InitializedWriter:
    def __init__(self,reader,q0):
        _,_,publisher,_,cmd,factory,crc=reader.sdk
        self.factory=factory;self.crc=crc()
        validate_upper(np.asarray(q0))
        self.mode_machine=int(reader.get().mode_machine)
        # Validate/construct a fully initialized zero-weight message BEFORE
        # creating the publisher. No default-zero upper-body command is sent.
        self.initial=make_hold_initialized_command(
            factory,self.crc,q0,0.,self.mode_machine)
        self.publisher=publisher('rt/arm_sdk',cmd)
        self.publisher.Init()
        self.weight=0.;self.sent=0;self.zero_sent=False
        self.maximum_weight=0.;self.positive_weight_count=0
        self.first_positive_monotonic=None;self.first_full_weight_monotonic=None
        self.last_q=np.asarray(q0).copy()

    def write(self,q,weight):
        message=make_hold_initialized_command(
            self.factory,self.crc,q,weight,self.mode_machine)
        if self.publisher.Write(message) is False:
            raise RuntimeError('DDS write failed; receipt is unconfirmed')
        self.weight=float(weight);self.last_q=np.asarray(q).copy();self.sent+=1
        self.maximum_weight=max(self.maximum_weight,float(weight))
        self.positive_weight_count+=int(weight>0.)
        if weight>0. and self.first_positive_monotonic is None:
            self.first_positive_monotonic=time.monotonic()
        if weight>=1. and self.first_full_weight_monotonic is None:
            self.first_full_weight_monotonic=time.monotonic()
        self.zero_sent=bool(weight==0.)

    def close(self):
        self.publisher.Close()


def _retime_upper_window(q,duration=3.,smoothing_window=11,sample_hz=50):
    """Apply the same smoothing/quintic retiming to waist + both arms."""
    q=np.asarray(q,dtype=float)
    if q.ndim!=2 or q.shape[1]!=17 or len(q)<3 or not np.isfinite(q).all():
        raise ValueError('Expected at least three finite upper-body frames')
    padded=np.pad(q,((smoothing_window//2,smoothing_window//2),(0,0)),mode='edge')
    kernel=np.ones(smoothing_window)/smoothing_window
    smoothed=np.stack([np.convolve(padded[:,i],kernel,mode='valid')
                       for i in range(q.shape[1])],axis=1)
    output_t=np.linspace(0,duration,int(round(duration*sample_hz))+1)
    phase=smoothstep(output_t/duration)
    path=np.stack([np.interp(phase,np.linspace(0,1,len(q)),smoothed[:,i])
                   for i in range(q.shape[1])],axis=1)
    return output_t,path


def _transition_duration(a,b,minimum,motion_layout):
    if motion_layout=='right-arm':
        return transition_duration(a,b,minimum)
    delta=float(np.max(np.abs(np.asarray(b)-np.asarray(a))))
    return max(minimum,1.875*delta/MAX_SPEED,np.sqrt(5.774*delta/MAX_ACCEL))


def build_paths(snapshot,scale,motion_layout='right-arm',waist_scale=None):
    if scale not in (0.,.25,.5,1.):raise ValueError('Only hold, 25%, 50%, or full scale is supported')
    if digest(SOURCE)!=SOURCE_HASH:raise ValueError('Selected MotionDecode CSV hash changed')
    q0=np.asarray(snapshot['all_q']);validate_upper(q0)
    a,_=read_motion(SOURCE)
    if motion_layout=='right-arm':
        t,clip=retime_arm_window(a[5552:5613,7:][:,ARM_INDICES],3.,11,50)
        clip[:,:7]=q0[15:22]
        neutral=q0[15:]
    elif motion_layout=='upper-body':
        t,clip=_retime_upper_window(a[5552:5613,7+12:7+29],3.,11,50)
        neutral=q0[12:]
    else:
        raise ValueError('Unsupported motion layout')
    durations={'entry':_transition_duration(neutral,clip[0],2.,motion_layout),'clip':3.,
               'return':_transition_duration(clip[-1],neutral,2.,motion_layout)}
    full={'entry':transition(neutral,clip[0],durations['entry']), 'clip':clip,
          'return':transition(clip[-1],neutral,durations['return'])}
    paths={phase:neutral+scale*(path-neutral) for phase,path in full.items()}
    if waist_scale is not None:
        if motion_layout!='upper-body':
            raise ValueError('Waist scale is valid only for upper-body motion')
        if waist_scale not in (0.,.25,.5,1.):
            raise ValueError('Waist scale must be 0, 0.25, 0.5, or 1')
        for phase,path in paths.items():
            path[:,:3]=neutral[:3]+waist_scale*(full[phase][:,:3]-neutral[:3])
    return paths,full,durations


def evaluate_paths(paths,durations,snapshot,hold=False,motion_layout='right-arm'):
    q0=np.asarray(snapshot['all_q']);validate_upper(q0)
    model,data=make_scene();base=model.qpos0.copy()
    for i,name in enumerate(NAMES):base[model.joint(name).qposadr[0]]=q0[i]
    phases={};limits=True
    merged_t=[];merged_q=[];offset=0.
    for phase,path in paths.items():
        try:
            if motion_layout=='right-arm':validate_arm_pose(path)
            else:
                lower=np.array([j['lower'] for j in JOINTS[12:]])+.03
                upper=np.array([j['upper'] for j in JOINTS[12:]])-.03
                if not np.isfinite(path).all() or np.any(path<lower) or np.any(path>upper):
                    raise ValueError('Upper-body joint limit + margin')
        except ValueError:limits=False
        t=np.linspace(0,durations[phase],len(path));dt=t[1]-t[0]
        v=np.diff(path,axis=0)/dt;acc=np.diff(v,axis=0)/dt
        phases[phase]={'duration_s':float(durations[phase]),
                       'max_velocity_rad_s':float(np.max(abs(v))),
                       'max_acceleration_rad_s2':float(np.max(abs(acc)))}
        merged_t.extend((t+(offset))[int(bool(merged_t)):])
        merged_q.extend(path[int(bool(merged_q)):]);offset+=t[-1]
    mq=np.asarray(merged_q);mt=np.asarray(merged_t)
    vel=np.diff(mq,axis=0)/np.diff(mt)[:,None]
    acc=np.diff(vel,axis=0)/((np.diff(mt)[1:]+np.diff(mt)[:-1])/2)[:,None]
    if motion_layout=='right-arm':
        moving_indices=ACTIVE;path_names=NAMES[15:];mesh_prefixes=('right_',)
        neutral=q0[15:];delta_indices=slice(7,None)
    else:
        moving_indices=list(range(17));path_names=NAMES[12:]
        mesh_prefixes=('left_','right_');neutral=q0[12:];delta_indices=slice(None)
    collision=check_baseline_contact_path(
        paths['entry'],paths['clip'],paths['return'],base,moving_indices,
        path_joint_names=path_names,moving_mesh_prefixes=mesh_prefixes)
    if hold:
        # HOLD has no motion onset and never claims to clear the q0 contacts.
        # All samples are exactly q0; no existing pair may worsen and no new
        # penetration/deficit may occur. Motion stages use the UNCHANGED checker.
        exact_hold=all(np.array_equal(path,np.tile(neutral,(len(path),1))) for path in paths.values())
        collision_pass=(exact_hold and collision['baseline_rules']['no_baseline_pair_worsened']
                        and not collision['new_collision_pairs'] and not collision['new_clearance_deficit_pairs'])
    else:collision_pass=collision['passed']
    selected_delta=abs(mq[:,delta_indices]-neutral[delta_indices])
    delta_row,delta_col=np.unravel_index(np.argmax(selected_delta),selected_delta.shape)
    selected_names=np.asarray(path_names)[delta_indices]
    max_delta=float(selected_delta[delta_row,delta_col])
    vel_row,vel_col=np.unravel_index(np.argmax(abs(vel)),vel.shape)
    acc_row,acc_col=np.unravel_index(np.argmax(abs(acc)),acc.shape)
    vmax=float(abs(vel[vel_row,vel_col]));amax=float(abs(acc[acc_row,acc_col]))
    entry_start_jump=float(np.max(abs(paths['entry'][0]-neutral)))
    return_end_jump=float(np.max(abs(paths['return'][-1]-neutral)))
    checks={'joint_limits':limits,'max_delta_le_0_8':max_delta<=.8,
            'velocity':vmax<=MAX_SPEED,'acceleration':amax<=MAX_ACCEL,
            'clip_range':float(np.ptp(paths['clip'][:,delta_indices],axis=0).max())<=.5,
            'finite':bool(np.isfinite(mq).all()),
            'timestamp_continuity':bool(np.all(np.diff(mt)>0)),
            'first_frame_jump':entry_start_jump<=1e-9,
            'last_frame_jump':return_end_jump<=1e-9,
            'collision':bool(collision_pass)}
    report={'passed':bool(all(checks.values())),'checks':checks,'phases':phases,
            'max_delta_rad':max_delta,'max_velocity_rad_s':vmax,'max_acceleration_rad_s2':amax,
            'max_delta_joint':str(selected_names[delta_col]),'max_delta_frame':int(delta_row),
            'max_velocity_joint':path_names[vel_col],'max_velocity_frame':int(vel_row),
            'max_acceleration_joint':path_names[acc_col],'max_acceleration_frame':int(acc_row),
            'first_frame_jump_rad':entry_start_jump,'last_frame_jump_rad':return_end_jump,
            'joint_count':len(path_names),'joint_names':list(path_names),
            'collision':collision,'hold_only':hold}
    if motion_layout=='upper-body':
        waist=mq[:,:3];waist_v=vel[:,:3];waist_a=acc[:,:3]
        report['waist']={
            'max_delta_rad':float(np.max(abs(waist-q0[12:15]))),
            'max_velocity_rad_s':float(np.max(abs(waist_v))),
            'max_acceleration_rad_s2':float(np.max(abs(waist_a))),
        }
    return report


def evaluate_hold_target(snapshot):
    q0=np.asarray(snapshot['all_q']);validate_upper(q0)
    return {'passed':True,'hold_only':True,
            'checks':{'joint_limits':True,'all_controlled_targets_equal_fresh_q0':True,
                      'reaction_trajectory_loaded':False},
            'max_delta_rad':0.0,'max_velocity_rad_s':0.0,'max_acceleration_rad_s2':0.0}


def observe(reader,folder,interface,allow_robot_internal_idle_traffic=False):
    reader.wait(1.)
    started=time.monotonic();timeline=[]
    while time.monotonic()-started<15.:
        state=reader.get();validate_state(state)
        # Discovery/action readers may not have delivered their first sample at
        # the start of this 15 s observation.  Keep writer checks strict while
        # requiring exact fresh IDLE at the final gate below.
        ownership=reader.ownership_snapshot(
            allow_arm_action_unknown=True,
            allow_robot_internal_idle_traffic=allow_robot_internal_idle_traffic)
        if not ownership['passed']:
            raise RuntimeError('Arm control ownership blocked: '+'; '.join(ownership['reasons']))
        with reader.lock:age=time.monotonic()-reader.latest[0]
        timeline.append({'t_s':time.monotonic()-started,'tick':int(state.tick),'age_s':age,
                         'ownership_passed':ownership['passed']})
        time.sleep(.1)
    reader.require_stationary();state=reader.get();validate_state(state,initial=True)
    os_scan=local_process_scan(interface_ipv4(interface))
    if os_scan['candidate_processes_excluding_diagnostic']:
        raise RuntimeError('Local arm/DDS controller process candidate exists')
    ownership=wait_for_fresh_idle_ownership(
        reader,allow_robot_internal_idle_traffic=allow_robot_internal_idle_traffic)
    snapshot=state_summary(state)
    snapshot.update(timestamp=time.strftime('%Y-%m-%d %H:%M:%S %z'),receive_only=True,
                    captured_monotonic=time.monotonic(),command_publishers_created=0,
                    source_ownership_report=str(folder/'preflight.json'))
    report={'passed':True,'ownership':ownership,'stationary':True,'lowstate':'PASS',
            'duration_s':time.monotonic()-started,'timeline':timeline,'local_os_scan':os_scan}
    write_json(folder/'fresh_q0.json',snapshot);write_json(folder/'preflight.json',report)
    return snapshot


class CollisionWorker:
    """Evaluate the unchanged full FK/collision set outside the 20 ms loop."""
    def __init__(self,reader,model,data,base,pairs,baseline,period_s=0.,context_provider=None):
        self.reader=reader;self.model=model;self.data=data;self.base=base;self.pairs=pairs
        self.baseline=dict(baseline);self.period_s=float(period_s)
        self.context_provider=context_provider or (lambda:{'phase':'PREFLIGHT','weight':0.})
        self.lock=threading.Lock();self.stop_event=threading.Event();self.thread=None
        self.records=[];self.latest=None;self.latched_fault=False;self.latched_reasons=[]

    def evaluate_once(self):
        started=time.monotonic();record={'started_monotonic_s':started,
                                         **self.context_provider()}
        try:
            state=self.reader.get();tick=int(state.tick)
            q=np.array([m.q for m in state.motor_state[:29]])
            distances,timing=measured_pose_clearances(
                self.model,self.data,self.base,self.pairs,q)
            collision=assess_collision_clearances(
                distances,self.baseline,CLEARANCE,BASELINE_TOLERANCE)
            reasons=[]
            if collision['new_penetration_pairs']:reasons.append('new penetration')
            if collision['new_dangerous_pairs']:reasons.append('new dangerous clearance pair')
            completed=time.monotonic()
            record.update(completed_monotonic_s=completed,duration_s=completed-started,
                          last_state_tick=tick,error=None,fk_s=timing['fk_s'],
                          collision_s=timing['collision_s'],
                          new_penetration=collision['new_penetration_pairs'],
                          new_dangerous_pair=collision['new_dangerous_pairs'],
                          new_pair_distances_m={key:float(distances[key]) for key in
                              set(collision['new_penetration_pairs']+
                                  collision['new_dangerous_pairs'])},
                          baseline_relative_values=collision['baseline_relative'],
                          minimum_clearance_m=min(distances.values()))
        except BaseException as exc:
            completed=time.monotonic();reasons=['collision worker error: '+str(exc)]
            record.update(completed_monotonic_s=completed,duration_s=completed-started,
                          last_state_tick=None,error=str(exc),new_penetration=[],
                          new_dangerous_pair=[],new_pair_distances_m={},
                          baseline_relative_values=None,
                          minimum_clearance_m=None)
        with self.lock:
            previous=self.records[-1]['started_monotonic_s'] if self.records else None
            record['interval_s']=None if previous is None else started-previous
            if reasons and not self.latched_fault:
                self.latched_fault=True;self.latched_reasons=list(reasons)
            record['latched_collision_fault']=self.latched_fault
            record['latched_reasons']=list(self.latched_reasons)
            self.records.append(record);self.latest=dict(record)
        return record

    def seed_safe_result(self,sweep):
        """Seed the cache from the synchronous q0 sweep immediately pre-acquire."""
        if self.thread is not None:
            raise RuntimeError('Seed collision cache before starting worker')
        if sweep['new_penetration_pairs'] or sweep['new_dangerous_pairs']:
            raise RuntimeError('Cannot seed collision cache from unsafe sweep')
        record={'started_monotonic_s':sweep['started_monotonic_s'],
                'completed_monotonic_s':sweep['completed_monotonic_s'],
                'duration_s':sweep['duration_s'],'last_state_tick':sweep['state_tick'],
                'phase':'PREFLIGHT','weight':0.,
                'error':None,'fk_s':sweep['fk_s'],'collision_s':sweep['collision_s'],
                'new_penetration':[],'new_dangerous_pair':[],
                'new_pair_distances_m':{},'baseline_relative_values':{
                    'existing_pairs':{},'legacy_threshold_m':BASELINE_TOLERANCE,
                    'legacy_threshold_exceeded_pairs':[],
                    'calibrated_threshold_applied':False,'stop_policy_changed':False},
                'minimum_clearance_m':sweep['minimum_clearance_m'],'interval_s':None,
                'latched_collision_fault':False,'latched_reasons':[],
                'source':'synchronous pre-acquire full sweep'}
        with self.lock:self.records=[record];self.latest=dict(record)
        return record

    def _run(self):
        while not self.stop_event.is_set():
            started=time.monotonic();self.evaluate_once()
            self.stop_event.wait(max(0.,self.period_s-(time.monotonic()-started)))

    def start(self):
        self.thread=threading.Thread(target=self._run,name='collision-worker',daemon=True)
        self.thread.start();return self

    def wait_samples(self,count,timeout_s):
        deadline=time.monotonic()+timeout_s
        while time.monotonic()<deadline:
            with self.lock:n=len(self.records)
            if n>=count:return
            time.sleep(.01)
        raise RuntimeError('Collision worker benchmark did not produce enough samples')

    def snapshot(self,stale_s):
        now=time.monotonic()
        with self.lock:
            latest=None if self.latest is None else dict(self.latest)
            latched=self.latched_fault;reasons=list(self.latched_reasons)
        age=None if latest is None else now-latest['completed_monotonic_s']
        stale=age is None or age>stale_s
        safe=latest is not None and not stale and not latched and latest.get('error') is None
        return {'collision_safe':safe,'last_collision_check_time':None if latest is None else latest['completed_monotonic_s'],
                'last_state_tick':None if latest is None else latest['last_state_tick'],
                'result_age_s':age,'stale':stale,'stale_limit_s':stale_s,
                'latched_collision_fault':latched,'latched_reasons':reasons,'latest':latest}

    def timing_records(self):
        with self.lock:return [dict(x) for x in self.records]

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:self.thread.join(timeout=3.)
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError('Collision worker did not stop')


class PrevalidatedCollisionCache:
    """Expose the synchronous preflight result without running MuJoCo in-loop."""
    def __init__(self,sweep):
        self.completed=float(sweep['completed_monotonic_s'])
        self.tick=sweep.get('state_tick')
        self.minimum=sweep['minimum_clearance_m']

    def snapshot(self,_stale_s):
        age=time.monotonic()-self.completed
        latest={'new_penetration':[],'new_dangerous_pair':[],
                'minimum_clearance_m':self.minimum,
                'baseline_relative_values':None,'source':'prevalidated full sweep'}
        return {'collision_safe':True,'last_collision_check_time':self.completed,
                'last_state_tick':self.tick,'result_age_s':age,'stale':False,
                'stale_limit_s':None,'latched_collision_fault':False,
                'latched_reasons':[],'latest':latest}


class PrevalidatedOwnershipCache:
    """Latch the complete ownership preflight for one short attended stage."""
    def __init__(self,snapshot):
        self.cached=dict(snapshot)
        self.started=time.monotonic()

    def snapshot(self):
        result=dict(self.cached)
        result['last_successful_check_age_s']=time.monotonic()-self.started
        return result

    def stop(self): pass

    def timing_records(self):
        return [{'mode':'prevalidated ownership cache',
                 'started_monotonic_s':self.started,
                 'ownership_safe':self.cached.get('ownership_safe')}]


class Runtime:
    def __init__(self,reader,writer,snapshot,folder,stage,ownership_watchdog=None,
                 baseline_worsening_log_only=False,prevalidated_runtime=False,
                 release_settling_warning_only=False,hackathon_suspended_mode=False,
                 release_duration_s=RELEASE_S):
        self.reader=reader;self.writer=writer;self.stage=stage;self.q0=np.asarray(snapshot['all_q'])
        self.ownership_watchdog=ownership_watchdog
        self.baseline_worsening_log_only=bool(baseline_worsening_log_only)
        self.prevalidated_runtime=bool(prevalidated_runtime)
        self.release_settling_warning_only=bool(release_settling_warning_only)
        self.hackathon_suspended_mode=bool(hackathon_suspended_mode)
        self.release_duration_s=float(release_duration_s)
        self.model,self.data=make_scene();self.pairs=clearance_pairs(self.model)
        self.base=None;self.baseline=None;self.collision_worker=None;self.collision_stale_s=None
        self.reset_baseline(snapshot)
        self.rows=[];self.ownership_rows=[];self.previous_target=self.q0.copy()
        self.fast_loop_timings=[];self.command_rows=[];self.velocity_records=[]
        self.phase_terminations=[]
        self.last_velocity_sequence=0
        self.current_context={'phase':'PREFLIGHT','weight':0.}
        log_buffer=64*1024
        self.command_stream=(folder/'command_trajectory.csv').open('w',buffering=log_buffer)
        self.measured_stream=(folder/'measured_trajectory.csv').open('w',buffering=log_buffer)
        self.velocity_stream=(folder/'lowstate_velocity_samples.csv').open('w',buffering=log_buffer)
        self.shoulder_stream=(folder/'left_shoulder_control.csv').open('w',buffering=log_buffer)
        self.leg_stream=(folder/'leg_control.csv').open('w',buffering=log_buffer)
        self.cw=csv.writer(self.command_stream);self.mw=csv.writer(self.measured_stream)
        self.sw=csv.writer(self.shoulder_stream)
        self.lw=csv.writer(self.leg_stream)
        self.vw=csv.DictWriter(self.velocity_stream,fieldnames=[
            'sequence','monotonic_s','phase','weight','tick','tick_delta','raw_interval_s',
            'raw_velocity_max_rad_s','raw_velocity_joint_index','raw_velocity_joint',
            'lowstate_dq_max_rad_s','lowstate_dq_joint_index','lowstate_dq_joint',
            '20ms_interval_s','20ms_velocity_max_rad_s','20ms_velocity_joint_index','20ms_velocity_joint',
            '50ms_interval_s','50ms_velocity_max_rad_s','50ms_velocity_joint_index','50ms_velocity_joint',
            '100ms_interval_s','100ms_velocity_max_rad_s','100ms_velocity_joint_index','100ms_velocity_joint'])
        self.cw.writerow(['monotonic_s','phase','weight']+[n+'_rad' for n in NAMES])
        self.mw.writerow(['monotonic_s','phase','tick','lowstate_age_s','weight','tracking_error_rad',
                         'q0_deviation_rad','raw_clearance_m','collision_passed']+
                         [n+'_rad' for n in NAMES]+[n+'_dq_rad_s' for n in NAMES])
        self.vw.writeheader()
        self.sw.writerow(['monotonic_s','phase','joint_index','joint_name','q0_rad',
                          'q_actual_rad','q_command_rad','q_actual_minus_q0_rad',
                          'q_command_minus_q0_rad','weight','dq_command_rad_s',
                          'tau_ff_nm','kp','kd'])
        self.lw.writerow(['monotonic_s','phase','joint_index','joint_name','q0_rad',
                          'q_actual_rad','q_command_rad','q_actual_minus_q0_rad',
                          'q_command_minus_q0_rad','weight','kp','kd','imu_roll_rad',
                          'imu_pitch_rad'])

    def reset_baseline(self,snapshot):
        self.q0=np.asarray(snapshot['all_q']);validate_upper(self.q0)
        self.base=self.model.qpos0.copy()
        for i,n in enumerate(NAMES):self.base[self.model.joint(n).qposadr[0]]=self.q0[i]
        started=time.monotonic()
        distances,timing=measured_pose_clearances(
            self.model,self.data,self.base,self.pairs,self.q0)
        self.baseline=distances;completed=time.monotonic()
        self.previous_target=self.q0.copy()
        return {'started_monotonic_s':started,'completed_monotonic_s':completed,
                'duration_s':completed-started,'fk_s':timing['fk_s'],
                'collision_s':timing['collision_s'],'pair_count':len(self.pairs),
                'existing_penetration_pairs':[k for k,v in distances.items() if v<0],
                'existing_dangerous_pairs':[k for k,v in distances.items() if v<CLEARANCE],
                'new_penetration_pairs':[],'new_dangerous_pairs':[],
                'minimum_clearance_m':min(distances.values())}

    def make_collision_worker(self,period_s=0.):
        return CollisionWorker(self.reader,self.model,self.data,self.base,self.pairs,
                               self.baseline,period_s,
                               context_provider=lambda:dict(self.current_context))

    def reset_velocity_sampling(self):
        history=self.reader.motion_history_snapshot()
        self.last_velocity_sequence=(int(history[-1]['sequence']) if history else 0)

    def capture_velocity(self,phase,weight):
        observations,last=velocity_observations(
            self.reader.motion_history_snapshot(self.last_velocity_sequence),
            self.last_velocity_sequence)
        self.last_velocity_sequence=last
        for record in observations:
            record.update(phase=phase,weight=float(weight))
            self.velocity_records.append(record);self.vw.writerow(record)
        result={'sample_count':len(observations),'warning_reference_rad_s':MAX_SPEED,
                'hard_stop_applied':False}
        for key in ('raw_velocity_max_rad_s','lowstate_dq_max_rad_s',
                    '20ms_velocity_max_rad_s','50ms_velocity_max_rad_s',
                    '100ms_velocity_max_rad_s'):
            values=[row[key] for row in observations if row.get(key) is not None]
            result[key]=max(values) if values else None
        result['warning']=any(value is not None and value>MAX_SPEED for key,value in result.items()
                              if key.endswith('_velocity_max_rad_s') or key=='lowstate_dq_max_rad_s')
        return result

    def sample(self,phase,target,check=True):
        state=self.reader.get();validate_state(
            state,hackathon_suspended_mode=bool(
                getattr(self,'hackathon_suspended_mode',False)))
        q=np.array([m.q for m in state.motor_state[:29]])
        dq=np.array([m.dq for m in state.motor_state[:29]])
        target=np.asarray(target,dtype=float)
        if np.max(abs(target[:12]-self.q0[:12]))>1e-12:
            raise RuntimeError('MotionDecode trajectory reached leg command targets')
        imu_roll=float(state.imu_state.rpy[0]);imu_pitch=float(state.imu_state.rpy[1])
        validate_upper(q)
        if self.ownership_watchdog is None:
            raise RuntimeError('Independent ownership watchdog is required')
        ownership=self.ownership_watchdog.snapshot()
        now=time.monotonic()
        with self.reader.lock:age=now-self.reader.latest[0]
        if self.collision_worker is None or self.collision_stale_s is None:
            raise RuntimeError('Collision worker is required')
        collision=self.collision_worker.snapshot(self.collision_stale_s)
        latest=collision['latest'] or {}
        limits=joint_limit_violations(q)
        tracking=float(np.max(abs(q[12:]-self.previous_target[12:])))
        deviation=float(np.max(abs(q-self.q0)))
        leg_deviation=float(np.max(abs(q[:12]-self.q0[:12])))
        velocity=({'sample_count':0,'hard_stop_applied':False,
                   'mode':'disabled in prevalidated fast loop'}
                  if getattr(self,'prevalidated_runtime',False) else
                  self.capture_velocity(phase,self.writer.weight))
        lowstate_dq_peak=float(np.max(abs(dq[12:])))
        hard_reasons=[]
        if age>.15:hard_reasons.append('LowState stale')
        if not ownership['ownership_safe']:hard_reasons.append('ownership unsafe')
        if limits:hard_reasons.append('joint limit violation')
        if not collision['collision_safe']:
            hard_reasons.extend(collision['latched_reasons'] or
                                (['collision worker stale'] if collision['stale'] else ['collision worker unsafe']))
        hackathon=bool(getattr(self,'hackathon_suspended_mode',False))
        if tracking>TRACKING_LIMIT and not hackathon:
            hard_reasons.append('large tracking error')
        if leg_deviation>HOLD_DEVIATION_LIMIT and not hackathon:
            hard_reasons.append('unexpected leg movement >0.03 rad')
        settling_warning=(phase in ('ACQUIRE','ACQUIRE_RAMP','HOLD','RELEASE') and
                          deviation>HOLD_DEVIATION_LIMIT and
                          (self.release_settling_warning_only or
                           hackathon))
        if (phase in ('ACQUIRE','ACQUIRE_RAMP','HOLD','RELEASE') and
                deviation>HOLD_DEVIATION_LIMIT and not settling_warning):
            hard_reasons.append('HOLD deviation >0.03 rad')
        hard={'passed':not hard_reasons,'reasons':hard_reasons,
              'new_penetration_pairs':latest.get('new_penetration',[]),
              'new_dangerous_pairs':latest.get('new_dangerous_pair',[]),
              'joint_limit_violations':limits,'tracking_error_rad':tracking,
              'leg_q0_deviation_rad':leg_deviation,
              'lowstate_dq_max_rad_s':lowstate_dq_peak,
              'velocity_hard_stop_applied':False,
              'lowstate_age_s':float(age),'ownership_safe':ownership['ownership_safe'],
              'collision_worker_safe':collision['collision_safe']}
        relative=latest.get('baseline_relative_values') or {
            'existing_pairs':{},'legacy_threshold_m':BASELINE_TOLERANCE,
            'legacy_threshold_exceeded_pairs':[],'calibrated_threshold_applied':False,
            'stop_policy_changed':False}
        violations=sorted(set(hard['new_penetration_pairs']+hard['new_dangerous_pairs']+
                              relative['legacy_threshold_exceeded_pairs']))
        action=(ownership.get('current') or {}).get('arm_action',{})
        soft_warnings=[]
        if tracking>TRACKING_LIMIT:soft_warnings.append('tracking error >0.1 rad')
        if leg_deviation>HOLD_DEVIATION_LIMIT:
            soft_warnings.append('leg q0 deviation >0.03 rad; leg command remains NONE')
        if settling_warning:soft_warnings.append(f'{phase} q0 settling deviation >0.03 rad')
        row={'monotonic_s':now,'phase':phase,'q':q.tolist(),'dq':dq.tolist(),
             'target_q':target.tolist(),'imu_roll_rad':imu_roll,'imu_pitch_rad':imu_pitch,
             'age_s':age,'weight':self.writer.weight,'tracking_error_rad':tracking,
             'q0_deviation_rad':deviation,'leg_q0_deviation_rad':leg_deviation,
             'minimum_clearance_m':latest.get('minimum_clearance_m'),
             'collision_violations':violations,'ownership_passed':ownership['ownership_safe'],
             'hard_safety':hard,'baseline_relative':relative,
             'collision_result_age_s':collision['result_age_s'],
             'collision_last_state_tick':collision['last_state_tick'],
             'collision_latched_fault':collision['latched_collision_fault'],
             'soft_warnings':soft_warnings,
             'cleanup_warnings':soft_warnings,
             'velocity_observation':velocity,
             'arm_action_status':action.get('status','UNKNOWN'),
             'arm_action_telemetry_unknown':action.get('status')=='UNKNOWN',
             'violating_pair_distances':{key:{'baseline_m':self.baseline[key],
                                            'measured_m':relative['existing_pairs'].get(key,{}).get('measured_m'),
                                            'delta_m':relative['existing_pairs'].get(key,{}).get('delta_from_baseline_m')}
                                         for key in violations if key in self.baseline}}
        self.rows.append(row)
        self.ownership_rows.append({'monotonic_s':now,'passed':ownership['ownership_safe'],
                                   'reasons':ownership['reasons'],
                                   'last_successful_check_age_s':ownership['last_successful_check_age_s'],
                                   'latched_fault':ownership['latched_fault']})
        self.mw.writerow([now,phase,state.tick,age,self.writer.weight,tracking,deviation,
                          latest.get('minimum_clearance_m'),hard['passed']]+q.tolist()+dq.tolist())
        i=16
        self.sw.writerow([now,phase,i,NAMES[i],self.q0[i],q[i],target[i],
                          q[i]-self.q0[i],target[i]-self.q0[i],self.writer.weight,
                          0.,0.,60.,1.5])
        for i in range(12):
            self.lw.writerow([now,phase,i,NAMES[i],self.q0[i],q[i],target[i],
                              q[i]-self.q0[i],target[i]-self.q0[i],self.writer.weight,
                              0.,0.,imu_roll,imu_pitch])
        if check:
            # The comparison and its measurements remain unchanged. An explicit
            # HOLD-only invocation may log existing-pair micro-worsening while
            # retaining every hard-safety stop.
            enforce_collision_assessment({'hard_safety':hard,'baseline_relative':relative},
                                         self.baseline_worsening_log_only)
        return q

    def send(self,phase,q,weight):
        self.current_context={'phase':phase,'weight':float(weight)}
        self.writer.write(q,weight)
        now=time.monotonic();self.cw.writerow([now,phase,weight]+list(q))
        self.command_rows.append({'monotonic_s':now,'phase':phase,'weight':float(weight)})
        self.previous_target=np.asarray(q).copy()

    def phase(self,name,seconds,pose,weight):
        print('PHASE '+name,flush=True)
        def cycle(k,u,cycle_started):
            q=pose(u);command_weight=float(weight(u))
            self.current_context={'phase':name,'weight':command_weight}
            self.sample(name,q)
            self.send(name,q,command_weight)
        try:
            result=run_periodic_loop(
                seconds,1./DT,cycle,
                enforce_deadline=not getattr(self,'hackathon_suspended_mode',False))
        except BaseException as exc:
            self.fast_loop_timings.extend(
                {'phase':name,**row} for row in getattr(exc,'timings',[]))
            self.phase_terminations.append({'phase':name,
                'termination_reason':getattr(exc,'reason','exception'),
                'exception':str(exc),'traceback':traceback.format_exc()})
            raise
        self.fast_loop_timings.extend({'phase':name,**row} for row in result.timings)
        self.phase_terminations.append({'phase':name,
            'termination_reason':result.termination_reason,'exception':None,
            'requested_duration_s':seconds,'actual_duration_s':result.actual_duration_s})
        return result

    def abort_release(self):
        # No q0 return on a fault. Track the latest fresh measured hold target
        # while reducing weight at the same 0.2/s rate. If state is unavailable,
        # immediately send weight=0 with last initialized angles and zero gains.
        initial_weight=self.writer.weight
        if getattr(self,'prevalidated_runtime',False):
            steps=max(1,int(np.ceil(initial_weight*self.release_duration_s/DT)))
            for k in range(steps+1):
                state=self.reader.get();measured=np.array([m.q for m in state.motor_state[:29]])
                validate_upper(measured);q=self.q0.copy();q[12:]=measured[12:]
                weight=max(0.,initial_weight*(1-k/steps))
                self.send('abort_measured_hold_release',q,weight)
                try:
                    self.sample('abort_measured_hold_release',q,check=False)
                except RecoverableMotionEnvelopeExceeded:
                    # The envelope violation caused this release. Continue to
                    # weight zero; postflight rechecks all hard-safety gates.
                    pass
                if k<steps:time.sleep(DT)
            return
        steps=max(1,int(np.ceil(initial_weight*RAMP_S/DT)))
        try:
            for k in range(steps+1):
                state=self.reader.get();q=np.array([m.q for m in state.motor_state[:29]])
                validate_upper(q)
                self.send('abort_measured_hold_release',q,initial_weight*(1-k/steps))
                # Save feedback even if the reason for abort persists.
                self.sample('abort_measured_hold_release',q,check=False)
                if k<steps:time.sleep(DT)
        except BaseException:
            cmd=make_hold_initialized_command(
                self.writer.factory,self.writer.crc,self.writer.last_q,0.,
                getattr(self.writer,'mode_machine',0))
            for m in cmd.motor_cmd[:29]:m.kp=0.;m.kd=0.
            cmd.crc=self.writer.crc.Crc(cmd)
            ok=False
            for _ in range(3):
                ok=self.writer.publisher.Write(cmd) is not False or ok
                time.sleep(DT)
            self.writer.zero_sent=ok;self.writer.weight=0.
            self.current_context={'phase':'POST-RELEASE','weight':0.}
            self.cw.writerow([time.monotonic(),'abort_no_feedback_zero_gains',0.]+self.writer.last_q.tolist())

    def observe(self,name,seconds,target):
        print('PHASE '+name,flush=True);start=time.monotonic()
        self.current_context={'phase':name,'weight':float(self.writer.weight)}
        while True:
            self.sample(name,target)
            if time.monotonic()-start>=seconds:return
            time.sleep(DT)

    def close_logs(self):
        self.command_stream.close();self.measured_stream.close();self.velocity_stream.close()
        self.shoulder_stream.close();self.leg_stream.close()


def timing_distribution(records,key):
    values=[float(row[key]) for row in records if row.get(key) is not None]
    if not values:return {'count':0,'mean_s':None,'p95_s':None,'p99_s':None,'max_s':None}
    return {'count':len(values),'mean_s':float(np.mean(values)),
            'p95_s':float(np.percentile(values,95)),
            'p99_s':float(np.percentile(values,99)),'max_s':max(values)}


def first_collision_occurrences(records,event_field):
    if event_field not in ('new_dangerous_pair','new_penetration'):
        raise ValueError('Unsupported collision event field')
    first={}
    for row in records:
        pairs=sorted(set(row.get(event_field,[])))
        for pair in pairs:
            if pair in first:continue
            first[pair]={'pair':pair,
                         'state_sample_monotonic_s':row['started_monotonic_s'],
                         'detected_monotonic_s':row['completed_monotonic_s'],
                         'phase':row.get('phase','UNKNOWN'),
                         'weight':row.get('weight'),
                         'signed_distance_m':row.get('new_pair_distances_m',{}).get(pair),
                         'event':event_field}
    return list(first.values())


def velocity_summary(records):
    summary={'sample_count':len(records),'runtime_velocity_hard_stop':'NONE',
             'offline_command_velocity_limit_rad_s':MAX_SPEED}
    for key in ('raw_velocity_max_rad_s','lowstate_dq_max_rad_s',
                '20ms_velocity_max_rad_s','50ms_velocity_max_rad_s',
                '100ms_velocity_max_rad_s'):
        available=[row for row in records if row.get(key) is not None]
        if not available:
            summary[key]=None;continue
        row=max(available,key=lambda value:value[key]);prefix=key.replace('_max_rad_s','')
        summary[key]={'value':row[key],'joint':row.get(prefix+'_joint'),
                      'joint_index':row.get(prefix+'_joint_index'),
                      'monotonic_s':row['monotonic_s'],'tick':row['tick'],
                      'phase':row.get('phase'),'weight':row.get('weight')}
    return summary


def wait_for_fresh_idle_ownership(reader,timeout_s=10.,own_writer=False,
                                  allow_robot_internal_idle_traffic=False):
    """Let telemetry recover after the CPU-heavy receive-only benchmark.

    Only a sole UNKNOWN Arm Action condition is retryable.  An ACTIVE action,
    external writer, missing expected participant, or any other reason blocks
    immediately.  Success still requires the normal fresh-IDLE snapshot.
    """
    deadline=time.monotonic()+timeout_s;latest=None
    while time.monotonic()<deadline:
        if not own_writer and not allow_robot_internal_idle_traffic:
            latest=reader.ownership_snapshot()
        else:
            latest=reader.ownership_snapshot(
                own_writer=own_writer,
                allow_robot_internal_idle_traffic=allow_robot_internal_idle_traffic)
        if latest['passed']:return latest
        if (latest['reasons']!=['Arm Action is not confirmed idle'] or
                latest['arm_action']['status']!='UNKNOWN'):
            raise RuntimeError('Arm control ownership blocked: '+'; '.join(latest['reasons']))
        time.sleep(.05)
    raise RuntimeError('Arm control ownership blocked after telemetry recovery: '+
                       '; '.join((latest or {}).get('reasons',[]) or ['no fresh IDLE']))


EXPERIMENTAL_WARNING_CHECKS={
    'velocity','acceleration','max_delta_le_0_8','clip_range',
}


def suspended_experimental_profile(args):
    name=getattr(args,'named_reaction_profile',None)
    if not name:return False
    return load_trajectory_profile(name).get('validation_mode')=='hackathon_suspended'


def experimental_preflight(report):
    checks=report['checks']
    hard={key:value for key,value in checks.items() if key not in EXPERIMENTAL_WARNING_CHECKS}
    warnings={key:value for key,value in checks.items() if key in EXPERIMENTAL_WARNING_CHECKS}
    return {'hard_safety_passed':bool(all(hard.values())),
            'hard_checks':hard,'warning_checks':warnings,
            'normal_validation_passed':bool(report['passed'])}


def show_suspended_banner(event,report,profile):
    lines=['','============================================================',
           'SUSPENDED EXPERIMENTAL TEST','','NOT A VALIDATED REACTION','',
           'Motion:',profile['source'],'',
           'Arms:','100%','','Waist:','100%','',
           'Legs:','MotionDecode command NONE','',
           f"Max velocity: {report['max_velocity_rad_s']:.5f} rad/s",
           'NORMAL LIMIT EXCEEDED','',
           f"Max acceleration: {report['max_acceleration_rad_s2']:.5f} rad/s^2",
           'NORMAL LIMIT EXCEEDED','',
           f"Max delta: {report['max_delta_rad']:.5f} rad",
           'NORMAL LIMIT EXCEEDED','',
           'Joint limits: PASS','Collision: PASS',
           f"Minimum clearance: {report['collision']['minimum_clearance_m']:.5f} m",'',
           'THIS PROFILE WILL NOT BE ADDED TO THE REACTION LIBRARY',
           '============================================================','']
    for line in lines:event(line)


def prerequisites(args):
    motion_layout=getattr(args,'motion_layout','right-arm')
    waist_scale=getattr(args,'waist_scale',None)
    named_profile=getattr(args,'named_reaction_profile',None)
    final_gate_token=getattr(args,'final_gate_token',None)
    experimental=suspended_experimental_profile(args)
    fast_reaction=bool(getattr(args,'fast_reaction',False))
    arm_action_stale_s=getattr(args,'arm_action_stale_s',ARM_ACTION_STALE_S)
    if not np.isfinite(arm_action_stale_s) or arm_action_stale_s<=0:
        raise ValueError('--arm-action-stale-s must be finite and positive')
    if args.hold_baseline_worsening_log_only and args.stage!='hold':
        raise ValueError('--hold-baseline-worsening-log-only is valid only for HOLD')
    if args.stage=='hold' and motion_layout!='right-arm':
        raise ValueError('HOLD has no MotionDecode layout')
    allowed_waist=(0.,.125,.25,.5,1.) if named_profile else (0.,.25,.5,1.)
    if waist_scale is not None and (motion_layout!='upper-body' or waist_scale not in allowed_waist):
        raise ValueError('Waist scale is unsupported for this motion layout/profile')
    if args.stage=='full' and not args.confirm_full_real_robot:
        raise ValueError('FULL requires --confirm-full-real-robot; never automatically advanced')
    if experimental:
        if (args.stage!='full' or waist_scale!=1.0 or
                not getattr(args,'confirm_suspended_experimental',False)):
            raise ValueError('Suspended experiment requires full/waist=1 and explicit confirmation')
        expected_token=load_trajectory_profile(named_profile).get('manual_gate_token')
        if final_gate_token!=expected_token:
            raise ValueError('Suspended experiment requires its exact attended final gate')
        if fast_reaction and not load_trajectory_profile(named_profile).get('fast_reaction'):
            raise ValueError('Fast reaction is not enabled for this experimental profile')
    elif getattr(args,'confirm_suspended_experimental',False):
        raise ValueError('Suspended experimental confirmation is profile-specific')
    elif fast_reaction:
        raise ValueError('Fast reaction is limited to hackathon suspended profiles')
    if args.execute and not args.confirm_site_ready:
        raise ValueError('Execution requires operator-confirmed clear workspace and immediate physical stop')
    if final_gate_token is not None and (not args.execute or not final_gate_token.strip()):
        raise ValueError('Final gate token requires execution and must be non-empty')
    if args.execute and args.stage in ('scaled','half','full') and not experimental:
        if not args.previous_stage_result or not args.confirm_previous_stage_observed:
            raise ValueError('Previous stage PASS and human observation confirmation are required')
        previous=json.loads(args.previous_stage_result.read_text())
        expected={'scaled':'hold','half':'scaled','full':'half'}[args.stage]
        if (not named_profile and args.stage=='half' and
                motion_layout=='upper-body' and waist_scale==.25):
            expected='half'
        if previous.get('stage')!=expected or previous.get('status')!='PASS' or not previous.get('executed'):
            raise ValueError('Previous independent real stage did not PASS')
        if previous.get('ownership_released') is not True:
            raise ValueError('Previous ownership release unconfirmed')
        if args.stage!='scaled' and previous.get('motion_layout','right-arm')!=motion_layout:
            raise ValueError('Previous stage motion layout does not match')
        if (args.stage=='half' and motion_layout=='upper-body' and waist_scale==.25 and
                not named_profile and previous.get('waist_scale')!=0.):
            raise ValueError('Arms-only 50% PASS is required before waist 25%')
        expected_named=('found' if experimental else named_profile)
        if (named_profile and expected!='hold' and
                previous.get('named_reaction')!=expected_named):
            raise ValueError('Previous stage named reaction does not match')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',choices=['hold','scaled','half','full'],required=True)
    p.add_argument('--motion-layout',choices=['right-arm','upper-body'],default='right-arm')
    p.add_argument('--named-reaction-profile',
                   help='Use a trimmed named trajectory profile through this same runtime')
    p.add_argument('--waist-scale',type=float,
                   help='Upper-body waist delta scale; arm scale continues to come from --stage')
    p.add_argument('--network-interface',required=True)
    p.add_argument('--discovery-peer',action='append',default=[],
                   help='Optional process-local DDS discovery peer; does not change OS networking')
    p.add_argument('--expected-arm-pid',type=int,required=True,
                   help='PID from the immediately preceding read-only internal Arm service inventory')
    p.add_argument('--arm-action-stale-s',type=float,default=ARM_ACTION_STALE_S,
                   help='Arm Action freshness limit; keep default unless measured read-only first')
    p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--confirm-site-ready',action='store_true')
    p.add_argument('--previous-stage-result',type=Path)
    p.add_argument('--confirm-previous-stage-observed',action='store_true')
    p.add_argument('--confirm-full-real-robot',action='store_true')
    p.add_argument('--confirm-suspended-experimental',action='store_true',
                   help='One-run manual CLI gate for found 100% suspended experiment')
    p.add_argument('--final-gate-token',
                   help='Pause after offline validation and require this exact stdin token before publisher creation')
    p.add_argument('--hold-baseline-worsening-log-only',action='store_true',
                   help='HOLD only: log existing baseline-pair micro-worsening; hard stops remain active')
    p.add_argument('--prevalidated-runtime',action='store_true',
                   help='No MuJoCo worker during commands; synchronous full sweep remains required')
    p.add_argument('--fast-reaction',action='store_true',
                   help='Hackathon suspended only: shorten smooth acquire/hold/return/release phases')
    p.add_argument('--acquire-ramp-s',type=float,default=0.,
                   help='Fixed-q0 0->1 weight ramp; 0 keeps official immediate acquire')
    args=p.parse_args();prerequisites(args)
    experimental=suspended_experimental_profile(args)
    fast_reaction=bool(args.fast_reaction)
    if args.acquire_ramp_s and not .3<=args.acquire_ramp_s<=.5:
        raise ValueError('Acquire ramp must be 0 or 0.3..0.5 seconds')
    effective_acquire_s=(FAST_ACQUIRE_S if fast_reaction else args.acquire_ramp_s)
    effective_hold_s=(FAST_HOLD_S if fast_reaction else HOLD_S)
    effective_release_s=(FAST_RELEASE_S if fast_reaction else RELEASE_S)
    effective_post_release_s=(FAST_POST_RELEASE_S if fast_reaction else .5)
    folder=output_path(args.run_dir/'stage_result.json').parent
    if any(folder.iterdir()):raise ValueError('Run directory must be new and empty')
    log=(folder/'runtime.log').open('w',buffering=1)
    def event(s):print(s,flush=True);log.write(time.strftime('%Y-%m-%d %H:%M:%S')+' '+s+'\n')
    result={'schema':'motiondecode-test.real-stage.v1','stage':args.stage,'executed':False,
            'status':'BLOCKED','command_publishers_created':0,'commands_sent':0,
            'ownership_released':False,'unexpected_movement':'NOT OBSERVED BY SOFTWARE; human review required',
            'full_real_robot_test_ready':False,'navigation':'NONE','SLAM':'NONE',
            'walking':'NONE','motion_action_api':'NONE',
            'candidate':'EESB_Frustration_00001 [5552,5613)',
            'named_reaction':args.named_reaction_profile,
            'motion_layout':args.motion_layout,
            'arm_scale':{'hold':0.,'scaled':.25,'half':.5,'full':1.}[args.stage],
            'waist_scale':(args.waist_scale if args.waist_scale is not None else
                           ({'hold':0.,'scaled':.25,'half':.5,'full':1.}[args.stage]
                            if args.motion_layout=='upper-body' else None)),
            'baseline_worsening_policy':('LOG ONLY (this HOLD invocation)'
                                         if args.hold_baseline_worsening_log_only else 'STOP'),
            'execution_strategy':('prevalidated trajectory' if args.prevalidated_runtime
                                  else 'synchronous preflight + collision worker'),
            'acquire_ramp_s':effective_acquire_s,
            'fast_reaction':fast_reaction,
            'suspended_experimental_100':experimental,
            'expected_arm_service_pid':str(args.expected_arm_pid),
            'thresholds':{'tracking_rad':TRACKING_LIMIT,'hold_deviation_rad':HOLD_DEVIATION_LIMIT,
                          'baseline_numerical_tolerance_m':BASELINE_TOLERANCE,
                          'clearance_m':CLEARANCE,
                          'arm_action_stale_s':args.arm_action_stale_s}}
    reader=writer=runtime=ownership_watchdog=collision_worker=benchmark_worker=None
    gc_disabled=False
    benchmark_watchdog=None;benchmark_ownership_records=[]
    collision_benchmark_records=[];full_collision_sweep=None
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda s,f:(_ for _ in ()).throw(KeyboardInterrupt(f'signal {s}')))
    lock=(ROOT/'output/robot.lock').open('a')
    try:
        acquire_file_lock(lock)
        event('RECEIVE ONLY: observe fresh LowState/ownership for 15 seconds')
        reader=ReadOnlyState(args.network_interface,args.discovery_peer,args.expected_arm_pid,
                             args.arm_action_stale_s)
        snapshot=observe(reader,folder,args.network_interface,
                         allow_robot_internal_idle_traffic=experimental)
        scale={'hold':0.,'scaled':.25,'half':.5,'full':1.}[args.stage]
        if args.stage=='hold':
            event('HOLD ONLY: validate fresh q0 target; reaction path is not loaded')
            paths=full=durations=None
            full_report={'status':'NOT RUN FOR HOLD','passed':None}
            stage_report=evaluate_hold_target(snapshot)
        else:
            if args.named_reaction_profile:
                waist=(args.waist_scale if args.waist_scale is not None else scale/2)
                paths,full,durations,profile_metadata=build_named_paths(
                    snapshot,args.named_reaction_profile,scale,waist,
                    fast_reaction=fast_reaction)
                result['candidate']=profile_metadata['source']
                result['profile_metadata']=profile_metadata
            else:
                paths,full,durations=build_paths(
                    snapshot,scale,args.motion_layout,args.waist_scale)
                profile_metadata=None
            event('OFFLINE: recheck full candidate and selected stage from fresh q0')
            full_report=evaluate_paths(full,durations,snapshot,motion_layout=args.motion_layout)
            stage_report=evaluate_paths(paths,durations,snapshot,motion_layout=args.motion_layout)
        source_hash=(load_trajectory_profile(args.named_reaction_profile)['source_sha256']
                     if args.named_reaction_profile else digest(SOURCE))
        recheck={'source_sha256':source_hash,'scale':scale,
                 'arm_scale':scale,
                 'waist_scale':(args.waist_scale if args.waist_scale is not None else
                                (scale if args.motion_layout=='upper-body' else None)),
                 'derived_trajectory':args.motion_layout,'full':full_report,
                 'selected_stage':stage_report,'durations_s':durations,
                 'profile_metadata':profile_metadata if args.stage!='hold' else None,
                 'reaction_trajectory_loaded':args.stage!='hold'}
        write_json(folder/'candidate_recheck.json',recheck)
        experimental_report=(experimental_preflight(stage_report) if experimental else None)
        selected_pass=(experimental_report['hard_safety_passed'] if experimental
                       else stage_report['passed'])
        result['candidate_revalidation_passed']=(
            selected_pass if args.motion_layout=='upper-body' else full_report['passed'])
        result['full_amplitude_revalidation_passed']=full_report['passed']
        result['normal_selected_stage_preflight_passed']=stage_report['passed']
        result['selected_stage_preflight_passed']=selected_pass
        if experimental:
            result['experimental_preflight']=experimental_report
            recheck['experimental_policy']=experimental_report
            write_json(folder/'candidate_recheck.json',recheck)
        require_full=(args.stage!='hold' and args.motion_layout=='right-arm')
        if (require_full and not full_report['passed']) or not selected_pass:
            raise RuntimeError('Fresh q0 full/selected-stage offline safety BLOCKED')
        if not args.execute:
            result['status']='PREFLIGHT PASS';event('Receive-only preflight complete; no publisher created')
        else:
            if experimental:
                show_suspended_banner(
                    event,stage_report,load_trajectory_profile(args.named_reaction_profile))
            if args.final_gate_token:
                event('FINAL MANUAL GATE: offline safety PASS; publisher not created')
                if input().strip()!=args.final_gate_token:
                    raise RuntimeError('Final manual gate token did not match')
                result['trigger_monotonic_s']=time.monotonic()
            q0=np.asarray(snapshot['all_q'])
            # Allocate/compile FK before command publishing, outside the 20ms loop.
            runtime=Runtime(reader,None,snapshot,folder,args.stage,
                            baseline_worsening_log_only=(
                                args.hold_baseline_worsening_log_only or experimental),
                            prevalidated_runtime=args.prevalidated_runtime,
                            release_settling_warning_only=experimental,
                            hackathon_suspended_mode=experimental,
                            release_duration_s=effective_release_s)
            if args.prevalidated_runtime:
                collision_period=collision_stale=None
                event('PREVALIDATED MODE: heavy collision worker OFF during command loop')
            else:
                event('RECEIVE ONLY: benchmark full 175-pair worker with ownership load')
                benchmark_watchdog=OwnershipWatchdog(
                    lambda:reader.ownership_snapshot(),period_s=.1).start()
                if not benchmark_watchdog.wait_first()['ownership_safe']:
                    raise RuntimeError('Ownership unsafe during integrated collision benchmark')
                benchmark_worker=runtime.make_collision_worker(0.).start()
                benchmark_worker.wait_samples(10,15.)
                benchmark_worker.stop();collision_benchmark_records=benchmark_worker.timing_records()
                benchmark_worker=None
                benchmark_watchdog.stop()
                benchmark_ownership_records=benchmark_watchdog.timing_records()
                benchmark_watchdog=None
                benchmark_duration=timing_distribution(collision_benchmark_records,'duration_s')
                benchmark_interval=timing_distribution(collision_benchmark_records,'interval_s')
                if any(row['latched_collision_fault'] for row in collision_benchmark_records):
                    raise RuntimeError('Collision worker benchmark latched a safety fault')
                collision_period=max(.05,benchmark_duration['p99_s']*1.10)
                collision_stale=max(.30,collision_period+2*benchmark_duration['p99_s'],
                                    HOLD_S+2*DT+benchmark_duration['p99_s'])
                benchmark_report={'receive_only':True,'integrated_ownership_load':True,
                                  'pair_count':len(runtime.pairs),
                                  'evaluation_duration':benchmark_duration,
                                  'evaluation_interval':benchmark_interval,
                                  'selected_period_s':collision_period,
                                  'selected_stale_s':collision_stale,
                                  'selection_reason':'period=max(50ms, 1.10*p99 integrated duration)',
                                  'ownership_watchdog_records':benchmark_ownership_records,
                                  'records':collision_benchmark_records}
                write_json(folder/'collision_benchmark.json',benchmark_report)
                event('COLLISION WORKER: period %.3f s, stale limit %.3f s' % (
                    collision_period,collision_stale))
            ownership=wait_for_fresh_idle_ownership(
                reader,allow_robot_internal_idle_traffic=experimental)
            if experimental:
                source=(ownership['expected_robot_internal'][0].get('process_name')
                        if ownership['expected_robot_internal'] else 'UNKNOWN')
                event('rt/arm_sdk traffic: '+
                      ('YES' if ownership['robot_internal_traffic']=='ACTIVE' else 'NO'))
                event('source: '+str(source))
                event('Arm Action state: '+ownership['arm_action']['status'])
                event('external writers: '+str(len(ownership['external_arm_sdk_writers'])))
                event('active command detected: '+
                      ('YES' if ownership['robot_internal_active_command'] else 'NO'))
                event('ownership verdict: PASS')

            # Create the publisher from a valid provisional pose, but send
            # nothing until a later fresh q0 and synchronous sweep both pass.
            reader.require_stationary();state=reader.get();validate_state(state,initial=True)
            # Stationarity observation itself lasts 0.5 s, equal to the Arm
            # Action freshness window. Refresh the IDLE evidence afterwards
            # instead of taking a single boundary-racy snapshot.
            wait_for_fresh_idle_ownership(
                reader,allow_robot_internal_idle_traffic=experimental)
            provisional_q0=np.asarray(state_summary(state)['all_q'])
            writer=InitializedWriter(reader,provisional_q0);runtime.writer=writer
            result['command_publishers_created']=1
            # Wait for own discovery without sending a command; fresh checks still apply.
            deadline=time.monotonic()+2.
            while time.monotonic()<deadline:
                own=reader.ownership_snapshot(
                    own_writer=True,allow_arm_action_unknown=True,
                    allow_robot_internal_idle_traffic=experimental)
                if own['passed']:break
                if any(r!='this controller writer is not exactly one' for r in own['reasons']):
                    raise RuntimeError('Ownership changed before first command')
                reader.get();time.sleep(DT)
            wait_for_fresh_idle_ownership(
                reader,own_writer=True,
                allow_robot_internal_idle_traffic=experimental)
            if args.prevalidated_runtime:
                cached=reader.ownership_snapshot(
                    own_writer=True,allow_arm_action_unknown=True,
                    allow_robot_internal_idle_traffic=experimental)
                if not cached['passed']:
                    raise RuntimeError('Ownership preflight did not establish a safe state')
                cached={'ownership_safe':True,'reasons':[],
                        'last_successful_check_age_s':0.,'latched_fault':False,
                        'current':cached}
                ownership_watchdog=PrevalidatedOwnershipCache(cached)
            else:
                ownership_watchdog=OwnershipWatchdog(
                    lambda:reader.ownership_snapshot(
                        own_writer=True,allow_arm_action_unknown=True,
                        allow_robot_internal_idle_traffic=experimental),period_s=.1).start()
                cached=ownership_watchdog.wait_first()
                if not cached['ownership_safe']:
                    raise RuntimeError('Ownership watchdog did not establish a safe initial state')
            runtime.ownership_watchdog=ownership_watchdog

            # This is the command q0: capture it after publisher discovery and
            # immediately evaluate all 175 pairs outside the control loop.
            reader.require_stationary();state=reader.get();validate_state(state,initial=True)
            wait_for_fresh_idle_ownership(
                reader,own_writer=True,
                allow_robot_internal_idle_traffic=experimental)
            snapshot=state_summary(state)
            snapshot.update(timestamp=time.strftime('%Y-%m-%d %H:%M:%S %z'),receive_only=True,
                            captured_monotonic=time.monotonic(),command_publishers_created=1,
                            arm_sdk_commands_sent=0,
                            source_ownership_report=str(folder/'preflight.json'))
            write_json(folder/'fresh_q0.json',snapshot);q0=np.asarray(snapshot['all_q'])
            full_collision_sweep=runtime.reset_baseline(snapshot)
            full_collision_sweep.update(state_tick=int(state.tick),lowstate_age_s=0.,
                                        baseline_micro_worsening_policy='LOG ONLY for this HOLD')
            write_json(folder/'full_collision_sweep.json',full_collision_sweep)
            if (full_collision_sweep['new_penetration_pairs'] or
                    full_collision_sweep['new_dangerous_pairs']):
                raise RuntimeError('Fresh q0 full collision sweep failed')
            if args.prevalidated_runtime:
                reader.wait(1.)
                reader.require_stationary();state=reader.get();validate_state(state,initial=True)
                precommand_q0_delta=float(np.max(
                    abs(np.asarray(state_summary(state)['all_q'])-q0)))
                result['precommand_q0_delta_rad']=precommand_q0_delta
                if precommand_q0_delta>HOLD_DEVIATION_LIMIT:
                    if experimental:
                        event('WARNING: q changed >0.03 rad after prevalidated full sweep')
                    else:
                        raise RuntimeError('q changed after prevalidated full sweep')
                ownership=wait_for_fresh_idle_ownership(
                    reader,own_writer=True,
                    allow_robot_internal_idle_traffic=experimental)
                reader.wait(.5);state=reader.get();validate_state(state)
                ownership_watchdog=OwnershipWatchdog(
                    lambda:reader.ownership_snapshot(
                        own_writer=True,allow_arm_action_unknown=True,
                        allow_robot_internal_idle_traffic=experimental),period_s=.1).start()
                cached=ownership_watchdog.wait_first()
                if not cached['ownership_safe']:
                    raise RuntimeError('Ownership watchdog did not establish a safe initial state')
                runtime.ownership_watchdog=ownership_watchdog
                runtime.collision_worker=PrevalidatedCollisionCache(full_collision_sweep)
                runtime.collision_stale_s=float('inf')
            else:
                collision_worker=runtime.make_collision_worker(collision_period)
                collision_worker.seed_safe_result(full_collision_sweep);collision_worker.start()
                runtime.collision_worker=collision_worker;runtime.collision_stale_s=collision_stale
            runtime.reset_velocity_sampling()
            gc.collect();gc.disable();gc_disabled=True

            event(f'EXECUTE STAGE {args.stage}: initialized targets, guarded acquire')
            result['executed']=True
            result['q0_captured_monotonic_s']=snapshot['captured_monotonic']
            if effective_acquire_s:
                result['acquire_method']='fixed fresh q0 target with smooth 0->1 weight ramp'
                runtime.phase('ACQUIRE_RAMP',effective_acquire_s,lambda u:q0,lambda u:u)
            else:
                result['acquire_method']='official-style immediate weight=1 with fresh measured q'
                runtime.phase('ACQUIRE',DT,lambda u:q0,lambda u:1.)
            runtime.phase('HOLD',effective_hold_s,lambda u:q0,lambda u:1.)
            result['hold_completed']=True
            if args.stage!='hold':
                result['motion_start_monotonic_s']=time.monotonic()
                for phase,path in paths.items():
                    times=np.linspace(0,durations[phase],len(path))
                    def pose(u,path=path,times=times):
                        target=q0.copy()
                        start=12 if args.motion_layout=='upper-body' else 15
                        target[start:]=interpolate(times,path,u*times[-1]);return target
                    runtime.phase(phase,durations[phase],pose,lambda u:1.)
                result['motion_completed']=True
                runtime.phase('q0_hold_after',effective_hold_s,lambda u:q0,lambda u:1.)
            runtime.phase('RELEASE',effective_release_s,lambda u:q0,lambda u:1-u)
            result['release_completed']=True
            runtime.observe('POST-RELEASE',effective_post_release_s,q0)
            result['post_release_observation_completed']=True
            result['execution_completed_monotonic_s']=time.monotonic()
            gc.enable();gc_disabled=False
            result['status']='PASS'
    except BaseException as exc:
        event('STOP: '+str(exc));result['reason']=str(exc)
        result['status']='FAIL' if writer and writer.sent else 'BLOCKED'
        if writer and runtime:
            event('Abort: latest measured hold / weight release, no q0 return')
            try:runtime.abort_release()
            except BaseException as release_exc:result['release_error']=str(release_exc)
    finally:
        if gc_disabled:gc.enable()
        if benchmark_worker:
            try:benchmark_worker.stop()
            except BaseException as exc:result['benchmark_worker_stop_error']=str(exc)
        if benchmark_watchdog:
            try:benchmark_watchdog.stop()
            except BaseException as exc:result['benchmark_watchdog_stop_error']=str(exc)
        if collision_worker:
            try:collision_worker.stop();result['collision_worker_stopped']=True
            except BaseException as exc:result['collision_worker_stop_error']=str(exc)
            collision_records=collision_worker.timing_records()
            write_json(folder/'collision_worker_timing.json',collision_records)
            result['collision_evaluation_duration']=timing_distribution(collision_records,'duration_s')
            result['collision_evaluation_interval']=timing_distribution(collision_records,'interval_s')
            result['dangerous_pair_first_occurrences']=first_collision_occurrences(
                collision_records,'new_dangerous_pair')
            result['penetration_first_occurrences']=first_collision_occurrences(
                collision_records,'new_penetration')
        if collision_benchmark_records:
            result['collision_benchmark_duration']=timing_distribution(
                collision_benchmark_records,'duration_s')
            result['collision_benchmark_interval']=timing_distribution(
                collision_benchmark_records,'interval_s')
        if full_collision_sweep:result['full_collision_sweep']=full_collision_sweep
        if ownership_watchdog:
            ownership_watchdog.stop()
            if runtime:
                write_json(folder/'ownership_watchdog_timing.json',
                           ownership_watchdog.timing_records())
        if writer:
            result['executed']=bool(writer.sent)
            result['commands_sent']=writer.sent;result['zero_weight_sent']=writer.zero_sent
            result['maximum_weight_sent']=writer.maximum_weight
            result['positive_weight_command_count']=writer.positive_weight_count
            result['first_positive_weight_monotonic_s']=writer.first_positive_monotonic
            result['first_full_weight_monotonic_s']=writer.first_full_weight_monotonic
            result['hold_at_full_weight_completed']=bool(result.get('hold_completed'))
            result['hold_duration_s']=effective_hold_s if result.get('hold_completed') else 0.
            try:writer.close();result['publisher_closed']=True
            except BaseException as exc:result['close_error']=str(exc)
        if reader and writer:
            try:
                deadline=time.monotonic()+2.
                while time.monotonic()<deadline:
                    ownership=reader.ownership_snapshot(
                        allow_robot_internal_idle_traffic=experimental)
                    if ownership['passed']:break
                    time.sleep(.1)
                result['post_release_ownership']=ownership
                result['ownership_released']=bool(writer.zero_sent and result.get('publisher_closed')
                                                   and ownership['passed'])
                result['release_evidence']='zero weight published + publisher closed + no own DDS writer; no firmware ACK'
                final=state_summary(reader.get());write_json(folder/'final_state.json',final)
                delta=np.asarray(final['all_q'])-np.asarray(snapshot['all_q'])
                result['final_q_minus_q0_rad']=delta.tolist()
                result['returned_to_q0']=bool(np.max(abs(delta[12:]))<=FINAL_Q_TOLERANCE)
                result['post_release_native_controller_movement']=not result['returned_to_q0']
                if experimental and not result['returned_to_q0']:
                    result.setdefault('soft_warnings',[]).append(
                        'q0 return acceptance threshold exceeded')
                if result['status']=='PASS' and not result['ownership_released']:
                    result['status']='FAIL';result['reason']='Ownership release verification failed'
            except BaseException as exc:
                result['post_release_error']=str(exc)
                if result['status']=='PASS':result['status']='FAIL'
        if runtime:
            result['phase_terminations']=runtime.phase_terminations
            rows=runtime.rows
            if rows:
                result['max_tracking_error_rad']=max(r['tracking_error_rad'] for r in rows)
                result['max_q0_deviation_rad']=max(r['q0_deviation_rad'] for r in rows)
                result['max_leg_q0_deviation_rad']=max(r['leg_q0_deviation_rad'] for r in rows)
                leg_peak=max(
                    ((abs(r['q'][i]-runtime.q0[i]),i,r) for r in rows for i in range(12)),
                    key=lambda value:value[0])
                result['max_leg_q0_deviation']={
                    'value_rad':float(leg_peak[0]),'joint_index':int(leg_peak[1]),
                    'joint':NAMES[leg_peak[1]],'phase':leg_peak[2]['phase'],
                    'monotonic_s':leg_peak[2]['monotonic_s'],
                    'q0_rad':float(runtime.q0[leg_peak[1]]),
                    'commanded_rad':float(leg_peak[2]['target_q'][leg_peak[1]]),
                    'actual_rad':float(leg_peak[2]['q'][leg_peak[1]]),
                }
                crossings=[r for r in rows if r['leg_q0_deviation_rad']>HOLD_DEVIATION_LIMIT]
                result['first_leg_deviation_crossing']=None if not crossings else {
                    'phase':crossings[0]['phase'],'monotonic_s':crossings[0]['monotonic_s'],
                    'value_rad':crossings[0]['leg_q0_deviation_rad']}
                result['max_leg_command_deviation_rad']=float(max(
                    abs(r['target_q'][i]-runtime.q0[i]) for r in rows for i in range(12)))
                result['motiondecode_leg_command_found']=bool(
                    result['max_leg_command_deviation_rad']>1e-12)
                result['max_abs_imu_roll_rad']=max(abs(r['imu_roll_rad']) for r in rows)
                result['max_abs_imu_pitch_rad']=max(abs(r['imu_pitch_rad']) for r in rows)
                result['max_imu_roll_delta_rad']=max(
                    abs(r['imu_roll_rad']-snapshot['imu_rpy'][0]) for r in rows)
                result['max_imu_pitch_delta_rad']=max(
                    abs(r['imu_pitch_rad']-snapshot['imu_rpy'][1]) for r in rows)
                result['max_lowstate_dq_field_rad_s']=max(max(abs(v) for v in r['dq'][12:]) for r in rows)
                result['runtime_velocity_hard_stop']='NONE; position/tracking hard stops unchanged'
                result['measured_right_range_rad']=np.ptp(np.array([r['q'][22:] for r in rows]),axis=0).tolist()
                clearances=[r['minimum_clearance_m'] for r in rows
                            if r['minimum_clearance_m'] is not None]
                result['minimum_measured_clearance_m']=min(clearances) if clearances else None
                result['hard_safety_passed']=all(r['hard_safety']['passed'] for r in rows)
                result.setdefault('soft_warnings',[]).extend(sorted({
                    warning for row in rows for warning in row.get('soft_warnings',[])}))
                result['baseline_worsening_observed']=any(
                    r['baseline_relative']['legacy_threshold_exceeded_pairs'] for r in rows)
                result['collision_monitor_passed']=result['hard_safety_passed'] and (
                    args.hold_baseline_worsening_log_only or
                    not result['baseline_worsening_observed'])
                result['collision_worker_stale']=any(
                    'collision worker stale' in r['hard_safety']['reasons'] for r in rows)
                result['ownership_conflict']=any(not r['ownership_passed'] for r in rows)
                result['collision_result_age']=timing_distribution(
                    rows,'collision_result_age_s')
                phase_metrics={}
                for phase in ('ACQUIRE','ACQUIRE_RAMP','HOLD','entry','clip','return',
                              'q0_hold_after','RELEASE','POST-RELEASE',
                              'abort_measured_hold_release'):
                    selected=[r for r in rows if r['phase']==phase]
                    if not selected:continue
                    phase_velocity=[r for r in runtime.velocity_records if r['phase']==phase]
                    times=[r['monotonic_s'] for r in selected]
                    phase_metrics[phase]={
                        'samples':len(selected),
                        'observed_duration_s':max(times)-min(times) if len(times)>1 else 0.,
                        'max_tracking_error_rad':max(r['tracking_error_rad'] for r in selected),
                        'max_measured_q0_movement_rad':max(r['q0_deviation_rad'] for r in selected),
                        'max_leg_q0_deviation_rad':max(r['leg_q0_deviation_rad'] for r in selected),
                        'left_wrist_roll_movement_rad':max(abs(r['q'][19]-runtime.q0[19]) for r in selected),
                        'left_wrist_pitch_movement_rad':max(abs(r['q'][20]-runtime.q0[20]) for r in selected),
                        'left_wrist_yaw_movement_rad':max(abs(r['q'][21]-runtime.q0[21]) for r in selected),
                        'right_wrist_roll_movement_rad':max(abs(r['q'][26]-runtime.q0[26]) for r in selected),
                        'right_wrist_pitch_movement_rad':max(abs(r['q'][27]-runtime.q0[27]) for r in selected),
                        'right_wrist_yaw_movement_rad':max(abs(r['q'][28]-runtime.q0[28]) for r in selected),
                        'max_lowstate_dq_rad_s':max(r['hard_safety']['lowstate_dq_max_rad_s'] for r in selected),
                        'lowstate_max_age_s':max(r['age_s'] for r in selected),
                        'collision_result_age':timing_distribution(selected,'collision_result_age_s'),
                        'ownership_safe':all(r['ownership_passed'] for r in selected),
                        'arm_action_unknown_samples':sum(r['arm_action_telemetry_unknown'] for r in selected),
                        'collision_safe':all(r['hard_safety']['collision_worker_safe'] for r in selected),
                        'hard_safety_passed':all(r['hard_safety']['passed'] for r in selected),
                        'velocity':velocity_summary(phase_velocity),
                    }
                result['phase_metrics']=phase_metrics
                result['hold_result']='PASS' if (result.get('hold_completed') and
                    phase_metrics.get('HOLD',{}).get('hard_safety_passed')) else 'FAIL'
                result['release_safety']='PASS' if (result.get('release_completed') and
                    phase_metrics.get('RELEASE',{}).get('hard_safety_passed')) else 'FAIL'
                result['max_tracking_error_during_clip_rad']=(
                    phase_metrics.get('clip',{}).get('max_tracking_error_rad'))
                result['velocity_summary']=velocity_summary(runtime.velocity_records)
                write_json(folder/'runtime_samples.json',rows)
            if runtime.fast_loop_timings:
                intervals=[b['started_monotonic_s']-a['started_monotonic_s']
                           for a,b in zip(runtime.fast_loop_timings,runtime.fast_loop_timings[1:])]
                interval_rows=[{'interval_s':value} for value in intervals]
                result['fast_loop_interval']=timing_distribution(interval_rows,'interval_s')
                elapsed=(runtime.fast_loop_timings[-1]['started_monotonic_s']-
                         runtime.fast_loop_timings[0]['started_monotonic_s'])
                result['fast_loop_rate_hz']=((len(runtime.fast_loop_timings)-1)/elapsed
                                             if elapsed>0 else None)
                result['fast_loop_processing']=timing_distribution(
                    runtime.fast_loop_timings,'processing_s')
                if experimental and result['fast_loop_interval']['max_s']>.1:
                    result.setdefault('soft_warnings',[]).append(
                        'fast-loop interval >100 ms')
                write_json(folder/'fast_loop_timing.json',runtime.fast_loop_timings)
            result['soft_warnings']=sorted(set(result.get('soft_warnings',[])))
            result['motion_execution']=('PASS' if result.get('motion_completed')
                                        else 'HARD FAULT')
            if result.get('trigger_monotonic_s') and result.get('motion_start_monotonic_s'):
                result['trigger_to_motion_s']=(result['motion_start_monotonic_s']-
                                               result['trigger_monotonic_s'])
            if result.get('trigger_monotonic_s') and result.get('execution_completed_monotonic_s'):
                result['total_execution_time_s']=(result['execution_completed_monotonic_s']-
                                                  result['trigger_monotonic_s'])
            write_json(folder/'runtime_ownership.json',runtime.ownership_rows)
            runtime.close_logs()
        if reader:
            if not (folder/'preflight.json').exists():
                write_json(folder/'preflight.json',{'passed':False,'reason':result.get('reason'),
                                                   'command_publishers_created':0})
            try:reader.close()
            except BaseException as exc:result['reader_close_error']=str(exc)
        result['full_real_robot_test_ready']=False
        result['human_review_required']=True
        write_json(folder/'stage_result.json',result)
        event('RESULT '+json.dumps(result,ensure_ascii=False))
        log.close();lock.close()
    return 0 if result['status'] in ('PASS','PREFLIGHT PASS') else 2


if __name__=='__main__':raise SystemExit(main())
