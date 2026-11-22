#!/usr/bin/env python3
"""Strict receive-only 120-second LowState/FK baseline noise characterization.

All received unique-tick LowStates are archived. Live monitoring targets 50 Hz;
after DDS is closed, every captured LowState is replayed through the SAME FK
routine to retain high-frequency pair statistics without blocking reception.
No robot command or service client is created; DataWriter construction is denied.
"""
import argparse
import csv
import json
from pathlib import Path
import threading
import time

import numpy as np
from common import ROOT, NAMES, write_json, output_path, digest
from robot_transport import (ReadOnlyState, validate_state, state_summary, load_sdk,
                             EXPECTED_ARM_HOST, EXPECTED_ARM_IP, EXPECTED_ARM_PROCESS,
                             EXPECTED_ARM_PUBLICATIONS, EXPECTED_ARM_SUBSCRIPTIONS)
from inspect_arm_sdk_publishers import local_process_scan, interface_ipv4
from scene import make_scene, clearance_pairs, measured_pose_clearances

CURRENT_TOLERANCE_MM=.001
MINIMUM_M=.015
PREVIOUS_PAIRS=['left_wrist_yaw_link | left_hip_pitch_link',
                'left_rubber_hand | left_hip_pitch_link']


def forbid_writers():
    load_sdk()  # imports only; does not initialize DDS or a command writer
    from unitree_sdk2py.core.channel import ChannelPublisher
    from cyclonedds.pub import DataWriter
    def forbidden(*args,**kwargs):
        raise RuntimeError('RECEIVE ONLY: application Publisher/DataWriter construction forbidden')
    ChannelPublisher.__init__=forbidden
    DataWriter.__init__=forbidden


class RecordingState(ReadOnlyState):
    def __init__(self,interface,internal_profile=None,discovery_peers=()):
        self.capture_lock=threading.Lock();self.captured=[];self.capture=False
        self.capture_tick=None;self.max_capture_callback_s=0.
        self.internal_profile=internal_profile
        super().__init__(interface,discovery_peers)

    def _endpoint_record(self,endpoint,pubs,subs,participants):
        record=super()._endpoint_record(endpoint,pubs,subs,participants)
        p=self.internal_profile
        if p and all(record.get(k)==p.get(k) for k in
                     ('participant_guid','hostname','process_name','pid','source_ips',
                      'related_publications','related_subscriptions')):
            record['classification']='EXPECTED ROBOT INTERNAL PARTICIPANT'
            record['identity_scope']='this receive-only observation only; exact recorded GUID/PID/topology'
        return record

    def _receive(self,msg):
        start=time.monotonic()
        super()._receive(msg)
        with self.capture_lock:
            if not self.capture or msg.tick==self.capture_tick:return
            self.capture_tick=msg.tick
            self.captured.append((start,time.time(),int(msg.tick),
                np.array([m.q for m in msg.motor_state[:29]],dtype=float),
                np.array([m.dq for m in msg.motor_state[:29]],dtype=float)))
            self.max_capture_callback_s=max(self.max_capture_callback_s,time.monotonic()-start)


def summary(values):
    x=np.asarray(values,dtype=float)
    if len(x)==0:return None
    return {k:float(v) for k,v in {
        'mean':x.mean(),'median':np.median(x),'min':x.min(),'max':x.max(),
        'p95':np.percentile(x,95),'p99':np.percentile(x,99)}.items()}


def observed_internal_profile(path,expected_ip=EXPECTED_ARM_IP):
    """Read-only identity binding after restart, never a real-runner allowlist.

    Exactly the former internal-service topology, host and address must have
    been observed alone and idle for >=15 seconds. PID/GUID are bound to this
    recorded observation, never wildcarded at runtime.
    """
    d=json.loads(path.read_text())
    if (d.get('schema')!='motiondecode-test.arm-control-ownership.v2'
            or d.get('command_publishers_created')!=0 or not d.get('receive_only')
            or d.get('observation_actual_s',0)<15):
        raise ValueError('Need >=15-second receive-only ownership observation')
    from datetime import datetime
    age=time.time()-datetime.strptime(d['checked_local_time'],'%Y-%m-%d %H:%M:%S %z').timestamp()
    if not 0<=age<=900:raise ValueError('Internal identity observation is older than 15 minutes')
    topics=d['topics'];arm=topics['rt/arm_sdk']
    if (d['lowstate']['status']!='PASS' or d['lowstate']['stationary']!='PASS'
            or topics['rt/arm/action/state']['status_during_observation']!='IDLE'
            or topics['rt/armsdk']['publishers'] or topics['rt/armsdk']['total_sample_count']!=0
            or arm['total_sample_count']!=0 or len(arm['publishers'])!=1
            or d['local_os_scan']['candidate_processes_excluding_diagnostic']):
        raise ValueError('Observation does not contain one idle internal-topology candidate only')
    p=arm['publishers'][0]
    if (p['hostname']!=EXPECTED_ARM_HOST or p['process_name']!=EXPECTED_ARM_PROCESS
            or p['source_ips']!=[expected_ip] or p['sample_count']!=0
            or set(p['related_publications'])!=EXPECTED_ARM_PUBLICATIONS
            or set(p['related_subscriptions'])!=EXPECTED_ARM_SUBSCRIPTIONS):
        raise ValueError('Robot-internal service identity/topology mismatch')
    return p


def series_stats(x,initial):
    x=np.asarray(x);med=float(np.median(x));dev=abs(x-initial)
    return {'initial':float(initial),'mean':float(x.mean()),'median':med,
            'minimum':float(x.min()),'maximum':float(x.max()),
            'peak_to_peak':float(np.ptp(x)),'std':float(np.std(x)),
            'MAD':float(np.median(abs(x-med))),
            'maximum_baseline_worsening':float(max(0,initial-x.min())),
            'p95_absolute_deviation_from_q0':float(np.percentile(dev,95)),
            'p99_absolute_deviation_from_q0':float(np.percentile(dev,99))}


def corr(a,b):
    a=np.asarray(a);b=np.asarray(b)
    if len(a)<3 or np.std(a)<1e-15 or np.std(b)<1e-15:return None
    return float(np.corrcoef(a,b)[0,1])


def classify(pair_stats):
    # A transparent descriptive criterion, NOT a changed runtime threshold.
    if any(p['fraction_worsening_over_current_tolerance']>=.05 for p in pair_stats):return 'NOISE-A'
    if any(p['fraction_worsening_over_current_tolerance']>0 for p in pair_stats):return 'NOISE-B'
    return 'NOISE-C'


def preflight(reader,folder,interface):
    start=time.monotonic();rows=[];errors=[]
    time.sleep(1.)
    for _ in range(150):
        try:
            state=reader.get();validate_state(state,initial=True)
            owner=reader.check_exclusive();reader.require_stationary()
            rows.append({'monotonic_s':time.monotonic(),'tick':int(state.tick),'passed':True})
        except (RuntimeError,ValueError) as exc:
            errors.append(str(exc));rows.append({'monotonic_s':time.monotonic(),'passed':False,'reason':str(exc)})
        time.sleep(.1)
    owner=reader.ownership_snapshot()
    scan=local_process_scan(interface_ipv4(interface))
    if scan['candidate_processes_excluding_diagnostic']:errors.append('Local controller process candidate')
    report={'passed':not errors and owner['passed'],'duration_s':time.monotonic()-start,
            'ownership':owner,'timeline':rows,'errors':sorted(set(errors)),
            'local_process_scan':scan,'command_publishers_created':0}
    write_json(folder/'preflight.json',report)
    if not report['passed']:raise RuntimeError('Latest receive-only preflight BLOCKED: '+'; '.join(report['errors']+owner['reasons']))
    state=reader.get();validate_state(state,initial=True)
    with reader.lock:received=reader.latest[0]
    snapshot=state_summary(state)
    snapshot.update(timestamp=time.strftime('%Y-%m-%d %H:%M:%S %z'),
                    captured_monotonic_s=time.monotonic(),received_monotonic_s=received,
                    lowstate_age_s=time.monotonic()-received,receive_only=True,
                    source_ownership_report=str(folder/'preflight.json'),command_publishers_created=0)
    return snapshot


def collect(reader,folder,snapshot,seconds):
    model,data=make_scene();pairs=clearance_pairs(model);base=model.qpos0.copy()
    q0=np.array(snapshot['all_q'])
    initial,_=measured_pose_clearances(model,data,base,pairs,q0)
    keys=[key for key,d in initial.items() if d<MINIMUM_M]
    snapshot['baseline_pair_clearance_m']={key:initial[key] for key in keys}
    snapshot['current_tolerance_mm']=CURRENT_TOLERANCE_MM
    write_json(folder/'fresh_q0.json',snapshot)
    records=[];owners=[];errors=[]
    start=time.monotonic();last_progress=start;next_at=start
    with reader.capture_lock:reader.capture=True
    try:
        while time.monotonic()-start<seconds:
            time.sleep(max(0,next_at-time.monotonic()))
            cycle_start=time.monotonic()
            state=reader.get();validate_state(state,initial=True);reader.require_stationary()
            with reader.lock:
                received,msg=reader.latest
            q=np.array([m.q for m in msg.motor_state[:29]])
            if np.max(abs(q-q0))>.025:
                raise RuntimeError('q0 drift >0.025 rad: stationary-noise assumption invalid')
            owner_start=time.perf_counter();owner=reader.check_exclusive()
            ownership_s=time.perf_counter()-owner_start
            owners.append({'monotonic_s':cycle_start,'passed':owner['passed'],
                           'external_arm_sdk_writers':len(owner['external_arm_sdk_writers']),
                           'armsdk_writers':len(owner['armsdk_writers']),
                           'action':owner['arm_action']['status']})
            eval_start=time.monotonic()
            distances,timing=measured_pose_clearances(model,data,base,pairs,q)
            end=time.monotonic()
            records.append({'monotonic_s':cycle_start,'received_monotonic_s':received,'tick':int(msg.tick),
                'evaluation_age_s':eval_start-received,'cycle_s':end-cycle_start,
                'ownership_s':ownership_s,**timing,
                'clearance_m':{key:distances[key] for key in keys},
                'new_penetration_pairs':[key for key,d in distances.items() if initial[key]>=0 and d<0],
                'new_clearance_deficit_pairs':[key for key,d in distances.items() if initial[key]>=MINIMUM_M and d<MINIMUM_M]})
            next_at=max(next_at+.02,end)
            if end-last_progress>=10:
                with reader.capture_lock:count=len(reader.captured)
                print(f'RECEIVE ONLY {end-start:.1f}/{seconds:.0f}s: LowState={count}, FK={len(records)}',flush=True)
                last_progress=end
    except (RuntimeError,ValueError,KeyboardInterrupt) as exc:
        errors.append(str(exc))
    finally:
        with reader.capture_lock:
            reader.capture=False;captured=list(reader.captured)
    duration=time.monotonic()-start
    write_json(folder/'live_monitor_samples.json',records)
    write_json(folder/'ownership_samples.json',owners)
    with (folder/'lowstate_samples.csv').open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['sample_id','received_monotonic_s','received_unix_s','tick','reception_callback_age_s']+
                        [n+'_q_rad' for n in NAMES]+[n+'_dq_rad_s' for n in NAMES]+[n+'_delta_q0_rad' for n in NAMES])
        for i,(mono,unix,tick,q,dq) in enumerate(captured):
            writer.writerow([i,mono,unix,tick,0.]+q.tolist()+dq.tolist()+(q-q0).tolist())
    info={'duration_s':duration,'requested_duration_s':seconds,'errors':errors,
          'lowstate_samples':len(captured),'live_monitor_samples':len(records),
          'max_capture_callback_s':reader.max_capture_callback_s,
          'live_model':'exact shared scene.measured_pose_clearances used by run_real_reaction.Runtime.sample',
          'recording_note':'reception_callback_age_s=0 is age at receipt, not network latency. Live evaluation age is separately measured.'}
    return captured,records,initial,keys,info


def analyze(folder,snapshot,captured,live,initial,keys,info):
    if len(captured)<2 or not live:raise RuntimeError('Insufficient samples for noise statistics')
    model,data=make_scene();pairs=clearance_pairs(model);base=model.qpos0.copy()
    q0=np.asarray(snapshot['all_q']);q=np.array([r[3] for r in captured]);delta=q-q0
    times=np.array([r[0] for r in captured]);intervals=np.diff(times)
    values=[];new_penetrations=set();new_deficits=set();costs=[]
    replay_start=time.monotonic();last_progress=replay_start
    with (folder/'clearance_samples.csv').open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['sample_id','received_monotonic_s','received_unix_s','tick',
                         'evaluation_monotonic_s','evaluation_mode','replay_sample_age_s','fk_s','collision_s']+
                        [key+' [m]' for key in keys])
        for i,(mono,unix,tick,pose,dq) in enumerate(captured):
            when=time.monotonic()
            distances,cost=measured_pose_clearances(model,data,base,pairs,pose)
            row=[distances[key] for key in keys];values.append(row);costs.append(cost)
            writer.writerow([i,mono,unix,tick,when,'offline_full_reception_replay',when-mono,
                             cost['fk_s'],cost['collision_s']]+row)
            new_penetrations.update(key for key,d in distances.items() if initial[key]>=0 and d<0)
            new_deficits.update(key for key,d in distances.items() if initial[key]>=MINIMUM_M and d<MINIMUM_M)
            if time.monotonic()-last_progress>15:
                print(f'OFFLINE identical-FK replay {i+1}/{len(captured)}',flush=True);last_progress=time.monotonic()
    replay_duration=time.monotonic()-replay_start
    v=np.asarray(values)*1000;init=np.array([initial[k] for k in keys])*1000
    # Sensitivity uses the same full FK at q0 +/- 1 urad, never an alternative
    # collision model. It only explains observed joint-to-clearance variations.
    jac=np.zeros((len(keys),29))
    for j in range(29):
        up=q0.copy();down=q0.copy();up[j]+=1e-6;down[j]-=1e-6
        a,_=measured_pose_clearances(model,data,base,pairs,up)
        b,_=measured_pose_clearances(model,data,base,pairs,down)
        jac[:,j]=[(a[k]-b[k])*1000/2e-6 for k in keys]
    predicted=delta@jac.T
    pair_stats=[]
    for k,key in enumerate(keys):
        x=v[:,k];s=series_stats(x,init[k]);s.update(pair=key,units='mm',previous_failing_pair=key in PREVIOUS_PAIRS)
        worsening=init[k]-x
        s['fraction_worsening_over_current_tolerance']=float(np.mean(worsening>CURRENT_TOLERANCE_MM))
        s['fraction_absolute_deviation_over_current_tolerance']=float(np.mean(abs(x-init[k])>CURRENT_TOLERANCE_MM))
        s['sample_to_sample_abs_step_mm']=summary(abs(np.diff(x)))
        s['fraction_abs_steps_ge_0_012_mm']=float(np.mean(abs(np.diff(x))>=.012))
        s['step_vs_reception_interval_correlation']=corr(abs(np.diff(x)),intervals)
        normal=abs(np.diff(x))[intervals<=np.percentile(intervals,95)]
        gap=abs(np.diff(x))[intervals>np.percentile(intervals,95)]
        s['step_normal_intervals_mm']=summary(normal);s['step_long_intervals_mm']=summary(gap)
        med=np.median(x);center_dev=abs(x-med);bias=abs(init[k]-med)
        s['initial_minus_median_mm']=float(init[k]-med)
        s['median_centered_abs_deviation_mm']=summary(center_dev)
        s['alternate_initial_baseline_first_5s_worsening_mm']=summary(
            x[times-times[0]<=5]-x.min())
        cc=[{'joint':name,'pearson_r':corr(delta[:,j],x-init[k]),'sensitivity_mm_per_rad':float(jac[k,j])}
            for j,name in enumerate(NAMES)]
        s['joint_correlations']=sorted(cc,key=lambda z:abs(z['pearson_r'] or 0),reverse=True)
        residual=(x-init[k])-predicted[:,k]
        s['same_FK_local_sensitivity_explanation']={
            'rmse_mm':float(np.sqrt(np.mean(residual**2))),
            'max_abs_residual_mm':float(max(abs(residual))),
            'r2':float(1-np.sum(residual**2)/np.sum((x-x.mean())**2)) if np.std(x)>1e-15 else None}
        s['design_options_NOT_APPLIED_mm']={
            'q0_bias_plus_median_centered_p99':float(bias+np.percentile(center_dev,99)),
            'q0_bias_plus_3_robust_sigma_MAD':float(bias+3*1.4826*s['MAD']),
            'observed_max_worsening_from_this_q0':s['maximum_baseline_worsening'],
            'observed_peak_to_peak_for_arbitrary_baseline_in_this_window':s['peak_to_peak'],
            'hysteresis_release_band_median_centered_p95':float(np.percentile(center_dev,95))}
        pair_stats.append(s)
    joint_stats=[]
    for j,name in enumerate(NAMES):
        joint_stats.append({'joint':name,'peak_to_peak_rad':float(np.ptp(q[:,j])),
                            'std_rad':float(np.std(q[:,j])),
                            'p99_deviation_from_q0_rad':float(np.percentile(abs(delta[:,j]),99)),
                            'max_abs_deviation_from_q0_rad':float(max(abs(delta[:,j])))})
    live_times=np.array([r['monotonic_s'] for r in live]);live_intervals=np.diff(live_times)
    live_clr=np.array([[r['clearance_m'][key]*1000 for key in keys] for r in live])
    timing={**info,'lowstate_reception_rate_hz':(len(captured)-1)/(times[-1]-times[0]),
            'live_FK_calculation_rate_hz':len(live)/info['duration_s'],
            'live_collision_evaluation_rate_hz':len(live)/info['duration_s'],
            'lowstate_reception_interval_s':summary(intervals),
            'monitor_sample_interval_s':summary(live_intervals),
            'live_FK_processing_s':summary([r['fk_s'] for r in live]),
            'live_collision_processing_s':summary([r['collision_s'] for r in live]),
            'live_ownership_processing_s':summary([r['ownership_s'] for r in live]),
            'live_total_processing_s':summary([r['cycle_s'] for r in live]),
            'live_lowstate_age_at_FK_s':summary([r['evaluation_age_s'] for r in live]),
            'live_processing_over_20ms_fraction':float(np.mean([r['cycle_s']>.02 for r in live])),
            'offline_replay_duration_s':replay_duration,'offline_replay_rate_hz':len(captured)/replay_duration,
            'lowstate_tick_increment':summary(np.diff([r[2] for r in captured])),
            'live_step_vs_monitor_interval_correlation':{
                key:corr(abs(np.diff(live_clr[:,k])),live_intervals) for k,key in enumerate(keys)}}
    valid=info['duration_s']>=60 and not info['errors'] and np.max(np.ptp(q,axis=0))<=.025
    label=classify(pair_stats) if valid else 'UNCLASSIFIED'
    worst=max(pair_stats,key=lambda s:s['maximum_baseline_worsening'])
    affected=[s for s in pair_stats if s['previous_failing_pair']]
    reproduced=any(s['maximum_baseline_worsening']>=.012 for s in affected)
    interpretation='YES (likely monitoring false positive, not proof)' if label=='NOISE-A' and reproduced else 'UNKNOWN'
    report={'schema':'motiondecode-test.hold-baseline-noise.v1','measurement_valid':bool(valid),
        'commands_sent':'NONE','application_command_count':0,'command_publishers_created':0,
        'ownership_acquired':False,'current_tolerance_mm':CURRENT_TOLERANCE_MM,'threshold_modified':False,
        'classification':label,'classification_rule':'A: any pair worsens >0.001 mm in >=5% of samples; B: any rarer exceedance; C: none. Descriptive rule only.',
        'observation':info,'pair_statistics':pair_stats,'joint_statistics':joint_stats,
        'max_joint_deviation_rad':float(max(abs(delta).ravel())),
        'p99_of_per_sample_max_joint_deviation_rad':float(np.percentile(np.max(abs(delta),axis=1),99)),
        'worst_pair':worst['pair'],'maximum_natural_baseline_worsening_mm':worst['maximum_baseline_worsening'],
        'p95_per_sample_worst_absolute_clearance_deviation_mm':float(np.percentile(np.max(abs(v-init),axis=1),95)),
        'p99_per_sample_worst_absolute_clearance_deviation_mm':float(np.percentile(np.max(abs(v-init),axis=1),99)),
        'new_penetration_pairs':sorted(new_penetrations),'new_clearance_deficit_pairs':sorted(new_deficits),
        'previous_stop_likely_monitoring_false_positive':interpretation,
        'hold_retest_ready':False,
        'hold_retest_reason':'No threshold changed. Pair-specific statistical monitor and 20ms timing must be reviewed and verified before positive-weight HOLD.',
        'limitations':['Receive-only joint/FK variation includes encoder noise and small real posture variation; these cannot be separated without independent sensing.',
                      'Finite quiet observation does not characterize positive-weight control, arbitrary postures, hands, objects, or guarantee a future bound.',
                      'Same mesh model and base convention as runtime: IMU attitude is recorded but root attitude is not applied in that self-clearance model.'],
        'source_hashes':{name:digest(ROOT/'scripts'/name) for name in ['scene.py','run_real_reaction.py','measure_hold_baseline_noise.py']}}
    write_json(folder/'baseline_noise_stats.json',report);write_json(folder/'monitor_timing.json',timing)
    write_report(folder,report,timing)
    print(json.dumps({k:report[k] for k in ['classification','worst_pair','maximum_natural_baseline_worsening_mm',
          'max_joint_deviation_rad','previous_stop_likely_monitoring_false_positive','hold_retest_ready']},ensure_ascii=False),flush=True)


def write_report(folder,r,t):
    lines=['# G1 HOLD baseline noise characterization','', '## Safety','',
           'Commands sent to G1: **NONE**. Publisher/DataWriter construction is denied in this process.',
           'No weight, LowCmd, joint/motion command, ownership acquisition, service, Navigation or SLAM operation.',
           'External arm writers: 0 throughout accepted live samples. LowState fresh; Arm Action IDLE; stationary preflight PASS.',
           '', '## Observation','',f"Duration: {t['duration_s']:.3f} s; LowState samples: {t['lowstate_samples']}; received rate: {t['lowstate_reception_rate_hz']:.3f} Hz.",
           f"Live FK/collision evaluations: {t['live_monitor_samples']}; rate: {t['live_collision_evaluation_rate_hz']:.3f} Hz.",
           'All captured LowStates were also evaluated offline with the same FK/pair function. Clearance CSV is reception-time data replay; live ages/rates are in monitor_timing.json and live_monitor_samples.json.',
           '', '| Timing, milliseconds | mean | median | min | max | p95 | p99 |',
           '|---|---:|---:|---:|---:|---:|---:|']
    for key in ['lowstate_reception_interval_s','monitor_sample_interval_s','live_FK_processing_s','live_collision_processing_s','live_ownership_processing_s','live_total_processing_s','live_lowstate_age_at_FK_s']:
        a=t[key];lines.append('| '+key+' | '+' | '.join(f'{a[c]*1000:.3f}' for c in ['mean','median','min','max','p95','p99'])+' |')
    lines+=['','## Joint noise','',f"Max q deviation: {r['max_joint_deviation_rad']:.9f} rad. p99 of per-sample maximum: {r['p99_of_per_sample_max_joint_deviation_rad']:.9f} rad.",
            'Per-joint peak-to-peak, std, p99 and correlations are in baseline_noise_stats.json.',
            '', '## Clearance noise','',f"Current tolerance: **{CURRENT_TOLERANCE_MM} mm** (unchanged). Worst pair: `{r['worst_pair']}`.",
            '', '| Pair (* = previous stop) | initial mm | min mm | p-p mm | std mm | MAD mm | max worsening mm | p95 abs mm | p99 abs mm | exceed % |',
            '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for s in r['pair_statistics']:
        vals=[s[k] for k in ['initial','minimum','peak_to_peak','std','MAD','maximum_baseline_worsening','p95_absolute_deviation_from_q0','p99_absolute_deviation_from_q0']]
        lines.append('| '+s['pair']+(' *' if s['previous_failing_pair'] else '')+' | '+' | '.join(f'{v:.6f}' for v in vals)+f" | {s['fraction_worsening_over_current_tolerance']*100:.2f} |")
    lines+=['','## Classification','',f"**{r['classification']}**. {r['classification_rule']}",'',
            '## Interpretation','',f"Was the previous STAGE 1 stop likely a monitoring false positive? **{r['previous_stop_likely_monitoring_false_positive']}**.",
            'Previous attempt never sent positive weight, so it did not test an engaged HOLD. Compare each previous pair’s same-sample variation, interval correlation and initial baseline bias below.', '']
    for s in r['pair_statistics']:
        if not s['previous_failing_pair']:continue
        lines+=[f"- {s['pair']}: max consecutive step {s['sample_to_sample_abs_step_mm']['max']:.6f} mm; reception-step correlation {s['step_vs_reception_interval_correlation']}; initial−median {s['initial_minus_median_mm']:.6f} mm; same-FK Jacobian residual RMSE {s['same_FK_local_sensitivity_explanation']['rmse_mm']:.9f} mm."]
    lines+=['','## Recommended baseline-worsening tolerance design','',
            '**Do NOT modify it yet.** Hard safety stays independent: new penetration/pair, joint-limit violation, and substantial absolute clearance loss require separate stop conditions. A noise band must never authorize new penetration.',
            'Measured candidate bands below are proposals only, not safety guarantees. Use fresh per-pair receive-only distributions; keep a fixed reference during motion so a drifting baseline cannot hide worsening.',
            'p99 is efficient but permits about 1% tail exceedance in its own calibration distribution. Three robust sigmas from MAD resist outliers but require tail validation. Observed maxima/p-p cover this finite observation only and need independent repeat validation.',
            'Hysteresis could trip outside the reviewed stop band and clear only inside a smaller measured p95 band. Sustained worsening/cumulative drift and a hard absolute guard must remain independent. Do not choose persistence duration without checking temporal clustering.',
            '', '| Pair | bias+p99 mm | bias+3×1.4826×MAD mm | observed max worsening mm | observed p-p mm | centered p95 clear band mm |',
            '|---|---:|---:|---:|---:|---:|']
    for s in r['pair_statistics']:
        lines.append('| '+s['pair']+' | '+' | '.join(f'{v:.6f}' for v in s['design_options_NOT_APPLIED_mm'].values())+' |')
    lines+=['','## HOLD RETEST READY','', '**NO**. '+r['hold_retest_reason'],
            'After those prerequisites: fresh q0 → receive-only baseline → all controlled targets = fresh measured q → reviewed very-low-weight stages → q0 HOLD → release. No 25% reaction yet.',
            '', '## Commands sent','', 'arm_sdk: NONE. Motion: NONE. Navigation: NONE. SLAM: NONE.',
            '', '## Limits','']+['- '+x for x in r['limitations']]
    (folder/'report.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--network-interface',required=True)
    p.add_argument('--discovery-peer',action='append',default=[],
                   help='Optional process-local DDS discovery peer; does not change OS networking')
    p.add_argument('--seconds',type=float,default=120.)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--internal-observation-json',type=Path,
                   help='Optional recent idle internal-topology observation; exact identity binding for receive-only only')
    args=p.parse_args()
    if not 60<=args.seconds<=120:p.error('Observation duration must be 60..120 seconds')
    folder=output_path(args.output_dir/'report.md').parent
    if any(folder.iterdir()):p.error('Use a new empty output directory')
    forbid_writers();reader=None
    try:
        profile=observed_internal_profile(args.internal_observation_json) if args.internal_observation_json else None
        if profile:
            write_json(folder/'receive_only_identity_binding.json',{
                'source_observation':str(args.internal_observation_json.resolve()),
                'source_sha256':digest(args.internal_observation_json),'profile':profile,
                'scope':'receive-only process only; real runner unchanged',
                'evidence_limit':'DDS self-advertised metadata, same evidentiary limit as prior internal classification'})
        reader=RecordingState(args.network_interface,profile,args.discovery_peer)
        print('RECEIVE ONLY preflight: 15 seconds',flush=True)
        snapshot=preflight(reader,folder,args.network_interface)
        captured,live,initial,keys,info=collect(reader,folder,snapshot,args.seconds)
        reader.close();reader=None
        print('DDS closed; offline all-reception FK/statistics begin',flush=True)
        analyze(folder,snapshot,captured,live,initial,keys,info)
    except (RuntimeError,ValueError,KeyboardInterrupt) as exc:
        blocked={'measurement_valid':False,'classification':'UNCLASSIFIED','reason':str(exc),
                 'commands_sent':'NONE','application_command_count':0,'command_publishers_created':0,
                 'current_tolerance_mm':CURRENT_TOLERANCE_MM,'threshold_modified':False,'hold_retest_ready':False}
        write_json(folder/'baseline_noise_stats.json',blocked)
        write_json(folder/'monitor_timing.json',{'status':'BLOCKED','reason':str(exc)})
        if not (folder/'fresh_q0.json').exists():write_json(folder/'fresh_q0.json',{'status':'UNAVAILABLE','reason':str(exc)})
        for name in ('lowstate_samples.csv','clearance_samples.csv'):
            if not (folder/name).exists():(folder/name).write_text('status\n')
        (folder/'report.md').write_text('# G1 HOLD baseline noise characterization\n\nCommands sent to G1: **NONE**\n\nMeasurement **BLOCKED**: '+str(exc)+'\n\nNOISE-A/B/C: **UNCLASSIFIED** (no valid 60-second measurement).\n\nHOLD RETEST READY: **NO**.\n\nNo measured threshold proposal can be made. Existing 0.001 mm tolerance is unchanged.\n')
        print('BLOCKED: '+str(exc),flush=True);return 2
    finally:
        if reader:reader.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
