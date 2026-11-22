#!/usr/bin/env python3
"""Receive-only wireless LowState, watchdog, fast-loop, and geometry validation."""
import argparse
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from audit_geometry import audit as audit_geometry
from common import output_path, write_json
from inspect_arm_sdk_publishers import interface_ipv4, local_process_scan
from measure_hold_baseline_noise import (CURRENT_TOLERANCE_MM, RecordingState,
                                         forbid_writers, observed_internal_profile,
                                         series_stats, summary)
from robot_transport import state_summary, validate_state
from safety_monitor import OwnershipWatchdog, assess_runtime_safety
from scene import make_scene, clearance_pairs, measured_pose_clearances

MINIMUM_M=.015


def _command(command,json_output=False):
    try:
        result=subprocess.run(command,check=True,capture_output=True,text=True,timeout=5)
        return json.loads(result.stdout) if json_output else result.stdout
    except (OSError,subprocess.SubprocessError,json.JSONDecodeError) as exc:
        return {'error':f'{type(exc).__name__}: {exc}','command':command}


def network_state(interface,peers):
    addresses=_command(['ip','-j','-4','address','show'],True)
    routes=_command(['ip','-j','route','show'],True)
    targets=['192.168.123.161','192.168.123.164']+list(peers)
    target_routes={target:_command(['ip','-j','route','get',target],True) for target in targets}
    wireless=Path('/sys/class/net',interface,'wireless').exists()
    tcp=_command(['ss','-H','-tnp'])
    peer_connections=([line for line in tcp.splitlines() if any(peer in line for peer in peers)]
                      if isinstance(tcp,str) else tcp)
    return {'captured_local_time':time.strftime('%Y-%m-%d %H:%M:%S %z'),
            'active_interface_for_validation':interface,
            'local_ipv4':interface_ipv4(interface),'link_type':'wireless' if wireless else 'wired',
            'wireless':wireless,'dds_discovery_peers':list(peers),
            'route_to_G1_peer':{peer:target_routes[peer] for peer in peers},
            'route_to_192.168.123.161':target_routes['192.168.123.161'],
            'route_to_192.168.123.164':target_routes['192.168.123.164'],
            'all_ipv4_addresses':addresses,'all_routes':routes,
            'wireless_device_state':_command(['iw','dev']),
            'wireless_station_state':_command(['iw','dev',interface,'station','dump']) if wireless else None,
            'preexisting_tcp_connections_to_peers':peer_connections,
            'network_settings_changed':False,
            'read_only_commands':['ip -j -4 address show','ip -j route show','ip -j route get <target>',
                                  'iw dev','iw dev <interface> station dump']}


def timing(values):
    return summary(values) if values else None


def watchdog_report(watchdog,cached,duration_s):
    rows=watchdog.timing_records();intervals=[x['check_interval_s'] for x in rows
                                              if x['check_interval_s'] is not None]
    durations=[x['check_duration_s'] for x in rows]
    successes=sum(x['successful'] for x in rows)
    current=cached.get('current') or {}
    external_count=len(current.get('external_arm_sdk_writers',[]))
    armsdk_count=len(current.get('armsdk_writers',[]))
    return {'architecture':'separate worker; thread-safe cached state; conflict latched until explicit new session',
            'configured_period_s':watchdog.period_s,'observation_duration_s':duration_s,
            'checks':len(rows),'successful_checks':successes,
            'check_rate_hz':len(rows)/duration_s if duration_s else None,
            'check_interval_s':timing(intervals),'check_duration_s':timing(durations),
            'latest_cached_state':cached,'latched_fault':cached['latched_fault'],
            'observed_external_arm_sdk_writer_count':external_count,
            'observed_armsdk_writer_count':armsdk_count,
            'external_writer_detection':'0 observed on this interface; no writer was injected, and absent robot DDS prevents an end-to-end detection claim' if not external_count and not armsdk_count else 'writer observed and latched',
            'participant_writer_change_events':cached['participant_writer_change_events'],
            'discovery_update_latency_s':None,
            'discovery_update_latency_reason':'No remote participant/writer change event was intentionally generated.',
            'timeout_applied':False,
            'timeout_note':'Distribution is recorded for later review; no ownership freshness threshold selected.'}


def observe_lowstate(reader,watchdog,seconds):
    with reader.capture_lock:
        reader.captured=[];reader.capture_tick=None;reader.capture=True
    started=time.monotonic();ages=[];ownership_ages=[];errors=[]
    error_count=0;first_error=last_error=None
    while time.monotonic()-started<seconds:
        try:
            reader.get(max_age=.5)
            with reader.lock:ages.append(time.monotonic()-reader.latest[0])
        except RuntimeError as exc:
            error={'at_s':time.monotonic()-started,'error':str(exc)}
            error_count+=1;first_error=first_error or error;last_error=error
            if len(errors)<10:errors.append(error)
        ownership_age=watchdog.snapshot()['last_successful_check_age_s']
        if ownership_age is not None:ownership_ages.append(ownership_age)
        time.sleep(.01)
    ended=time.monotonic();cached=watchdog.snapshot()
    with reader.capture_lock:
        reader.capture=False;captured=list(reader.captured)
    intervals=np.diff([x[0] for x in captured]) if len(captured)>1 else []
    gaps=np.asarray(intervals)
    result={'status':'PASS' if len(captured)>1 and not error_count else 'FAIL',
            'duration_s':ended-started,'samples':len(captured),
            'rate_hz':(len(captured)-1)/(captured[-1][0]-captured[0][0]) if len(captured)>1 else 0.,
            'sample_interval_s':timing(intervals),'lowstate_age_s':timing(ages),
            'ownership_cached_state_age_s':timing(ownership_ages),
            'packet_sample_gaps_over_10ms':int(np.sum(gaps>.010)) if len(gaps) else 0,
            'packet_sample_gaps_over_100ms':int(np.sum(gaps>.100)) if len(gaps) else 0,
            'freshness_error_count':error_count,'first_freshness_error':first_error,
            'last_freshness_error':last_error,'freshness_error_examples':errors,
            'latest_ownership_cache':cached}
    return result,captured,cached


def calibration_stats(keys,initial,values):
    values=np.asarray(values);records=[]
    for index,key in enumerate(keys):
        x=values[:,index]*1000;initial_mm=initial[key]*1000
        stats=series_stats(x,initial_mm);median=float(np.median(x));center=abs(x-median)
        stats.update(pair=key,units='mm',design_candidates_NOT_APPLIED_mm={
            'T99_q0_bias_plus_centered_p99':float(abs(initial_mm-median)+np.percentile(center,99)),
            'MAD_q0_bias_plus_3_robust_sigma':float(abs(initial_mm-median)+3*1.4826*np.median(center)),
            'observed_maximum_worsening':float(max(0,initial_mm-x.min())),
            'hysteresis_centered_p95_release_band':float(np.percentile(center,95))})
        records.append(stats)
    return records


def fast_loop(reader,watchdog,snapshot,seconds):
    q0=np.asarray(snapshot['all_q']);model,data=make_scene();pairs=clearance_pairs(model)
    base=model.qpos0.copy();initial,_=measured_pose_clearances(model,data,base,pairs,q0)
    keys=sorted(initial);rows=[];values=[];errors=[];hard_events=[]
    started=time.monotonic();next_at=started
    while time.monotonic()-started<seconds:
        time.sleep(max(0,next_at-time.monotonic()));cycle_start=time.monotonic()
        try:
            state=reader.get(max_age=.15);validate_state(state)
            q=np.array([x.q for x in state.motor_state[:29]])
            with reader.lock:age=time.monotonic()-reader.latest[0]
            fk_start=time.perf_counter();distances,cost=measured_pose_clearances(
                model,data,base,pairs,q);completed=time.monotonic()
            cached=watchdog.snapshot()
            assessment=assess_runtime_safety(q,q0,q0,distances,initial,age,cached,
                                             MINIMUM_M,CURRENT_TOLERANCE_MM/1000)
            if not assessment['hard_safety']['passed']:
                hard_events.append({'at_s':completed-started,
                                    'hard_safety':assessment['hard_safety']})
            rows.append({'completed_monotonic_s':completed,'interval_anchor_s':cycle_start,
                         'lowstate_age_at_FK_start_s':age,'fk_s':cost['fk_s'],
                         'collision_s':cost['collision_s'],
                         'fast_loop_duration_s':completed-cycle_start,
                         'ownership_last_success_age_s':cached['last_successful_check_age_s']})
            values.append([distances[key] for key in keys])
        except (RuntimeError,ValueError) as exc:
            errors.append({'at_s':time.monotonic()-started,'error':str(exc)})
        next_at+=.01
        if next_at<time.monotonic()-.1:next_at=time.monotonic()
    duration=time.monotonic()-started
    intervals=np.diff([x['interval_anchor_s'] for x in rows]) if len(rows)>1 else []
    timing_report={'status':'PASS' if rows and not errors and not hard_events else 'FAIL',
        'duration_s':duration,'evaluations':len(rows),'evaluation_rate_hz':len(rows)/duration,
        'target_period_s':.01,'target_period_is_performance_schedule_not_acceptance_threshold':True,
        'fast_safety_loop_interval_s':timing(intervals),
        'FK_duration_s':timing([x['fk_s'] for x in rows]),
        'collision_duration_s':timing([x['collision_s'] for x in rows]),
        'fast_safety_loop_duration_s':timing([x['fast_loop_duration_s'] for x in rows]),
        'LowState_age_at_FK_start_s':timing([x['lowstate_age_at_FK_start_s'] for x in rows]),
        'ownership_state_freshness_s':timing([x['ownership_last_success_age_s'] for x in rows
                                              if x['ownership_last_success_age_s'] is not None]),
        'hard_safety_events':hard_events,'errors':errors,
        'baseline_relative_threshold_applied':False}
    if values:
        elapsed=np.asarray([x['interval_anchor_s'] for x in rows])-rows[0]['interval_anchor_s']
        calibration=np.asarray(values)[elapsed<=30]
        noise={'status':'MEASURED','calibration_window_s':min(30.,duration),
               'samples':len(calibration),'current_tolerance_mm':CURRENT_TOLERANCE_MM,
               'threshold_modified':False,'threshold_applied':False,
               'pair_statistics':calibration_stats(keys,initial,calibration),
               'full_observation_pair_statistics':calibration_stats(keys,initial,values)}
    else:noise={'status':'BLOCKED','reason':'No successful fast-loop evaluation',
                'current_tolerance_mm':CURRENT_TOLERANCE_MM,'threshold_modified':False}
    return timing_report,noise


def blocked_outputs(folder,reason,network,lowstate,watchdog,offline_q0):
    write_json(folder/'fresh_q0.json',{'status':'UNAVAILABLE','reason':reason,
                                      'receive_only':True,'command_publishers_created':0})
    write_json(folder/'fast_safety_timing.json',{'status':'BLOCKED','reason':reason})
    write_json(folder/'baseline_noise_stats.json',{'status':'BLOCKED','reason':reason,
               'current_tolerance_mm':CURRENT_TOLERANCE_MM,'threshold_modified':False})
    geometry=None
    if offline_q0:
        geometry=audit_geometry(offline_q0,folder/'geometry_audit',preview=False,current_fresh=False)
    elif not (folder/'geometry_audit.json').exists():
        write_json(folder/'geometry_audit.json',{'status':'BLOCKED','reason':'No fresh q0'})
    write_report(folder,network,lowstate,watchdog,None,geometry,reason)


def write_report(folder,network,lowstate,watchdog,fast,geometry,blocked_reason=None):
    interface=network['active_interface_for_validation'];local=', '.join(network['local_ipv4']) or 'NONE'
    lines=['# G1 wireless safety-monitor validation','','## Network','',
        f'interface: **{interface}**',f'local IP: **{local}**',
        f"route to G1: `{json.dumps(network['route_to_G1_peer'],ensure_ascii=False)}`",
        f"wireless: **{'YES' if network['wireless'] else 'NO'}**",'Network settings changed: **NO**',
        '','## LowState over Wi-Fi','',
        f"duration: {lowstate['duration_s']:.3f} s",f"samples: {lowstate['samples']}",
        f"rate: {lowstate['rate_hz']:.3f} Hz",
        f"data freshness: **{lowstate['status']}**"]
    if lowstate.get('sample_interval_s'):
        value=lowstate['sample_interval_s'];lines += [
            f"median interval: {value['median']*1000:.3f} ms",
            f"p95: {value['p95']*1000:.3f} ms",f"p99: {value['p99']*1000:.3f} ms",
            f"max: {value['max']*1000:.3f} ms"]
    else:lines += ['sample interval / LowState age: **UNAVAILABLE (0 samples)**']
    lines += ['','## Geometry audit','']
    if geometry:
        for index,pair in enumerate(geometry['pairs'],1):
            first=pair['geometry'][pair['pair'][0]]['urdf']
            second=pair['geometry'][pair['pair'][1]]['urdf']
            lines += [f"Pair {index}: `{' / '.join(pair['pair'])}`",
                      f"signed distance: {pair['signed_distance_mm']:.6f} mm",
                      f"closest points: `{pair['closest_point_on_body_a_m']}` / `{pair['closest_point_on_body_b_m']}`",
                      f"mesh sources: `{first['visual']['mesh']}` / `{second['visual']['mesh']}`",
                      f"URDF collision meshes: `{first['collision']['mesh']}` / `{second['collision']['mesh']}`",
                      f"visual interpretation: {pair['visual_interpretation']}",
                      f"classification: **{pair['classification']}**",'']
        lines += [f"Overall geometry classification: **{geometry['overall_classification']}**",
                  'No pair was added to an allowlist. The rubber-hand input has no URDF collision element; runtime intentionally evaluates its visual mesh as a conservative convex proxy. This makes a model artifact plausible but does not prove GEO-B.']
    else:lines += ['Geometry audit: **BLOCKED — no fresh q0**']
    lines += ['','## Ownership watchdog','',watchdog['architecture'],
              f"check rate: {watchdog['check_rate_hz']:.3f} Hz",
              f"latched fault: **{watchdog['latched_fault']}**",
              'freshness timeout applied: **NO**']
    if watchdog.get('check_interval_s'):
        interval=watchdog['check_interval_s'];duration=watchdog['check_duration_s']
        lines += [f"check interval p95 / p99: {interval['p95']*1000:.3f} / {interval['p99']*1000:.3f} ms",
                  f"check duration p95 / p99: {duration['p95']*1000:.3f} / {duration['p99']*1000:.3f} ms"]
    if lowstate.get('ownership_cached_state_age_s'):
        age=lowstate['ownership_cached_state_age_s']
        lines += [f"cached-state age p95 / p99: {age['p95']*1000:.3f} / {age['p99']*1000:.3f} ms"]
    lines += [f"external writer detection: {watchdog['external_writer_detection']}"]
    lines += ['','## Fast safety loop','']
    if fast:
        interval=fast['fast_safety_loop_interval_s'];age=fast['LowState_age_at_FK_start_s']
        lines += [f"rate: {fast['evaluation_rate_hz']:.3f} Hz",
                  f"median interval: {interval['median']*1000:.3f} ms",
                  f"p95: {interval['p95']*1000:.3f} ms",
                  f"p99: {interval['p99']*1000:.3f} ms",f"max: {interval['max']*1000:.3f} ms",
                  f"LowState age p95: {age['p95']*1000:.3f} ms",
                  f"LowState age p99: {age['p99']*1000:.3f} ms"]
    else:lines += [f"**BLOCKED**: {blocked_reason}"]
    lines += ['','Previous: 29.1 Hz / p99 interval ~104 ms.',
              'Improvement: **NOT EVALUATED — wireless preflight did not supply LowState.**',
              '', '## Baseline monitor','',
              'Hard safety separated: **YES**','Baseline-relative monitor separated: **YES**',
              'Threshold modified: **NO**','Calibration candidates are recorded only; none is applied.',
              'New penetration observed during receive-only: **NOT EVALUATED (no LowState)**',
              '','## HOLD RETEST READY','','**NO**',
              f"Reason: {blocked_reason or 'Threshold and geometry review remain required.'}"+
              ('; existing penetration remains GEO-E' if geometry and geometry.get('overall_classification')=='GEO-E' else ''),
              '','## Commands sent to G1','',
              'arm_sdk: **NONE**','Motion: **NONE**','Navigation: **NONE**','SLAM: **NONE**',
              'LiDAR / service operation: **NONE**',
              'Network configuration command: **NONE**']
    (folder/'report.md').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--network-interface',required=True)
    parser.add_argument('--discovery-peer',action='append',default=[])
    parser.add_argument('--seconds',type=float,default=120.)
    parser.add_argument('--lowstate-observation-seconds',type=float,default=30.)
    parser.add_argument('--watchdog-period',type=float,default=.1)
    parser.add_argument('--internal-observation-json',type=Path)
    parser.add_argument('--offline-q0',type=Path,
                        help='Fallback for clearly labeled offline geometry audit only')
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    if not 120<=args.seconds<=180:parser.error('--seconds must be 120..180')
    if not 30<=args.lowstate_observation_seconds<=60:
        parser.error('--lowstate-observation-seconds must be 30..60')
    folder=output_path(args.output_dir/'report.md').parent
    if any(folder.iterdir()):parser.error('Use a new empty output directory')
    network=network_state(args.network_interface,args.discovery_peer);write_json(folder/'network_state.json',network)
    forbid_writers();reader=watchdog=None
    try:
        if args.internal_observation_json and len(args.discovery_peer)!=1:
            raise ValueError('Exact wireless identity binding requires one discovery peer/IP')
        profile=(observed_internal_profile(args.internal_observation_json,args.discovery_peer[0])
                 if args.internal_observation_json else None)
        reader=RecordingState(args.network_interface,profile,args.discovery_peer)
        time.sleep(1.)  # let receive-only DDS discovery settle before latchable checks
        watchdog=OwnershipWatchdog(lambda:reader.ownership_snapshot(),args.watchdog_period).start()
        lowstate,captured,cached=observe_lowstate(reader,watchdog,args.lowstate_observation_seconds)
        watchdog.stop();watchdog_summary=watchdog_report(watchdog,cached,lowstate['duration_s'])
        write_json(folder/'lowstate_timing.json',lowstate)
        write_json(folder/'ownership_watchdog_timing.json',watchdog_summary)
        reasons=[]
        if lowstate['status']!='PASS':reasons.append('wireless LowState absent or stale')
        if not cached['ownership_safe']:reasons.append('ownership/Arm Action not safely established')
        scan=local_process_scan(interface_ipv4(args.network_interface))
        if scan['candidate_processes_excluding_diagnostic']:
            reasons.append('local arm/DDS controller candidate exists')
        try:reader.require_stationary()
        except RuntimeError as exc:reasons.append(str(exc))
        if reasons:
            reason='; '.join(reasons);blocked_outputs(folder,reason,network,lowstate,
                                                  watchdog_summary,args.offline_q0)
            print('BLOCKED: '+reason,flush=True);return 2
        state=reader.get();validate_state(state,initial=True);snapshot=state_summary(state)
        snapshot.update(timestamp=time.strftime('%Y-%m-%d %H:%M:%S %z'),receive_only=True,
                        interface=args.network_interface,discovery_peers=args.discovery_peer,
                        command_publishers_created=0)
        write_json(folder/'fresh_q0.json',snapshot)
        geometry=audit_geometry(folder/'fresh_q0.json',folder/'geometry_audit',False,True)
        watchdog.reset_session();watchdog.start();watchdog.wait_first()
        fast,noise=fast_loop(reader,watchdog,snapshot,args.seconds);cached=watchdog.snapshot()
        watchdog.stop();watchdog_summary=watchdog_report(watchdog,cached,args.seconds)
        write_json(folder/'fast_safety_timing.json',fast)
        write_json(folder/'ownership_watchdog_timing.json',watchdog_summary)
        write_json(folder/'baseline_noise_stats.json',noise)
        reason=None
        if fast['status']!='PASS':reason='Fast safety loop or hard safety validation failed'
        if geometry['overall_classification']=='GEO-E':
            reason=(reason+'; ' if reason else '')+'existing penetration remains GEO-E'
        write_report(folder,network,lowstate,watchdog_summary,fast,geometry,reason)
        return 0
    finally:
        if watchdog and watchdog.thread is not None and watchdog.thread.is_alive():watchdog.stop()
        if reader:reader.close()


if __name__=='__main__':raise SystemExit(main())
