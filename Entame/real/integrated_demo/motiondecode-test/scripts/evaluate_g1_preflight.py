#!/usr/bin/env python3
"""Combine a receive-only ownership report with an offline q0 path preflight."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from common import ROOT, read_arms, write_json
from play_g1_arms import ACTIVE, prepare, validate_clip


def check(status, evidence):
    return {"status":"PASS" if status else "FAIL", "evidence":evidence}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion",type=Path)
    parser.add_argument("--state-json",type=Path,required=True)
    parser.add_argument("--ownership-json",type=Path,required=True)
    parser.add_argument("--speed",type=float,default=.4)
    parser.add_argument("--blend",type=float,default=2.)
    parser.add_argument("--json",type=Path)
    args=parser.parse_args()
    if args.json is None:
        args.json=ROOT/'output'/f"combined_preflight_{time.strftime('%Y%m%d_%H%M%S')}.json"

    ownership=json.loads(args.ownership_json.read_text())
    state=json.loads(args.state_json.read_text())
    if ownership.get('schema')!='motiondecode-test.arm-control-ownership.v2':
        raise SystemExit('Unsupported ownership report schema')
    if state.get('source_ownership_report')!=str(args.ownership_json.resolve()):
        raise SystemExit('State and ownership report were not captured in the same observation')
    t,q,meta=read_arms(args.motion)
    velocity,acceleration=validate_clip(t,q,meta,args.speed,args.blend)
    _,_,path=prepare(t,q,state,args.blend,args.speed)

    arm_sdk=ownership['topics']['rt/arm_sdk']
    armsdk=ownership['topics']['rt/armsdk']
    action=ownership['topics']['rt/arm/action/state']
    collision=path['clearance']
    phases=collision['phases']
    baseline_rules=collision['baseline_rules']
    checks={
        'lowstate_healthy':check(ownership['lowstate']['status']=='PASS',ownership['lowstate']['status']),
        'g1_stationary':check(ownership['lowstate']['stationary']=='PASS',ownership['lowstate']['stationary']),
        'known_robot_internal_arm_service_only':check(
            arm_sdk['expected_robot_internal_count']==1 and arm_sdk['external_writer_count']==0,
            {'expected':arm_sdk['expected_robot_internal_count'],'external':arm_sdk['external_writer_count']}),
        'robot_internal_arm_command_idle':check(
            arm_sdk['expected_robot_internal_sample_count']==0,
            {'samples':arm_sdk['expected_robot_internal_sample_count'],
             'rate_hz':arm_sdk['total_sample_rate_hz']}),
        'no_unknown_external_rt_arm_sdk_writer':check(
            arm_sdk['external_writer_count']==0,arm_sdk['external_writer_count']),
        'no_unknown_external_rt_armsdk_writer':check(
            armsdk['unknown_writer_count']==0,armsdk['unknown_writer_count']),
        'arm_action_idle':check(action['status_during_observation']=='IDLE',
                                {'status':action['status_during_observation'],
                                 'samples':action['sample_count'],'rate_hz':action['sample_rate_hz']}),
        'no_local_arm_controller_process':check(
            not ownership['local_os_scan']['candidate_processes_excluding_diagnostic'],
            ownership['local_os_scan']['candidate_processes_excluding_diagnostic']),
        'joint_limits':check(path['joint_limits_passed'],path['joint_limits_passed']),
        'velocity':check(float(np.max(np.abs(velocity)))<=.25,
                         {'max_rad_s':float(np.max(np.abs(velocity))),'limit_rad_s':.25}),
        'acceleration':check(float(np.max(np.abs(acceleration)))<=1.,
                             {'max_rad_s2':float(np.max(np.abs(acceleration))),'limit_rad_s2':1.}),
        'transition_distance':check(path['transition_distance_passed'],
                                    {'max_rad':path['max_current_displacement_rad'],
                                     'limit_rad':path['transition_distance_limit_rad']}),
        'baseline_collision_does_not_worsen':check(
            baseline_rules['no_baseline_pair_worsened'],baseline_rules),
        'baseline_clearance_increases_at_motion_onset':check(
            baseline_rules['moving_baseline_clearance_increased_at_onset'],
            baseline_rules['onset_measurements']),
        'no_new_collision_pair':check(not collision['new_collision_pairs'],
                                      collision['new_collision_pairs']),
        'selected_clip_collision':check(
            path['collision']['nominal_selected_clip']['passed'],
            path['collision']['nominal_selected_clip']),
        'transition_collision':check(phases['entry']['passed'],phases['entry']),
        'actual_right_only_clip_collision':check(phases['clip']['passed'],phases['clip']),
        'return_collision':check(phases['return']['passed'],phases['return']),
        'right_arm_only':check(path['right_arm_only'],{'active_indices':ACTIVE}),
    }
    failures=[name for name,value in checks.items() if value['status']!='PASS']
    report={
        'schema':'motiondecode-test.combined-offline-preflight.v1',
        'checked_local_time':time.strftime('%Y-%m-%d %H:%M:%S %z'),
        'receive_only_and_offline':True,'command_publishers_created':0,
        'motion':str(args.motion),'state_json':str(args.state_json),
        'ownership_json':str(args.ownership_json),'speed':args.speed,
        'checks':checks,'failures':failures,
        'collision':collision,'trajectory':path,
        'real_robot_test':'READY' if not failures else 'BLOCKED',
    }
    write_json(args.json,report)
    print('\n=== COMBINED G1 PREFLIGHT ===\n')
    for name,value in checks.items():print(f"[{value['status']}] {name}")
    print('\nREAL ROBOT TEST '+report['real_robot_test'])
    if failures:print('Blocking checks: '+', '.join(failures))
    print(f'Report: {args.json}')
    return 0 if not failures else 2


if __name__=='__main__':raise SystemExit(main())
