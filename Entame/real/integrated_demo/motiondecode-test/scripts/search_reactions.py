#!/usr/bin/env python3
"""Receive-only q0-to-MotionDecode reaction search with staged offline checks."""
import argparse
import csv
import heapq
import json
from pathlib import Path
import time

import numpy as np

from common import (ROOT, ARMS, ARM_INDICES, ARM_NAMES, NAMES, digest, read_motion,
                    retime_arm_window, validate_arm_pose, write_json)
from play_g1_arms import ACTIVE, INACTIVE, MAX_ACCEL, MAX_SPEED, transition, transition_duration
from scene import (check_baseline_contact_path, clearance_pairs, make_scene,
                   pair_clearances, set_arms)

START_POSE_MAX_DELTA_RAD=1.0
STRICT_READY_MAX_DELTA_RAD=0.8
DISPLAY_ONLY_DELTA_TIERS_RAD=(0.9,1.0)
WINDOW_LENGTHS_FRAMES=(61,91,121)
AUTHORED_DURATIONS_S=(1.0,1.5,2.0,2.5,3.0)
SMOOTHING_WINDOW_FRAMES=11
OUTPUT_SAMPLE_HZ=50
MIN_REACTION_RANGE_RAD=0.12
MIN_SHOULDER_ELBOW_RANGE_RAD=0.08
EXCESSIVE_MOTION_PENALTY_START_RAD=0.45
CLIP_RANGE_LIMIT_RAD=0.50
SOURCE_MAX_STEP_RAD=0.15
COLLISION_CLEARANCE_M=0.015
PER_MOTION_DETAILED=3
PRE_POOL_PER_MOTION=240

# These are game-use classifications made in this project.  They are not
# additional labels supplied by the MotionDecode authors.
GAME_SEMANTICS={
    'DL_Attention_Pointing_And_Gaze_00001':('suspicious',1,'「あれ？」「何かいた？」'),
    'EESB_Observing_Surroundings_00001':('suspicious',1,'「あれ？」「周囲に何かいる？」'),
    'EESB_Attentive_Listening_00001':('suspicious',1,'「今、音がした？」'),
    'DL_Call_Response_Attention_Switch_00001':('notice',2,'「呼ばれた？」「何かいた？」'),
    'SII_Direction_Pointing_00001':('pointing',3,'「そこだ」'),
    'DL_Gesture_Communication_00001':('pointing',3,'「あそこを見て」'),
    'EESB_Confusion_00001':('confusion',4,'「気のせい？」「どこ？」'),
    'EESB_Frustration_00001':('confusion',4,'「おかしいな」'),
    'DL_Emotional_Expression_00001':('surprise',5,'「びっくりした」'),
    'EESB_Excitement_00001':('found / excitement',6,'「見つけた！」'),
}


def _joint_block(names,values):
    return {'joint_names':list(names),'values_rad':[float(x) for x in values]}


def _load_observation(state_path,ownership_path):
    state=json.loads(state_path.read_text())
    ownership=json.loads(ownership_path.read_text())
    if ownership.get('schema')!='motiondecode-test.arm-control-ownership.v2':
        raise ValueError('Unsupported ownership report schema')
    if ownership.get('ownership_preflight')!='PASS':
        raise ValueError('Ownership/LowState preflight is not PASS')
    if state.get('source_ownership_report')!=str(ownership_path.resolve()):
        raise ValueError('State and ownership report were not captured together')
    if not state.get('receive_only') or state.get('command_publishers_created')!=0:
        raise ValueError('State provenance is not receive-only')
    if ownership.get('command_publishers_created')!=0:
        raise ValueError('Ownership provenance reports a command publisher')
    if len(state.get('all_q',[]))!=29 or len(state.get('arm_q',[]))!=14:
        raise ValueError('Incomplete G1 joint state')
    return state,ownership


def _base_and_baseline(state):
    model,data=make_scene()
    base=model.qpos0.copy()
    for i,name in enumerate(NAMES):
        base[model.joint(name).qposadr[0]]=state['all_q'][i]
    q0=np.asarray(state['arm_q'],dtype=float)
    set_arms(model,data,q0,base)
    distances=pair_clearances(model,data,clearance_pairs(model))
    subthreshold=[{'pair':key.split(' | '),'distance_m':float(value),
                   'penetrating':bool(value<0)}
                  for key,value in sorted(distances.items()) if value<COLLISION_CLEARANCE_M]
    return base,distances,subthreshold


def _write_current_pose(path,state,ownership,state_path,ownership_path,baseline):
    checks={
        'g1_internal_participant_only':
            ownership['topics']['rt/arm_sdk']['expected_robot_internal_count']==1,
        'external_rt_arm_sdk_writers':ownership['topics']['rt/arm_sdk']['external_writer_count'],
        'rt_armsdk_writers':ownership['topics']['rt/armsdk']['unknown_writer_count'],
        'arm_action':ownership['topics']['rt/arm/action/state']['status_during_observation'],
        'lowstate':ownership['lowstate']['status'],
        'stationary':ownership['lowstate']['stationary'],
    }
    result={
        'schema':'motiondecode-test.current-pose-search.v1',
        'timestamp_local':state['checked_local_time'],
        'receive_only':True,'command_publishers_created':0,
        'source_state_json':str(state_path.resolve()),
        'source_ownership_report':str(ownership_path.resolve()),
        'ownership_result':ownership['ownership_preflight'],
        'ownership_checks':checks,
        'q0':{
            'right_arm':_joint_block(NAMES[22:29],state['all_q'][22:29]),
            'left_arm':_joint_block(NAMES[15:22],state['all_q'][15:22]),
            'waist':_joint_block(NAMES[12:15],state['all_q'][12:15]),
            'legs':_joint_block(NAMES[:12],state['all_q'][:12]),
            'all_joint_names':NAMES,'all_q_rad':[float(x) for x in state['all_q']],
        },
        'baseline_collision':{
            'required_clearance_m':COLLISION_CLEARANCE_M,
            'minimum_clearance_m':min(float(x) for x in baseline.values()),
            'subthreshold_pairs':[{'pair':key.split(' | '),'distance_m':float(value),
                                   'penetrating':bool(value<0)}
                                  for key,value in sorted(baseline.items())
                                  if value<COLLISION_CLEARANCE_M],
            'penetrating_pair_count':sum(value<0 for value in baseline.values()),
            'scope':'this measured q0 only; no permanent ignore list',
        },
        'robot_attitude':{
            'imu_rpy_rad':[float(x) for x in state['imu_rpy']],
            'gyro_rad_s':[float(x) for x in state['gyro']],
            'mode_machine':state['mode_machine'],'mode_pr':state['mode_pr'],'tick':state['tick'],
        },
    }
    write_json(path,result)
    return result


def _pose_distances(right,q0_right):
    delta=np.abs(right-q0_right)
    return {
        'max_joint_delta_rad':float(delta.max()),
        'l1_joint_delta_rad':float(delta.sum()),
        'l2_joint_delta_rad':float(np.linalg.norm(delta)),
        'shoulder_delta_rad':float(np.linalg.norm(delta[:3])),
        'elbow_delta_rad':float(delta[3]),
        'wrist_delta_rad':float(np.linalg.norm(delta[4:])),
    }


def _pre_score(dist,reaction,visible,wrist,entry_s,duration):
    excessive=max(0.0,reaction-EXCESSIVE_MOTION_PENALTY_START_RAD)*8
    wrist_only=max(0.0,wrist-visible-.02)*4
    estimated_speed=1.875*reaction/duration
    estimated_acceleration=5.774*reaction/(duration*duration)
    rate_penalty=max(0.0,estimated_speed-MAX_SPEED)*12+max(0.0,estimated_acceleration-MAX_ACCEL)*2
    duration_penalty=.12*abs(duration-2.0)
    # Higher is better.  Semantics are applied only in the final ranking so
    # every source receives the same geometric pre-screen.
    score=(3.0*reaction+2.5*visible-2.0*dist['max_joint_delta_rad']
           -.18*entry_s-excessive-wrist_only-rate_penalty-duration_penalty)
    return float(score),{
        'excessive_motion_penalty':float(excessive),
        'wrist_only_penalty':float(wrist_only),
        'estimated_speed_penalty':float(max(0.0,estimated_speed-MAX_SPEED)*12),
        'estimated_acceleration_penalty':float(max(0.0,estimated_acceleration-MAX_ACCEL)*2),
        'duration_penalty':float(duration_penalty),
    }


def _retimed_candidate(candidate,motion,q0):
    raw=motion[candidate['start_frame']:candidate['end_frame'],7:][:,ARM_INDICES]
    t,q=retime_arm_window(raw,candidate['authored_duration_s'],
                          SMOOTHING_WINDOW_FRAMES,OUTPUT_SAMPLE_HZ)
    q[:,INACTIVE]=q0[INACTIVE]
    joint_limits=True
    joint_error=None
    try:
        validate_arm_pose(q)
    except ValueError as exc:
        joint_limits=False;joint_error=str(exc)
    dt=np.diff(t)
    velocity=np.diff(q[:,ACTIVE],axis=0)/dt[:,None]
    acceleration=np.diff(velocity,axis=0)/dt[:-1,None]
    ranges=np.ptp(q[:,ACTIVE],axis=0)
    candidate.update({
        'output_frames':len(q),'output_sample_hz':OUTPUT_SAMPLE_HZ,
        'reaction_range_by_joint_rad':{
            ARM_NAMES[i]:float(ranges[k]) for k,i in enumerate(ACTIVE)},
        'reaction_magnitude_rad':float(ranges.max()),
        'shoulder_elbow_magnitude_rad':float(ranges[:4].max()),
        'wrist_magnitude_rad':float(ranges[4:].max()),
        'max_joint_delta_over_clip_from_q0_rad':
            float(np.max(np.abs(q[:,ACTIVE]-q0[ACTIVE]))),
        'max_velocity_rad_s':float(np.max(np.abs(velocity))),
        'max_acceleration_rad_s2':float(np.max(np.abs(acceleration))),
        'clip_range_passed':bool(ranges.max()<=CLIP_RANGE_LIMIT_RAD),
        'joint_limits_passed':joint_limits,'joint_limit_error':joint_error,
    })
    candidate['entry_transition_s']=float(transition_duration(q0,q[0],2.0))
    candidate['return_transition_s']=float(transition_duration(q[-1],q0,2.0))
    candidate['_clip']=q
    kinematic=(joint_limits and candidate['clip_range_passed'] and
               candidate['max_velocity_rad_s']<=MAX_SPEED and
               candidate['max_acceleration_rad_s2']<=MAX_ACCEL and
               candidate['max_joint_delta_over_clip_from_q0_rad']<=START_POSE_MAX_DELTA_RAD)
    candidate['kinematic_precheck_passed']=bool(kinematic)
    candidate['refined_pre_score']=float(candidate['pre_score']+
        1.5*max(0,1-candidate['max_joint_delta_over_clip_from_q0_rad'])+
        1.0*max(0,1-candidate['max_velocity_rad_s']/MAX_SPEED)+
        .5*max(0,1-candidate['max_acceleration_rad_s2']/MAX_ACCEL)-
        (0 if kinematic else 12))
    return candidate


def _overlaps(a,b):
    intersection=max(0,min(a['end_frame'],b['end_frame'])-max(a['start_frame'],b['start_frame']))
    return intersection/min(a['window_length_frames'],b['window_length_frames'])>.50


def _motion_candidates(path,q0):
    motion,_=read_motion(path)
    arms=motion[:,7:][:,ARM_INDICES]
    right=arms[:,ACTIVE]
    delta=np.abs(right-q0[ACTIVE])
    maximum=delta.max(axis=1)
    l1=delta.sum(axis=1);l2=np.linalg.norm(delta,axis=1)
    eligible=np.flatnonzero(maximum<=START_POSE_MAX_DELTA_RAD)
    stage={
        'motion':path.stem,'frames_scanned':len(motion),'eligible_start_frames':len(eligible),
        'minimum_max_joint_delta_rad':float(maximum.min()),
        'minimum_l1_joint_delta_rad':float(l1.min()),
        'minimum_l2_joint_delta_rad':float(l2.min()),
        'generated_window_duration_combinations':0,'meaningful_window_duration_combinations':0,
    }
    heap=[];serial=0
    right_lower=np.array([ARMS[i]['lower'] for i in ACTIVE])+.03
    right_upper=np.array([ARMS[i]['upper'] for i in ACTIVE])-.03
    for start in eligible:
        dist=_pose_distances(right[start],q0[ACTIVE])
        for length in WINDOW_LENGTHS_FRAMES:
            end=int(start+length)
            if end>len(motion):
                continue
            stage['generated_window_duration_combinations']+=len(AUTHORED_DURATIONS_S)
            window=right[start:end]
            if np.max(np.abs(np.diff(window,axis=0)))>SOURCE_MAX_STEP_RAD:
                continue
            if np.any(window<right_lower) or np.any(window>right_upper):
                continue
            ranges=np.ptp(window,axis=0)
            reaction=float(ranges.max());visible=float(ranges[:4].max());wrist=float(ranges[4:].max())
            if reaction<MIN_REACTION_RANGE_RAD or visible<MIN_SHOULDER_ELBOW_RANGE_RAD:
                continue
            entry=float(transition_duration(q0,np.r_[q0[INACTIVE],right[start]],2.0))
            for duration in AUTHORED_DURATIONS_S:
                stage['meaningful_window_duration_combinations']+=1
                score,penalties=_pre_score(dist,reaction,visible,wrist,entry,duration)
                item={
                    'motion':path.stem,'source_csv':str(path.resolve().relative_to(ROOT)),
                    'source_sha256':digest(path),'start_frame':int(start),'end_frame':end,
                    'window_length_frames':length,'source_fps':None,
                    'source_fps_status':'UNKNOWN; frame interval explicitly retimed',
                    'authored_duration_s':duration,'smoothing_window_frames':SMOOTHING_WINDOW_FRAMES,
                    'start_pose_distance':dist,'raw_reaction_magnitude_rad':reaction,
                    'raw_shoulder_elbow_magnitude_rad':visible,'raw_wrist_magnitude_rad':wrist,
                    'pre_score':score,'pre_score_penalties':penalties,
                }
                serial+=1
                entry_heap=(score,serial,item)
                if len(heap)<PRE_POOL_PER_MOTION:
                    heapq.heappush(heap,entry_heap)
                elif score>heap[0][0]:
                    heapq.heapreplace(heap,entry_heap)
    refined=[]
    for _,_,candidate in sorted(heap,reverse=True):
        refined.append(_retimed_candidate(candidate,motion,q0))
    refined.sort(key=lambda x:(x['kinematic_precheck_passed'],x['refined_pre_score']),reverse=True)
    selected=[]
    for candidate in refined:
        if any(_overlaps(candidate,prior) for prior in selected):
            continue
        selected.append(candidate)
        if len(selected)==PER_MOTION_DETAILED:
            break
    stage['pre_pool_size']=len(heap)
    stage['selected_for_detailed_collision']=len(selected)
    stage['kinematic_pool_pass_count']=sum(x['kinematic_precheck_passed'] for x in refined)
    return motion,stage,selected


def _collision_and_score(candidate,q0,base):
    clip=candidate['_clip']
    entry=transition(q0,clip[0],candidate['entry_transition_s'])
    return_path=transition(clip[-1],q0,candidate['return_transition_s'])
    collision=check_baseline_contact_path(entry,clip,return_path,base,ACTIVE,
                                          minimum=COLLISION_CLEARANCE_M)
    candidate['collision']=collision
    candidate['phase_clearance_m']={
        phase:float(collision['phases'][phase]['raw_minimum_clearance_m'])
        for phase in ('entry','clip','return')}
    candidate['phase_required_pair_clearance_m']={
        phase:float(collision['phases'][phase]['minimum_required_pair_clearance_m'])
        for phase in ('entry','clip','return')}
    candidate['new_penetration_count']=len(collision['new_collision_pairs'])
    candidate['baseline_worsening_count']=len(collision['baseline_rules']['worsened_pairs'])
    candidate['maximum_penetration_m']=max(0.0,-float(collision['minimum_clearance_m']))
    other=(candidate['joint_limits_passed'] and candidate['clip_range_passed'] and
           candidate['max_velocity_rad_s']<=MAX_SPEED and
           candidate['max_acceleration_rad_s2']<=MAX_ACCEL and collision['passed'])
    delta=candidate['max_joint_delta_over_clip_from_q0_rad']
    strict=bool(other and delta<=STRICT_READY_MAX_DELTA_RAD)
    if strict:
        tier='READY <=0.8 rad'
    elif other and delta<=DISPLAY_ONLY_DELTA_TIERS_RAD[0]:
        tier='DISPLAY ONLY <=0.9 rad'
    elif other and delta<=DISPLAY_ONLY_DELTA_TIERS_RAD[1]:
        tier='DISPLAY ONLY <=1.0 rad'
    else:
        tier='FAIL'
    candidate['strict_safety_passed']=strict
    candidate['safety_tier']=tier
    failures=[]
    if delta>STRICT_READY_MAX_DELTA_RAD:failures.append(f'max clip delta {delta:.3f} > 0.8 rad READY limit')
    if not candidate['joint_limits_passed']:failures.append('joint limits failed')
    if not candidate['clip_range_passed']:failures.append('clip range > 0.50 rad')
    if candidate['max_velocity_rad_s']>MAX_SPEED:failures.append('velocity > 0.25 rad/s')
    if candidate['max_acceleration_rad_s2']>MAX_ACCEL:failures.append('acceleration > 1.0 rad/s^2')
    if not collision['passed']:failures.append('baseline-aware entry/clip/return collision check failed')
    candidate['failure_reasons']=failures
    semantic,priority,suggested=GAME_SEMANTICS[candidate['motion']]
    candidate['reaction_type']=semantic
    candidate['semantic_priority_rank']=priority
    candidate['suggested_game_use']=suggested
    safety_points=40 if strict else (20 if tier.startswith('DISPLAY ONLY <=0.9') else
                                    10 if tier.startswith('DISPLAY ONLY <=1.0') else -10)
    pose_points=10*max(0.0,1-delta/STRICT_READY_MAX_DELTA_RAD)
    transition_points=max(0.0,8-candidate['entry_transition_s']-candidate['return_transition_s'])
    reaction_points=max(0.0,10-20*abs(candidate['reaction_magnitude_rad']-.32))
    semantic_points=3*(7-priority)
    candidate['ranking_components']={
        'safety':float(safety_points),'pose_similarity':float(pose_points),
        'short_transition':float(transition_points),'reaction_magnitude':float(reaction_points),
        'game_semantic_priority':float(semantic_points),
    }
    candidate['ranking_score']=float(sum(candidate['ranking_components'].values()))
    return candidate


def _print_top(candidates):
    print('\n=== REACTION SEARCH TOP 10 ===',flush=True)
    if not candidates:
        print('No detailed candidates.')
    for rank,c in enumerate(candidates[:10],1):
        pc=c['phase_clearance_m']
        print(f'\nRank {rank}\nMotion: {c["motion"]}\n'
              f'Start/end: [{c["start_frame"]}, {c["end_frame"]})\n'
              f'Reaction type: {c["reaction_type"]}\n'
              f'Max joint delta from q0: {c["max_joint_delta_over_clip_from_q0_rad"]:.3f} rad\n'
              f'Entry/return transition: {c["entry_transition_s"]:.3f} / {c["return_transition_s"]:.3f} s\n'
              f'Entry/clip/return raw clearance: {pc["entry"]*1000:.3f} / '
              f'{pc["clip"]*1000:.3f} / {pc["return"]*1000:.3f} mm\n'
              f'New penetration / baseline worsening: {c["new_penetration_count"]} / '
              f'{c["baseline_worsening_count"]}\n'
              f'Max velocity / acceleration: {c["max_velocity_rad_s"]:.3f} rad/s / '
              f'{c["max_acceleration_rad_s2"]:.3f} rad/s^2\n'
              f'Reaction magnitude: {c["reaction_magnitude_rad"]:.3f} rad\n'
              f'Safety: {c["safety_tier"]}\nSuggested use: {c["suggested_game_use"]}')


def _write_csv(path,candidates):
    fields=['rank','ranking_score','safety_tier','strict_safety_passed','motion','start_frame',
            'end_frame','window_length_frames','authored_duration_s','reaction_type',
            'suggested_game_use','max_joint_delta_over_clip_from_q0_rad','entry_transition_s',
            'return_transition_s','entry_minimum_clearance_m','clip_minimum_clearance_m',
            'return_minimum_clearance_m','new_penetration_count','baseline_worsening_count',
            'maximum_penetration_m','joint_limits_passed','clip_range_passed',
            'max_velocity_rad_s','max_acceleration_rad_s2','reaction_magnitude_rad',
            'shoulder_elbow_magnitude_rad','wrist_magnitude_rad','source_csv','source_sha256']
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for candidate in candidates:
            row={key:candidate.get(key) for key in fields}
            for phase in ('entry','clip','return'):
                row[f'{phase}_minimum_clearance_m']=candidate['phase_clearance_m'][phase]
            writer.writerow(row)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state-json',type=Path,required=True)
    parser.add_argument('--ownership-json',type=Path,required=True)
    parser.add_argument('--stamp',required=True,help='Timestamp shared by all outputs')
    args=parser.parse_args()
    state_path=args.state_json.resolve();ownership_path=args.ownership_json.resolve()
    state,ownership=_load_observation(state_path,ownership_path)
    q0=np.asarray(state['arm_q'],dtype=float);validate_arm_pose(q0)
    base,baseline,_=_base_and_baseline(state)
    current_pose_path=ROOT/'output'/f'current_pose_search_{args.stamp}.json'
    current_pose=_write_current_pose(current_pose_path,state,ownership,state_path,ownership_path,baseline)
    print(f'Current pose saved: {current_pose_path}',flush=True)

    motion_paths=sorted((ROOT/'data/motions').glob('*.csv'))
    unknown=[path.stem for path in motion_paths if path.stem not in GAME_SEMANTICS]
    missing=sorted(set(GAME_SEMANTICS)-{path.stem for path in motion_paths})
    if unknown or missing:
        raise ValueError(f'Motion inventory mismatch; unknown={unknown}, missing={missing}')
    stages=[];detail=[];motion_cache={}
    for path in motion_paths:
        motion,stage,selected=_motion_candidates(path,q0)
        motion_cache[path.stem]=motion;stages.append(stage);detail.extend(selected)
        print(f'Stage A/B {path.stem}: frames={stage["frames_scanned"]}, '
              f'near={stage["eligible_start_frames"]}, generated='
              f'{stage["generated_window_duration_combinations"]}, detailed={len(selected)}',flush=True)
    detail.sort(key=lambda x:(x['kinematic_precheck_passed'],x['refined_pre_score']),reverse=True)
    detail=detail[:30]
    evaluated=[]
    for index,candidate in enumerate(detail,1):
        print(f'Collision {index:02d}/{len(detail)}: {candidate["motion"]} '
              f'[{candidate["start_frame"]},{candidate["end_frame"]})',flush=True)
        evaluated.append(_collision_and_score(candidate,q0,base))
    evaluated.sort(key=lambda x:(x['strict_safety_passed'],
                                 x['safety_tier'].startswith('DISPLAY ONLY <=0.9'),
                                 x['safety_tier'].startswith('DISPLAY ONLY <=1.0'),
                                 x['ranking_score']),reverse=True)
    for rank,candidate in enumerate(evaluated,1):candidate['rank']=rank
    _print_top(evaluated)

    serializable=[]
    for candidate in evaluated:
        row={key:value for key,value in candidate.items() if key!='_clip'}
        serializable.append(row)
    strict_count=sum(x['strict_safety_passed'] for x in evaluated)
    tier09_count=sum(x['safety_tier']=='DISPLAY ONLY <=0.9 rad' for x in evaluated)
    tier10_count=sum(x['safety_tier']=='DISPLAY ONLY <=1.0 rad' for x in evaluated)
    report={
        'schema':'motiondecode-test.reaction-search.v1',
        'created_local_time':time.strftime('%Y-%m-%d %H:%M:%S %z'),
        'receive_only':True,'command_publishers_created':0,'real_robot_command_sent':False,
        'current_pose_json':str(current_pose_path.resolve()),
        'source_state_json':str(state_path),'source_ownership_json':str(ownership_path),
        'ownership_result':ownership['ownership_preflight'],
        'game_semantics_note':'ゲーム用途としてのこちらの分類。MotionDecode作者の正式ラベルではない。',
        'thresholds':{
            'stage_a_start_max_joint_delta_rad':START_POSE_MAX_DELTA_RAD,
            'strict_ready_max_joint_delta_over_clip_rad':STRICT_READY_MAX_DELTA_RAD,
            'display_only_delta_tiers_rad':list(DISPLAY_ONLY_DELTA_TIERS_RAD),
            'window_lengths_frames':list(WINDOW_LENGTHS_FRAMES),
            'authored_durations_s':list(AUTHORED_DURATIONS_S),
            'source_fps':None,'smoothing_window_frames':SMOOTHING_WINDOW_FRAMES,
            'output_sample_hz':OUTPUT_SAMPLE_HZ,
            'minimum_reaction_range_rad':MIN_REACTION_RANGE_RAD,
            'minimum_shoulder_elbow_range_rad':MIN_SHOULDER_ELBOW_RANGE_RAD,
            'clip_range_limit_rad':CLIP_RANGE_LIMIT_RAD,
            'source_max_step_rad':SOURCE_MAX_STEP_RAD,
            'max_velocity_rad_s':MAX_SPEED,'max_acceleration_rad_s2':MAX_ACCEL,
            'collision_clearance_m':COLLISION_CLEARANCE_M,
            'per_motion_detailed_candidates':PER_MOTION_DETAILED,
            'pre_pool_per_motion':PRE_POOL_PER_MOTION,
        },
        'pre_score_formula':(
            '3*reaction + 2.5*shoulder_elbow - 2*start_max_delta - 0.18*entry_time '
            '- excessive_motion_penalty - wrist_only_penalty - estimated_rate_penalties '
            '- duration_penalty'),
        'ranking_formula':'weighted sum of safety, pose similarity, short transition, reaction magnitude, and game semantic priority',
        'current_pose_summary':{
            'right_arm_q0_rad':current_pose['q0']['right_arm']['values_rad'],
            'baseline_collision':current_pose['baseline_collision'],
        },
        'search_size':{
            'motion_count':len(motion_paths),
            'scanned_frame_count':sum(x['frames_scanned'] for x in stages),
            'generated_window_count':sum(x['generated_window_duration_combinations'] for x in stages),
            'meaningful_pre_scored_window_count':sum(x['meaningful_window_duration_combinations'] for x in stages),
            'detailed_collision_check_count':len(evaluated),
        },
        'stage_summaries':stages,
        'pass_counts':{
            'strict_ready_le_0_8_rad':strict_count,
            'display_only_le_0_9_rad':tier09_count,
            'display_only_le_1_0_rad':tier10_count,
        },
        'result':('REAL ROBOT CANDIDATE FOUND' if strict_count else 'NO SAFE REACTION CANDIDATE'),
        'candidates':serializable,
    }
    json_path=ROOT/'output'/f'reaction_search_ranked_{args.stamp}.json'
    csv_path=ROOT/'output'/f'reaction_search_ranked_{args.stamp}.csv'
    write_json(json_path,report);_write_csv(csv_path,evaluated)
    print(f'\nStrict PASS: {strict_count}; display-only 0.9/1.0: {tier09_count}/{tier10_count}')
    print(report['result'])
    print(f'JSON: {json_path}\nCSV: {csv_path}',flush=True)


if __name__=='__main__':
    raise SystemExit(main())
