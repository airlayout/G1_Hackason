"""Only post-run evaluators read labels/event annotations. Runtime never does."""
import argparse
import json
from pathlib import Path
from time import perf_counter
from evaluate_latency import distribution
from evaluate_phase33 import evaluate_phase33
from brain.split_brain import parse_flat
from brain.split_schemas import CurrentObservation,PlannerDecision
from brain.state import State
from hybrid.core import RecordingRobot
from run_video import write_report


def stats(values):
    values=[v for v in values if v is not None]
    return {**distribution(values),'max':max(values) if values else None}


def selected(report,labels):
    lookup={x['timestamp']:x for x in labels['labels']}; rows=[]; main=[]; transitions=[]
    for run in report['runs']:
        state=State(); states=[]
        for r in run['results']:
            obs=CurrentObservation.model_validate(r['observation']); snapshot=state.update(obs)
            states.append({'timestamp':r['requested_timestamp'],'state':snapshot.model_dump(mode='json')})
        if len(run['timestamps'])==9: rows.extend(run['results'])
        else:
            main.append([r['observation']['person_direction'] for r in run['results']]==['LEFT','UNKNOWN','LEFT']
                and [s['state']['transition'] for s in states]==['UNKNOWN','DISAPPEARED','APPEARED']
                and states[1]['state']['last_seen_person_direction']=='LEFT')
        transitions.append(states)
    presence=sum(r['observation']['person_visible']==lookup[r['requested_timestamp']]['person_visible'] for r in rows)
    visible=[r for r in rows if lookup[r['requested_timestamp']]['person_visible']]
    direction=sum(r['observation']['person_direction']==lookup[r['requested_timestamp']]['person_direction'] for r in visible)
    eligible=presence==len(rows) and direction/len(visible)>=.8 and all(main)
    return {'presence_correct_total':[presence,len(rows)],'direction_correct_total':[direction,len(visible)],
        'presence_accuracy':presence/len(rows),'direction_accuracy':direction/len(visible),
        'main_geometry_pass':all(main),'eligible':eligible,
        'latency_s':stats([r['inference_latency_s'] for r in rows]),
        'frame_age_s':stats([r['received_timestamp']-r['capture_timestamp'] for r in rows]),
        'peak_allocated_gib':max(r.get('peak_allocated_gib',0) for r in rows),
        'peak_reserved_gib':max(r.get('peak_reserved_gib',0) for r in rows),'state_replay':transitions}


def original_policy(row):
    """Use only the unchanged predicate, not its single-model/full-history integrity verdict."""
    inp=row['planner_input']; decision=parse_flat(row['raw_response'],PlannerDecision)
    obs=CurrentObservation.model_validate(inp['current_observation']); robot=RecordingRobot(); robot.execute(decision)
    after={**inp['memory'],'transition':inp['transition']}
    frame={'timestamp':row['source_timestamp'],'state_before':State().snapshot.model_dump(mode='json'),'state_after':after,
        'perception':{'raw_response':obs.model_dump_json(),'observation':obs.model_dump(mode='json'),
                      'bare_json':True,'fallback':False,'metrics':{}},
        'planner':{'raw_response':row['raw_response'],'decision':decision.model_dump(mode='json',exclude_none=True),
                   'bare_json':True,'fallback':False,'metrics':row['metrics']},'mock_robot_output':robot.last_output}
    verdict=evaluate_phase33({'mode':'policy-only event view','status':'pass','frames':[frame]},
                            {'timestamps':[row['source_timestamp']],'checks':[]})
    return verdict['planner_policy_checks'][0]['pass']


def realtime(report,event_annotations):
    fast=report['fast_results']; accepted=[r for r in fast if r.get('accepted')]
    slow=report['slow_results']; active=[(r['processing_start'],r['processing_finish']) for r in slow]
    commands=report['commands']; origin=report['origin_perf_counter']
    interval=[b['received_timestamp']-a['received_timestamp'] for a,b in zip(accepted,accepted[1:])]
    overlap=[r for r in fast if any(r['processing_start']<end and r['processing_finish']>start for start,end in active)]
    look=[c for c in commands if c['channel']=='immediate_geometry']
    look_busy=[c for c in look if any(start<=c['ready_timestamp']<=end for start,end in active)]
    errors=[]; replay=State(); generation=-1; cap=float('-inf')
    for r in fast:
        before=replay.snapshot.model_dump(mode='json')
        if r.get('accepted'):
            if r['generation']<=generation or r['capture_timestamp']<=cap: errors.append('Old result accepted')
            obs=CurrentObservation.model_validate(r['observation']); replay.update(obs)
            generation,cap=r['generation'],r['capture_timestamp']
        if before!=r['state_before'] or replay.snapshot.model_dump(mode='json')!=r['state_after']:
            errors.append('State history mismatch')
        if r.get('cooperative_fast_update') and not r.get('slow_rope_cache_restored'):
            errors.append('Cooperative model RoPE cache not restored')
    valid_planners=[]; policy=[]; cancelled=[]
    for r in slow:
        if r.get('cancelled'):
            cancelled.append({'kind':r['kind'],'generation':r['generation'],'raw_response':r.get('raw_response')}); continue
        if r['kind']=='planner':
            try:
                decision=parse_flat(r['raw_response'],PlannerDecision)
                if decision.model_dump(mode='json',exclude_none=True)!=r['decision']: errors.append('Planner overwritten')
                if r['metrics']['image_count']!=0: errors.append('Image passed to Planner')
                if set(r['planner_input'])!={'current_observation','memory','transition'}: errors.append('Extra input passed to Planner')
                valid_planners.append(r); policy.append(original_policy(r))
            except (ValueError,TypeError): errors.append('Invalid original JSON Planner accepted')
    for c in commands:
        decision=PlannerDecision.model_validate(c['decision']); robot=RecordingRobot(); robot.execute(decision)
        if robot.last_output!=c['mock_robot_output']: errors.append('Unvalidated/changed Robot command')
        if c['channel']=='immediate_geometry' and (decision.action!='LOOK' or decision.target!='PERSON'):
            errors.append('Fast rule invented semantic action')
        if c['channel']=='vlm_semantic':
            original=parse_flat(c['raw_response'],PlannerDecision)
            if original.model_dump(mode='json',exclude_none=True)!=c['raw_decision']: errors.append('Raw semantic intent changed')
            if any(getattr(original,k)!=getattr(decision,k) for k in ['action','target','distance']): errors.append('Resolver rewrote action meaning')
            if original.action=='SEARCH' and original.direction!=decision.direction: errors.append('SEARCH memory direction changed')
    kpis={}
    for event in event_annotations['events']:
        lo,hi=event['lower_source_timestamp'],event['upper_source_timestamp']
        if event['type']=='disappeared':
            matches=[r for r in accepted if lo<=r['source_timestamp']<=hi+2 and r['state_after']['transition']=='DISAPPEARED']
            detection=matches[0] if matches else None
            searches=[c for c in commands if c['channel']=='vlm_semantic' and lo<=c['source_timestamp']<=hi+2
                      and c['decision']['action']=='SEARCH' and c['decision']['direction']==event['last_seen_direction']]
            command=searches[0] if searches else None
            kpis[event['name']]={'detection_latency_upper_s':detection['received_timestamp']-origin-lo if detection else None,
                'detection_latency_lower_s':max(0,detection['received_timestamp']-origin-hi) if detection else None,
                'detected_generation':detection['generation'] if detection else None,
                'search_ready_upper_s':command['ready_timestamp']-origin-lo if command else None,
                'search_from_detection_s':command['ready_timestamp']-detection['received_timestamp'] if command and detection else None,
                'search_ready':command['decision'] if command else None}
        else:
            # Conservative upper bound: command using a confirmed post-event frame.
            matches=[c for c in look if hi<=c['source_timestamp']<=hi+2 and c['decision']['direction']==event['direction']]
            command=matches[0] if matches else None
            kpis[event['name']]={'look_ready_upper_s':command['ready_timestamp']-origin-lo if command else None,
                'look_ready_lower_s':max(0,command['ready_timestamp']-origin-hi) if command else None,
                'generation':command['generation'] if command else None}
    busy_geometry=stats([c['event_to_ready_age_s'] for c in look_busy])
    fast_loop_pass=(bool(accepted) and stats(interval)['max'] is not None and stats(interval)['max']<=.25
                    and (not slow or bool(overlap)) and (not slow or not look_busy or busy_geometry['p95']<=.25))
    resources=report['resources']; metrics=[r for r in fast+slow if not r.get('cancelled')]
    video_duration=report['video']['duration_s']
    disappear=next(e for e in event_annotations['events'] if e['type']=='disappeared')
    reappear=next(e for e in event_annotations['events'] if e['type']=='reappeared')
    verified_empty=[r for r in accepted if disappear['upper_source_timestamp']<=r['source_timestamp']<=reappear['lower_source_timestamp']]
    return {'case':report['case'],'status':report['status'],'fast_count':len(fast),'accepted_count':len(accepted),
        'rejected_count':len(fast)-len(accepted),'inference_latency_s':stats([r['inference_latency_s'] for r in fast]),
        'slow_planner_latency_s':stats([r['inference_latency_s'] for r in valid_planners]),
        'slow_semantic_latency_s':stats([r['inference_latency_s'] for r in slow if r['kind']=='semantic' and not r.get('cancelled')]),
        'result_frame_age_s':stats([r['received_timestamp']-r['capture_timestamp'] for r in fast]),
        'update_interval_s':stats(interval),'update_rate_hz':len(accepted)/video_duration,
        'overlap_fast_count':len(overlap),'overlap_latency_s':stats([r['inference_latency_s'] for r in overlap]),
        'verified_empty_window_false_positive_total':[sum(r['observation']['person_visible'] for r in verified_empty),len(verified_empty)],
        'look_frame_to_ready_s':stats([c['event_to_ready_age_s'] for c in look]),
        'look_while_slow_frame_to_ready_s':busy_geometry,'look_while_slow_count':len(look_busy),
        'slow_active_fast_loop_pass':fast_loop_pass,'latest_frame_wins':report['slot_capacity']==1,
        'no_queue_accumulation':report['queue_capacity']==0 and report['max_in_flight_per_worker']==1,
        'stale_result_rejection_pass':not errors,'high_level_planner_integrity_pass':not errors,
        'policy_correct_total':[sum(policy),len(policy)],'cancelled_jobs':cancelled,'integrity_errors':errors,
        'event_kpis':kpis,'source_annotation_note':'Event latencies are conservative upper/lower bounds from visually annotated intervals; GT is evaluator-only.',
        'cpu_process_percent':{pid:stats([x['cpu_process_percent'].get(pid) for x in resources]) for pid in resources[0]['cpu_process_percent']},
        'gpu_utilization_percent':stats([x.get('gpu_utilization_percent') for x in resources]),
        'gpu_used_gib':stats([x.get('gpu_used_gib') for x in resources]),
        'peak_allocated_gib':max((r.get('peak_allocated_gib',r.get('metrics',{}).get('peak_allocated_gib',0)) for r in metrics),default=None),
        'peak_reserved_gib':max((r.get('peak_reserved_gib',r.get('metrics',{}).get('peak_reserved_gib',0)) for r in metrics),default=None),
        'fast_peak_allocated_gib':max((r.get('peak_allocated_gib',r.get('metrics',{}).get('peak_allocated_gib',0)) for r in fast),default=None),
        'fast_peak_reserved_gib':max((r.get('peak_reserved_gib',r.get('metrics',{}).get('peak_reserved_gib',0)) for r in fast),default=None),
        'planner_trigger_count':sum(e['type']=='planner_trigger' for e in report['events']),
        'slow_dispatch_count':sum(e['type']=='slow_dispatch' for e in report['events']),
        'cooperative_single_weight':report.get('cooperative_single_weight',False),
        'shared_qwen_negative_control':report.get('shared_qwen_negative_control',False)}


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--root',type=Path,default=Path('reports/phase36'))
    args=parser.parse_args(); labels=json.loads(Path('scenarios/IMG_7121_phase32_perception.json').read_text())
    annotations=json.loads(Path('scenarios/IMG_7121_phase36_events.json').read_text())
    for folder in ['selected','selected_lowconfidence']:
        for path in (args.root/folder).glob('*.json'):
            result=selected(json.loads(path.read_text()),labels); write_report(args.root/'evaluations'/folder/path.name,result)
            print(path.stem,'selected',result['presence_correct_total'],result['direction_correct_total'])
    for folder in ['realtime','realtime_preempt','realtime_cooperative','realtime_lowconfidence','realtime_final','realtime_eventsemantic']:
        for path in (args.root/folder).glob('*.json'):
            report=json.loads(path.read_text())
            if report.get('status')!='complete': print('Incomplete profile retained:',path,report.get('error')); continue
            result=realtime(report,annotations); write_report(args.root/'evaluations'/folder/path.name,result)
            print(path.stem,'fast loop',result['slow_active_fast_loop_pass'],'integrity',result['integrity_errors'])


if __name__=='__main__': main()
