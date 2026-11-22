"""Post-run Phase 3.2 scenario checks; no influence on model output."""
import argparse
import json
from pathlib import Path
from brain.state import State
from brain.split_brain import parse_flat
from brain.split_schemas import CurrentObservation, PlannerDecision, wait_planner_decision
from run_video import write_report


def evaluate(report, scenario):
    errors=[];state=State();frames=report.get('frames',[])
    if [f['timestamp'] for f in frames]!=scenario['timestamps']: errors.append('Frame schedule mismatch')
    if report.get('status')!='pass': errors.append('Pipeline failed')
    for f in frames:
        p,d=f['perception'],f['planner'];obs=None;decision=None
        try: obs=parse_flat(p['raw_response'],CurrentObservation)
        except (ValueError,TypeError): pass
        if f['state_before']!=state.snapshot.model_dump(mode='json'): errors.append(f"State before mismatch at {f['timestamp']}")
        if obs is not None:
            if p.get('fallback') or p.get('observation')!=obs.model_dump(mode='json'): errors.append(f"Raw/Observation mismatch at {f['timestamp']}")
            state.update(obs)
        elif not p.get('fallback') or p.get('observation') is not None: errors.append(f"Invalid Perception accepted at {f['timestamp']}")
        if f['state_after']!=state.snapshot.model_dump(mode='json'): errors.append(f"State update mismatch at {f['timestamp']}")
        try: decision=parse_flat(d['raw_response'],PlannerDecision)
        except (ValueError,TypeError): pass
        if decision is not None:
            if d.get('fallback') or d['decision']!=decision.model_dump(mode='json',exclude_none=True): errors.append(f"Raw/Decision mismatch at {f['timestamp']}")
        elif not d.get('fallback') or d['decision']!=wait_planner_decision().model_dump(mode='json',exclude_none=True): errors.append(f"Fallback Decision mismatch at {f['timestamp']}")
        if obs is None and (d.get('raw_response') or not d.get('fallback')): errors.append(f"Planner executed after invalid Perception at {f['timestamp']}")
        if report.get('mode') == 'saved-observation text-only planner':
            if p.get('inference_executed') is not False or p.get('metrics') != {}:
                errors.append(f"Saved replay Perception execution mismatch at {f['timestamp']}")
        elif p.get('metrics',{}).get('image_count')!=1: errors.append(f"Perception image count mismatch at {f['timestamp']}")
        if obs is not None and d.get('metrics',{}).get('image_count')!=0: errors.append(f"Planner received image at {f['timestamp']}")
    checks=[]
    for rule in scenario['checks']:
        found=[f for f in frames if f['timestamp']==rule['timestamp']];failures=[]
        if len(found)!=1: failures.append('Missing/duplicate frame')
        else:
            f=found[0];obs=f['perception']['observation'] or {};snapshot=f['state_after'];d=f['planner']['decision']
            if f['perception']['fallback'] or obs.get('person_visible')!=rule['person_visible']: failures.append('Perception presence mismatch')
            if obs.get('person_direction')!=rule['person_direction']: failures.append('Perception direction mismatch')
            if snapshot.get('transition')!=rule['transition']: failures.append('State transition mismatch')
            if d.get('action') not in rule['allowed_actions'] or f['planner']['fallback']: failures.append('Planner Action mismatch')
            if d.get('direction')!=rule['decision_direction'] or d.get('target')!=rule.get('decision_target','PERSON'): failures.append('Planner direction/target mismatch')
            if 'frames_since_person_seen' in rule and snapshot.get('frames_since_person_seen')!=rule['frames_since_person_seen']:
                failures.append('Missed frame count mismatch')
            if 'decision_distance' in rule and d.get('distance')!=rule['decision_distance']:
                failures.append('Decision distance mismatch')
            if rule['transition']=='DISAPPEARED':
                if f['state_before'].get('last_seen_person_direction')!='LEFT' or snapshot.get('last_seen_person_direction')!='LEFT': failures.append('Last known direction mismatch')
                if snapshot.get('frames_since_person_seen')!=1 or d.get('distance')!='UNKNOWN': failures.append('Search state/distance mismatch')
        checks.append({'timestamp':rule['timestamp'],'pass':not failures,'errors':failures})
    return {'pass':not errors and all(c['pass'] for c in checks),'integrity_errors':errors,'checks':checks}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path);parser.add_argument('scenario',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=evaluate(*(json.loads(p.read_text(encoding='utf-8')) for p in (args.report,args.scenario)))
    write_report(args.output,result);print(json.dumps(result,indent=2));return 0 if result['pass'] else 1

if __name__=='__main__': raise SystemExit(main())
