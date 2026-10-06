"""Post-run labels only. Original policy evaluator, original datasets, new timing gates."""
import argparse
import copy
import json
from pathlib import Path
from brain.split_schemas import CurrentObservation
from brain.state import State
from phase35 import codecs
from evaluate_latency import distribution, evaluate_report
from run_video import write_report

METRICS = ['processor_s','device_transfer_s','image_preprocessing_s',
           'patch_embed_cuda_elapsed_s','vision_cuda_elapsed_s','prefill_cuda_elapsed_s',
           'ttft_stage_s','ttft_generation_s','decode_total_s','text_decoding_s',
           'input_tokens','image_tokens','output_tokens','validation_parsing_s',
           'peak_allocated_gib','peak_reserved_gib','stage_wall_s']


def evaluate(report, labels, nine, three):
    if report['experiment']!='A':
        view=copy.deepcopy(report); errors=[]
        for run in view['runs']:
            # Runner completion and serialization validity are distinct. The unchanged
            # legacy evaluator expects the latter as its pipeline status.
            run['status']='pass' if all(not f[s]['fallback'] for f in run['frames'] for s in ['perception','planner']) else 'failed'
            for frame in run['frames']:
                current = None
                for stage, field in [('perception','observation'),('planner','decision')]:
                    result=frame[stage]
                    try:
                        format=report['profile']['format'] if stage=='perception' else 'planner_class'
                        parsed=(codecs.observation(result['raw_response'],format) if stage=='perception'
                                else codecs.decision(result['raw_response'],format,current))
                        payload=parsed.model_dump(mode='json',exclude_none=stage=='planner')
                        if payload!=result[field] or result['fallback']:
                            errors.append(f'Raw/{field} mismatch t={frame["timestamp"]}')
                        if stage=='perception': current=parsed
                        if stage=='planner':
                            from brain.split_prompt import planner_user
                            from brain.state import StateSnapshot
                            expected=json.loads(planner_user(current,StateSnapshot.model_validate(frame['state_after'])))
                            if result['metrics'].get('planner_input')!=expected:
                                errors.append(f'Planner input mismatch t={frame["timestamp"]}')
                        result['raw_response']=json.dumps(payload); result['bare_json']=True
                    except (ValueError,TypeError,AttributeError) as exc:
                        errors.append(f'Invalid {stage} t={frame["timestamp"]}: {exc}')
        view['profile'].update(format='json',planner_class=False,planner_json=False)
        view['status']='pass' if all(r['status']=='pass' for r in view['runs']) else 'failed'
        result=evaluate_report(view,labels,nine,three)
        result['serialization_errors']=errors
        result['eligible']=result['eligible'] and not errors and all(r['planner_policy_pass'] for r in result['run_evaluations'])
        result['policy_scores']=[sum(c['pass'] for c in r['planner_policy_checks']) for r in result['run_evaluations']]
        result['policy_total']=[sum(result['policy_scores']),sum(len(r['planner_policy_checks']) for r in result['run_evaluations'])]
        result['profile']=report['profile']
        result['tier']=tier(result['metrics']['total_brain_s']['mean']) if result['eligible'] else 'REJECT (accuracy)'
        return result
    lookup={r['timestamp']:r for r in labels['labels']}; rows=[]; errors=[]; scores=[]; main=[]
    for run in report['runs']:
        state=State(); score=0
        for frame in run['frames']:
            p=frame['perception']; label=lookup[frame['timestamp']]
            try:
                obs=codecs.observation(p['raw_response'],'class')
                if obs.model_dump(mode='json')!=p['observation'] or p['fallback']:
                    errors.append('Raw Observation mismatch')
                if state.snapshot.model_dump(mode='json')!=frame['state_before']: errors.append('State before mismatch')
                if state.update(obs).model_dump(mode='json')!=frame['state_after']: errors.append('State after mismatch')
                correct=(obs.person_visible==label['person_visible'] and
                    (not label['person_visible'] or obs.person_direction.value==label['person_direction']))
                score+=int(correct)
            except (ValueError,TypeError): errors.append('Invalid geometry class')
        if run['timestamps']==nine['timestamps']:
            scores.append(score); rows.extend(run['frames'])
        else: main.append(score==len(three['timestamps']))
    presence=sum(f['perception']['observation']['person_visible']==lookup[f['timestamp']]['person_visible'] for f in rows)
    visible=[f for f in rows if lookup[f['timestamp']]['person_visible']]
    direction=sum(f['perception']['observation']['person_direction']==lookup[f['timestamp']]['person_direction'] for f in visible)
    valid=sum(not f['perception']['fallback'] for f in rows)
    reuse=[f['perception']['metrics'] for f in rows]
    if (len({m.get('model_instance_id') for m in reuse})!=1 or
        len({m.get('processor_instance_id') for m in reuse})!=1 or
        any(m.get('model_load_count')!=1 or m.get('output_tokens')!=1 or m.get('image_count')!=1 for m in reuse)):
        errors.append('One-token/image/reuse invariant failed')
    eligible=(presence==len(rows) and direction/len(visible)>=.8 and all(main) and valid==len(rows) and not errors)
    return {'presence_correct_total':[presence,len(rows)],'direction_correct_total':[direction,len(visible)],
        'presence_accuracy':presence/len(rows),'direction_accuracy':direction/len(visible),
        'valid_class_rate':valid/len(rows),'main_pass':all(main),'extended_geometry_scores':scores,
        'eligible':eligible,'integrity_errors':errors,
        'metrics':{'perception_wall_s':distribution([f['timing']['perception_wall_s'] for f in rows]),
                   'perception':{key:distribution([f['perception']['metrics'].get(key) for f in rows]) for key in METRICS}},
        'peak_allocated_gib':max(m['peak_allocated_gib'] for m in reuse),
        'peak_reserved_gib':max(m['peak_reserved_gib'] for m in reuse),
        'note':'Geometry-only gate, not full Observation or Action accuracy. Warmup/load/fresh main excluded from aggregates.'}


def tier(mean):
    return 'S' if mean<=.5 else 'A' if mean<=1 else 'B' if mean<=1.5 else 'C' if mean<=2.5 else 'D'


def evaluate_dual(report, labels, nine, three):
    from evaluate_phase33 import evaluate_phase33
    from adapters.robot_mock import MockRobot
    from brain.split_brain import parse_flat
    from brain.split_schemas import PlannerDecision
    from phase35.dual_rate import SharedState, Stamp, execute_resolved
    from brain.split_prompt import planner_user
    runs=[]; errors=[]; rows=[]
    for run in report['runs']:
        view={'mode':'perception-state-planner','status':'pass','timestamps':run['timestamps'],'frames':[]}
        shared=SharedState()
        for row in run['frames']:
            fast=codecs.observation(row['fast_raw'],'class')
            semantic=parse_flat(row['semantic_raw'],CurrentObservation)
            stamp=Stamp(**row['stamp']); before=shared.state.snapshot.model_dump(mode='json')
            shared.update_fast(fast,stamp); shared.update_semantic(semantic,stamp)
            combined=shared.combined(); original=parse_flat(row['planner_raw'],PlannerDecision)
            if (fast.model_dump(mode='json')!=row['fast_observation'] or
                semantic.model_dump(mode='json')!=row['semantic_observation'] or
                combined.model_dump(mode='json')!=row['combined_observation'] or
                before!=row['state_before'] or shared.state.snapshot.model_dump(mode='json')!=row['state_after'] or
                json.loads(planner_user(combined,shared.state.snapshot))!=row['planner_input'] or
                original.model_dump(mode='json',exclude_none=True)!=row['planner_decision']):
                errors.append(f'Raw/fusion/state/input mismatch at {row["timestamp"]}')
            executed_robot=MockRobot()
            resolved=execute_resolved(executed_robot,original,shared)
            resolved_payload=resolved.model_dump(mode='json',exclude_none=True) if resolved else None
            if (resolved_payload!=row['resolved_final_decision'] or
                    executed_robot.last_output!=row['mock_robot_output']):
                errors.append(f'Resolver/validated Mock output mismatch at {row["timestamp"]}')
            # Evaluate ORIGINAL model decision before resolver. Never credit binding
            # for correcting a wrong Planner action or direction.
            robot=MockRobot(); robot.execute(original)
            view['frames'].append({'timestamp':row['timestamp'],'state_before':before,
                'state_after':row['state_after'],'transition':row['state_after']['transition'],
                'perception':{'raw_response':combined.model_dump_json(),'bare_json':True,'fallback':False,
                    'observation':combined.model_dump(mode='json'),'metrics':row['fast_metrics']},
                'planner':{'raw_response':row['planner_raw'],'bare_json':True,'fallback':False,
                    'decision':row['planner_decision'],'metrics':row['planner_metrics']},
                'mock_robot_output':robot.last_output})
        scenario=nine if run['timestamps']==nine['timestamps'] else three
        runs.append(evaluate_phase33(view,scenario))
        if run['timestamps']==nine['timestamps']: rows.extend(run['frames'])
    lookup={r['timestamp']:r for r in labels['labels']}
    presence=sum(r['fast_observation']['person_visible']==lookup[r['timestamp']]['person_visible'] for r in rows)
    visible=[r for r in rows if lookup[r['timestamp']]['person_visible']]
    direction=sum(r['fast_observation']['person_direction']==lookup[r['timestamp']]['person_direction'] for r in visible)
    late=report['late_delivery_dry_run']; before=late['geometry_before_old_delivery']; after=late['geometry_after_old_delivery']
    old=late['planner_decision']; resolved=late['resolved_final_decision']
    resolver_pass=(before==after and after['person_direction']=='RIGHT' and old['action']=='LOOK'
        and old['direction']=='LEFT' and resolved is not None and resolved['direction']=='RIGHT'
        and all(resolved[k]==old[k] for k in ['action','target','distance']))
    allmetrics=[r[k] for r in rows for k in ['fast_metrics','semantic_metrics','planner_metrics']]
    reuse=(len({m['model_instance_id'] for m in allmetrics})==1 and
           len({m['processor_instance_id'] for m in allmetrics})==1 and all(m['model_load_count']==1 for m in allmetrics))
    if not reuse: errors.append('Duplicate model/processor detected')
    return {'prototype_contract_pass':not errors and reuse and resolver_pass,
        'latest_geometry_resolver_pass':resolver_pass,'model_reuse':reuse,'integrity_errors':errors,
        'presence_correct_total':[presence,len(rows)],'direction_correct_total':[direction,len(visible)],
        'extended_original_planner_scores':[sum(c['pass'] for c in r['checks']) for r in runs[:-1]],
        'policy_original_scores':[sum(c['pass'] for c in r['planner_policy_checks']) for r in runs],
        'main_original_planner_pass':runs[-1]['pass'],
        'metrics':{k:distribution([r['timing'][k] for r in rows]) for k in
            ['fast_s','semantic_s','planner_s','cycle_s','fast_blocked_by_semantic_planner_s']},
        'peak_allocated_gib':max(m['peak_allocated_gib'] for m in allmetrics),
        'peak_reserved_gib':max(m['peak_reserved_gib'] for m in allmetrics),
        'run_evaluations':runs,
        'note':'Prototype PASS means precedence/reuse/binding contracts, not fast E2E viability. Semantic+Planner are nonpreemptible blocking jobs.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('reports',nargs='+',type=Path)
    args=parser.parse_args(); root=Path(__file__).parent
    data=[json.loads((root/'scenarios'/name).read_text()) for name in
          ['IMG_7121_phase32_perception.json','IMG_7121_phase33_nine.json','IMG_7121_phase33_three.json']]
    for path in args.reports:
        report=json.loads(path.read_text())
        result=evaluate_dual(report,*data) if report['experiment']=='D' else evaluate(report,*data)
        write_report(path.parent/'evaluations'/path.name,result)
        print(json.dumps({k:v for k,v in result.items() if k not in ['metrics','run_evaluations','profile']}))


if __name__=='__main__': main()
