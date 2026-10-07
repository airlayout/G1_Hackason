"""Post-run evaluation: decode serialization, then reuse the unchanged scenario criteria."""
import argparse
import copy
import json
import math
import statistics
from pathlib import Path
from latency import codecs
from evaluate_phase33 import evaluate_phase33
from run_video import write_report


def distribution(values):
    values=sorted(v for v in values if v is not None)
    if not values: return {'count':0,'mean':None,'median':None,'p95':None}
    position=(len(values)-1)*.95; lo=math.floor(position); hi=math.ceil(position)
    return {'count':len(values),'mean':statistics.mean(values),'median':statistics.median(values),
            'p95':values[lo]+(values[hi]-values[lo])*(position-lo)}


def evaluate_run(run,profile,scenario):
    view=copy.deepcopy(run); serialization_errors=[]
    for frame in view['frames']:
        for stage,decoder,field in [('perception',codecs.observation,'observation'),('planner',codecs.decision,'decision')]:
            result=frame[stage]
            try:
                format=('planner_class' if stage=='planner' and profile.get('planner_class') else
                        'json' if stage=='planner' and profile.get('planner_json') else profile['format'])
                decoded=decoder(result['raw_response'],format)
                payload=decoded.model_dump(mode='json',exclude_none=stage=='planner')
                if result[field]!=payload or result['fallback']:
                    serialization_errors.append(f"Raw/{field} mismatch at {frame['timestamp']}")
                # Evaluation view only: schema-equivalent representation of raw codes.
                # Original raw response and bare_json remain untouched in the source report.
                result['raw_response']=json.dumps(payload); result['bare_json']=True
            except (ValueError,TypeError,AttributeError) as exc:
                serialization_errors.append(f"Invalid {stage} raw at {frame['timestamp']}: {exc}")
    result=evaluate_phase33(view,scenario)
    result['serialization_errors']=serialization_errors
    result['pass']=result['pass'] and not serialization_errors
    result['representation_note']='Raw codes validated strictly; canonical JSON view used only for unchanged schema/scenario checks.'
    return result


def evaluate_report(report,labels,scenario_nine,scenario_three):
    profile=report['profile']; results=[]; rows=[]; presence=direction=visible=validp=validd=0
    labels={row['timestamp']:row for row in labels['labels']}
    nine_runs=[r for r in report['runs'] if r['timestamps']==scenario_nine['timestamps']]
    main_runs=[r for r in report['runs'] if r['timestamps']==scenario_three['timestamps']]
    for run in report['runs']:
        scenario=scenario_nine if run['timestamps']==scenario_nine['timestamps'] else scenario_three
        results.append(evaluate_run(run,profile,scenario))
    for run in nine_runs:
        for frame in run['frames']:
            label=labels[frame['timestamp']]; p=frame['perception']; d=frame['planner']; obs=p.get('observation') or {}
            presence+=int(not p['fallback'] and obs.get('person_visible')==label['person_visible'])
            if label['person_visible']:
                visible+=1; direction+=int(not p['fallback'] and obs.get('person_direction')==label['person_direction'])
            validp+=int(not p['fallback']); validd+=int(not d['fallback'])
            rows.append(frame)
    count=len(rows)
    metrics={key:distribution([f['timing'].get(key) for f in rows]) for key in ['perception_wall_s','state_update_s','planner_wall_s','total_brain_s']}
    for stage in ['perception','planner']:
        metrics[stage]={key:distribution([f[stage].get('metrics',{}).get(key) for f in rows]) for key in
                       ['image_preprocessing_s','processor_s','device_transfer_s','vision_cuda_elapsed_s','vision_host_forward_s',
                        'patch_embed_cuda_elapsed_s','vision_block0_cuda_elapsed_s',
                        'prefill_cuda_elapsed_s','prefill_host_forward_s','ttft_generation_s','ttft_stage_s',
                        'decode_total_s','decode_ms_per_token','input_tokens','image_tokens','output_tokens','validation_parsing_s',
                        'peak_allocated_gib','peak_reserved_gib']}
    extended=[result for run,result in zip(report['runs'],results) if run in nine_runs]
    extended_scores=[sum(c['pass'] for c in result['checks']) for result in extended]
    main_results=[result for run,result in zip(report['runs'],results) if run in main_runs]
    integrity=all(not r['integrity_errors'] and not r['serialization_errors'] for r in results)
    main_pass=bool(main_results) and all(r['pass'] for r in main_results)
    accuracy_maintained=(count==9*len(nine_runs) and count>0 and presence==count and visible>0 and direction/visible>=.8
                         and bool(extended_scores) and min(extended_scores)>=8)
    eligible=main_pass and accuracy_maintained and integrity and validp==validd==count
    total=metrics['total_brain_s']['mean']
    peaks=[f[stage].get('metrics',{}) for f in rows for stage in ['perception','planner']]
    tier=('S' if total<=1.5 else 'A' if total<=2.5 else 'B' if total<=4 else 'C') if eligible else 'REJECT'
    return {'profile':profile,'pipeline_status':report['status'],'presence_accuracy':presence/count if count else 0,
            'direction_accuracy':direction/visible if visible else 0,'presence_correct_total':[presence,count],
            'direction_correct_total':[direction,visible], 'perception_valid_rate':validp/count if count else 0,
            'planner_valid_rate':validd/count if count else 0,'main_pass':main_pass,'extended_scores':extended_scores,
            'accuracy_maintained':accuracy_maintained,'integrity_pass':integrity,'eligible':eligible,'tier':tier,
            'peak_allocated_gib':max((p['peak_allocated_gib'] for p in peaks if p.get('peak_allocated_gib') is not None),default=None),
            'peak_reserved_gib':max((p['peak_reserved_gib'] for p in peaks if p.get('peak_reserved_gib') is not None),default=None),
            'metrics':metrics,'run_evaluations':results,'sample_note':'Warmup and model load excluded; mean/median/p95 across 9-frame runs.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports',nargs='+',type=Path)
    parser.add_argument('--output-dir',type=Path,default=Path('reports/phase34/evaluations'))
    args=parser.parse_args(); root=Path(__file__).parent
    labels=json.loads((root/'scenarios/IMG_7121_phase32_perception.json').read_text())
    nine=json.loads((root/'scenarios/IMG_7121_phase33_nine.json').read_text())
    three=json.loads((root/'scenarios/IMG_7121_phase33_three.json').read_text())
    for path in args.reports:
        report=json.loads(path.read_text()); result=evaluate_report(report,labels,nine,three)
        write_report(args.output_dir/path.name,result)
        print(json.dumps({k:result[k] for k in ['profile','presence_accuracy','direction_accuracy','main_pass','extended_scores','eligible','tier']}))
    return 0


if __name__=='__main__': raise SystemExit(main())
