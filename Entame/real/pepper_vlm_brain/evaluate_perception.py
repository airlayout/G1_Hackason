"""Post-run perception evaluation. Human labels are never imported by inference."""
import argparse
import hashlib
import json
import statistics
from pathlib import Path
from brain.split_brain import parse_flat, bare_json
from brain.split_schemas import CurrentObservation
from brain.split_prompt import PERCEPTION_SYSTEM, PERCEPTION_USER
from run_video import write_report
from config import MODEL_2B, MODEL_4B


def evaluate(report, annotations):
    labels=annotations['labels'];frames=report.get('frames',[]);errors=[]
    schedule=[x['timestamp'] for x in labels]
    if report.get('timestamps')!=schedule or [f['timestamp'] for f in frames]!=schedule:
        errors.append('Schedule mismatch')
    if report.get('system_prompt')!=PERCEPTION_SYSTEM or report.get('user_prompt')!=PERCEPTION_USER:
        errors.append('Fixed perception prompt mismatch')
    expected_hash=hashlib.sha256((PERCEPTION_SYSTEM+PERCEPTION_USER).encode()).hexdigest()
    rows=[]
    for label in labels:
        matches=[f for f in frames if f['timestamp']==label['timestamp']]
        if len(matches)!=1:
            rows.append({**label,'valid':False,'presence_correct':False,'direction_correct':False,'error':'Missing/duplicate frame'})
            continue
        f=matches[0];obs=None;error=None
        try: obs=parse_flat(f['raw_response'],CurrentObservation)
        except (ValueError,TypeError) as exc: error=str(exc)
        valid=obs is not None
        if valid and (f.get('fallback') or f.get('observation')!=obs.model_dump(mode='json')):
            errors.append(f"Raw/Observation mismatch at {f['timestamp']}")
        if not valid and (not f.get('fallback') or f.get('observation') is not None):
            errors.append(f"Invalid output accepted at {f['timestamp']}")
        m=f.get('metrics',{})
        if m.get('stage')!='perception' or m.get('image_count')!=1 or m.get('prompt_sha256')!=expected_hash:
            errors.append(f"Perception input contract mismatch at {f['timestamp']}")
        rows.append({**label,'valid':valid,'bare_json':bare_json(f['raw_response']),
                     'predicted_visible':obs.person_visible if obs else None,
                     'predicted_direction':obs.person_direction.value if obs else None,
                     'presence_correct':bool(obs and obs.person_visible==label['person_visible']),
                     'direction_correct':bool(obs and obs.person_direction.value==label['person_direction']),
                     'error':error})
    metrics=[f['metrics'] for f in frames if f.get('metrics',{}).get('inference_latency_s') is not None]
    negatives=[x for x in rows if x['subset']=='additional' and not x['person_visible']]
    positives=[x for x in rows if x['person_visible']]
    n=len(rows)
    result={'integrity_errors':errors,'sample_count':n,'schema_valid_rate':sum(x['valid'] for x in rows)/n,
            'bare_json_rate':sum(x.get('bare_json',False) for x in rows)/n,
            'presence_accuracy_including_invalid_as_failure':sum(x['presence_correct'] for x in rows)/n,
            'direction_accuracy_all':sum(x['direction_correct'] for x in rows)/n,
            'direction_accuracy_visible_only':sum(x['direction_correct'] for x in positives)/len(positives),
            'additional_absence_correct':sum(x['presence_correct'] for x in negatives),
            'additional_absence_samples':len(negatives),
            'additional_false_positives':sum(x.get('predicted_visible') is True for x in negatives),
            'additional_invalid_absences':sum(not x['valid'] for x in negatives),
            'mean_latency_s':statistics.mean(x['inference_latency_s'] for x in metrics) if metrics else None,
            'input_tokens':[x['input_tokens'] for x in metrics], 'output_tokens':[x['output_tokens'] for x in metrics],
            'image_tokens':[x['image_tokens'] for x in metrics],
            'peak_allocated_gib':max((x['peak_allocated_gib'] for x in metrics),default=None),
            'peak_reserved_gib':max((x['peak_reserved_gib'] for x in metrics),default=None),
            'model_load':report.get('model_load'), 'rows':rows}
    result['pass']=not errors and all(x['valid'] and x['presence_correct'] and x['direction_correct'] for x in rows)
    return result


def compare(two, four, annotations):
    t,f=evaluate(two,annotations),evaluate(four,annotations)
    # Gate only usability/absence improvement. This evaluator never selects Actions.
    ten=next((x for x in f['rows'] if x['timestamp']==10),{})
    peaks=[f.get('peak_allocated_gib'),f.get('peak_reserved_gib'),
           (four.get('model_load') or {}).get('model_load_peak_allocated_gib'),
           (four.get('model_load') or {}).get('model_load_peak_reserved_gib')]
    within_vram=all(x is not None and x<8 for x in peaks)
    model_load=four.get('model_load') or {}
    model_ok=(model_load.get('loaded_in_4bit') is True and model_load.get('nf4_linear_layers',0)>0
              and four.get('config',{}).get('model_name')==MODEL_4B
              and four.get('config',{}).get('quantize_4bit') is True
              and model_load.get('gpu_used') is True)
    usable_improved=f['additional_absence_correct']>t['additional_absence_correct']
    gate=not t['integrity_errors'] and two.get('config',{}).get('model_name')==MODEL_2B and two.get('config',{}).get('quantize_4bit') is False and not f['integrity_errors'] and model_ok and within_vram and f['schema_valid_rate']==1 and usable_improved and f['additional_absence_correct']==3 and ten.get('presence_correct') is True
    return {'2b':t,'4b':f,'planner_gate':{'pass':bool(gate),'model_load_nf4':model_ok,'within_8gib':within_vram,
             'usable_absence_improved':usable_improved,'note':'Improvement means validated usable absent Observations; invalid 2B JSON is not counted as a false positive or repaired. Raw 2B may already describe absence.'}}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('two',type=Path);parser.add_argument('four',type=Path)
    parser.add_argument('annotations',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=compare(*(json.loads(p.read_text(encoding='utf-8')) for p in (args.two,args.four,args.annotations)))
    write_report(args.output,result);print(json.dumps(result['planner_gate'],indent=2))
    return 0 if result['planner_gate']['pass'] else 1

if __name__=='__main__': raise SystemExit(main())
