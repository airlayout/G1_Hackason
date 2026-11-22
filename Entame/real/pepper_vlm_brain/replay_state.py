"""Replay validated Perception outputs through State only; never invent a Decision."""
import argparse
import json
from pathlib import Path
from time import perf_counter
from brain.split_brain import parse_flat
from brain.split_schemas import CurrentObservation
from brain.state import State
from run_video import write_report
from adapters.video_source import validate_timestamps


def replay(report, timestamps):
    state=State();rows=[]
    for timestamp in validate_timestamps(timestamps):
        found=[f for f in report['frames'] if f['timestamp']==timestamp]
        if len(found)!=1: raise ValueError('Missing/duplicate source frame')
        frame=found[0];before=state.snapshot;obs=None;error=None
        try:
            obs=parse_flat(frame['raw_response'],CurrentObservation)
            if frame['fallback'] or frame.get('observation')!=obs.model_dump(mode='json'):
                raise ValueError('Raw/Observation mismatch')
        except (ValueError,TypeError) as exc: obs=None;error=str(exc)
        start=perf_counter();after=state.update(obs) if obs is not None else before;elapsed=perf_counter()-start
        rows.append({'timestamp':timestamp,'observation':obs.model_dump(mode='json') if obs else None,
                     'state_before':before.model_dump(mode='json'),'state_after':after.model_dump(mode='json'),
                     'state_update_s':elapsed,'error':error})
    return {'mode':'saved-observation state-only replay','planner_executed':False,
            'note':'No new Perception inference; no Decision is generated or substituted.',
            'frames':rows,'status':'pass' if all(f['error'] is None for f in rows) else 'failed'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path);parser.add_argument('--timestamps',default='8,10,12');parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=replay(json.loads(args.report.read_text(encoding='utf-8')),args.timestamps.split(','))
    write_report(args.output,result);print(json.dumps(result,indent=2));return 0 if result['status']=='pass' else 1

if __name__=='__main__': raise SystemExit(main())
