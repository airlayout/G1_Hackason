"""Real-GPU, label-free Phase 3.5 benchmarks. Evaluation is a separate command."""
import argparse
import hashlib
import json
from pathlib import Path
from time import perf_counter
from adapters.robot_mock import MockRobot
from adapters.video_source import VideoSource
from brain.split_brain import PerceptionResult
from brain.split_schemas import PlannerDecision
from brain.state import State
from phase35 import codecs
from phase35.profiles import PROFILES
from phase35.runtime import LatencyBrain
from run_video import write_report

TIMESTAMPS = [3,7,8,9,9.5,10,10.25,12,13]


def geometry(engine, image, state):
    start = perf_counter(); before = state.snapshot; p = PerceptionResult()
    try:
        p.raw_response, p.metrics = engine.perceive(image)
        if p.metrics.get('class_logits_finite') is False:
            raise ValueError('Nonfinite scene logits')
        validation = perf_counter()
        p.observation = codecs.observation(p.raw_response, 'class')
        p.metrics['validation_parsing_s'] = perf_counter()-validation
    except Exception as exc:
        p.fallback = True; p.error = f'{type(exc).__name__}: {exc}'
        if not p.metrics: p.metrics = engine.failure_metrics('perception')
    ptime = perf_counter()-start; sstart = perf_counter()
    after = state.update(p.observation) if not p.fallback else before
    return {'perception':p.to_dict(), 'state_before':before.model_dump(mode='json'),
            'state_after':after.model_dump(mode='json'), 'transition':after.transition.value,
            'timing':{'perception_wall_s':ptime, 'state_update_s':perf_counter()-sstart,
                      'total_geometry_s':perf_counter()-start}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', type=Path)
    parser.add_argument('--experiments', default='A,B')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--output-dir', type=Path, default=Path('reports/phase35'))
    args = parser.parse_args()
    if args.repeats < 2: parser.error('At least two measured repeats required')
    names = args.experiments.split(',')
    for name in names:
        if name.startswith('C'):
            partner='B2' if name=='C2' else 'B'
            gates = [json.loads((args.output_dir/'evaluations'/f'{n}.json').read_text()) for n in ['A',partner]]
            if not all(g['eligible'] for g in gates):
                parser.error('C requires independently evaluated A and B accuracy gates PASS')
    from phase35.engine import FastQwen
    engine = FastQwen(PROFILES[names[0]]); start = perf_counter(); engine.load()
    load = {**engine.last_metrics, 'load_wall_s':perf_counter()-start}
    with VideoSource(args.video) as source:
        frames = {t:source.frame_at(t) for t in TIMESTAMPS}
        for name in names:
            engine.configure(PROFILES[name]); path = args.output_dir/f'{name}.json'
            report = {'experiment':name, 'mode':'phase35-experimental', 'profile':engine.profile.to_dict(),
                      'model_load':load, 'single_token_codes':engine.code_tokens,
                      'video':source.metadata, 'status':'running', 'runs':[],
                      'prompts':{s:engine.profile.prompt(s) for s in ['perception','planner']},
                      'distance_binding':'LOOK/FOUND/APPROACH copy supplied Observation distance only; action/direction/target remain chosen code'}
            report['warmup'] = (geometry(engine,frames[3].image,State()) if name=='A'
                                else LatencyBrain(engine).decide(frames[3].image))
            write_report(path,report)
            for index, schedule in enumerate([TIMESTAMPS]*args.repeats+[[8,10,12]]):
                state, brain, robot = State(), LatencyBrain(engine), MockRobot()
                run = {'mode':'geometry-only' if name=='A' else 'perception-state-planner',
                       'timestamps':schedule, 'status':'running', 'frames':[]}
                report['runs'].append(run)
                for t in schedule:
                    frame = frames[t]
                    result = (geometry(engine,frame.image,state) if name=='A' else brain.decide(frame.image))
                    if name!='A':
                        robot.execute(PlannerDecision.model_validate(result['planner']['decision']))
                        result['mock_robot_output'] = robot.last_output
                        result['resolved_final_decision'] = result['planner']['decision']
                    run['frames'].append({'timestamp':t,'actual_timestamp':frame.actual_timestamp,
                        'frame_index':frame.frame_index,
                        'input_pixels_sha256':hashlib.sha256(frame.image.tobytes()).hexdigest(), **result})
                    write_report(path,report)
                    print(f"{name} run={index} t={t} P={result['perception']['raw_response']!r} "
                          f"D={result.get('planner',{}).get('raw_response')} time={result['timing']}",flush=True)
                run['status']='complete'
            report['status']='complete'; write_report(path,report)
    return 0


if __name__=='__main__': raise SystemExit(main())
