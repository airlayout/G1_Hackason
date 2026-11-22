"""No-label GPU benchmark. Outputs are evaluated separately after each run."""
import argparse
import hashlib
from dataclasses import replace
from pathlib import Path
from time import perf_counter
from adapters.robot_mock import MockRobot
from adapters.video_source import VideoSource
from brain.split_schemas import PlannerDecision
from latency.profiles import PROFILES
from latency.runtime import LatencyBrain
from run_video import write_report

TIMESTAMPS=[3,7,8,9,9.5,10,10.25,12,13]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video',type=Path)
    parser.add_argument('--profiles',default='json')
    parser.add_argument('--repeats',type=int,default=2)
    parser.add_argument('--dtype',choices=['bf16','fp16'],default='bf16')
    parser.add_argument('--attention',choices=['default','flash','efficient','efficient_expanded'],default='default')
    parser.add_argument('--suffix',default='')
    parser.add_argument('--output-dir',type=Path,default=Path('reports/phase34'))
    args=parser.parse_args()
    if args.repeats<1: parser.error('Positive repeats required')
    profiles=[replace(PROFILES[name],dtype=args.dtype,attention=args.attention) for name in args.profiles.split(',')]
    from latency.engine import LatencyQwen
    engine=LatencyQwen(profiles[0]); loadstart=perf_counter(); engine.load(); loadwall=perf_counter()-loadstart
    with VideoSource(args.video) as source:
        frames={t:source.frame_at(t) for t in TIMESTAMPS}
        for profile in profiles:
            engine.configure(profile)
            report={'mode':'phase34-experimental','profile':profile.to_dict(),'model_load':{**engine.last_metrics,'load_wall_s':loadwall},
                    'single_token_codes':engine.code_tokens,'video':source.metadata,'status':'running','runs':[],
                    'prompts':{stage:profile.prompt(stage) for stage in ['perception','planner']}}
            path=args.output_dir/(profile.name+args.suffix+'.json'); write_report(path,report)
            warmup=LatencyBrain(engine).decide(frames[TIMESTAMPS[0]].image)
            report['warmup']=warmup; write_report(path,report)
            schedules=[TIMESTAMPS]*args.repeats+[[8,10,12]]
            for index,schedule in enumerate(schedules):
                brain,robot=LatencyBrain(engine),MockRobot()
                run={'mode':'perception-state-planner','timestamps':schedule,'status':'running','frames':[]}
                report['runs'].append(run)
                for timestamp in schedule:
                    frame=frames[timestamp]; result=brain.decide(frame.image)
                    robot.execute(PlannerDecision.model_validate(result['planner']['decision']))
                    entry={'timestamp':timestamp,'actual_timestamp':frame.actual_timestamp,'frame_index':frame.frame_index,
                           'input_pixels_sha256':hashlib.sha256(frame.image.tobytes()).hexdigest(),
                           'mock_robot_output':robot.last_output,**result}
                    run['frames'].append(entry); write_report(path,report)
                    print(f"{profile.name}{args.suffix} run={index} t={timestamp} "
                          f"P={result['perception']['raw_response']!r} D={result['planner']['raw_response']!r} "
                          f"brain={result['timing']['total_brain_s']:.3f}s",flush=True)
                run['status']='pass' if all(not f[k]['fallback'] for f in run['frames'] for k in ['perception','planner']) else 'failed'
                write_report(path,report)
            report['status']='pass' if all(r['status']=='pass' for r in report['runs']) else 'failed'
            write_report(path,report)
    return 0


if __name__=='__main__': raise SystemExit(main())
