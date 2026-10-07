"""Shared-weight serialized GPU prototype, plus explicit late-delivery dry run."""
import argparse
import json
from pathlib import Path
from time import perf_counter
from adapters.video_source import VideoSource
from adapters.robot_mock import MockRobot
from brain.split_brain import parse_flat
from brain.split_schemas import CurrentObservation, PlannerDecision
from latency.profiles import PROFILES as STABLE
from phase35.profiles import PROFILES
from phase35.dual_rate import SharedState, Stamp, execute_resolved
from phase35.codecs import observation
from run_video import write_report
from benchmark_phase35 import TIMESTAMPS


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('video',type=Path)
    parser.add_argument('--repeats',type=int,default=2)
    parser.add_argument('--output',type=Path,default=Path('reports/phase35/D.json'))
    args=parser.parse_args()
    from phase35.engine import FastQwen
    engine=FastQwen(PROFILES['A']); engine.load()
    report={'experiment':'D','mode':'serialized-shared-weight-prototype','model_load':engine.last_metrics,
            'runs':[],'status':'running','scheduler':'fast -> full semantic -> original JSON Planner; single GPU owner',
            'note':'Offline selected video frames. No claim of real-time camera rate or parallel GPU execution.'}
    with VideoSource(args.video) as source:
        frames={t:source.frame_at(t) for t in TIMESTAMPS}; report['video']=source.metadata
        def cycle(t, generation, shared, robot):
            start=perf_counter(); stamp=Stamp(frames[t].actual_timestamp,generation)
            before=shared.state.snapshot.model_dump(mode='json')
            engine.configure(PROFILES['A']); faststart=perf_counter()
            fraw,fmetrics=engine.perceive(frames[t].image); fast=observation(fraw,'class')
            shared.update_fast(fast,stamp); fastwall=perf_counter()-faststart
            # Shared-model nonpreemptible semantic job. This blocks subsequent fast work.
            engine.configure(STABLE['json_linear_uninstrumented']); semstart=perf_counter()
            sraw,smetrics=engine.perceive(frames[t].image); semantic=parse_flat(sraw,CurrentObservation)
            shared.update_semantic(semantic,stamp); semwall=perf_counter()-semstart
            combined=shared.combined(); dstart=perf_counter()
            draw,dmetrics=engine.plan(combined,shared.state.snapshot); decision=parse_flat(draw,PlannerDecision)
            resolved=execute_resolved(robot,decision,shared); dwall=perf_counter()-dstart
            return {'timestamp':t,'stamp':stamp.__dict__,'fast_raw':fraw,'fast_observation':fast.model_dump(mode='json'),
                    'fast_metrics':fmetrics,'semantic_raw':sraw,'semantic_observation':semantic.model_dump(mode='json'),
                    'semantic_metrics':smetrics,'combined_observation':combined.model_dump(mode='json'),
                    'state_before':before,'state_after':shared.state.snapshot.model_dump(mode='json'),
                    'planner_raw':draw,'planner_input':dmetrics['planner_input'],'planner_metrics':dmetrics,
                    'planner_decision':decision.model_dump(mode='json',exclude_none=True),
                    'resolved_final_decision':resolved.model_dump(mode='json',exclude_none=True) if resolved else None,
                    'mock_robot_output':robot.last_output,'timing':{'fast_s':fastwall,'semantic_s':semwall,
                    'planner_s':dwall,'cycle_s':perf_counter()-start,'fast_blocked_by_semantic_planner_s':semwall+dwall}}
        report['warmup']=cycle(3,0,SharedState(),MockRobot()); write_report(args.output,report)
        for schedule in [TIMESTAMPS]*args.repeats+[[8,10,12]]:
            shared,robot=SharedState(),MockRobot(); run={'timestamps':schedule,'frames':[]}; report['runs'].append(run)
            for index,t in enumerate(schedule):
                row=cycle(t,index,shared,robot); run['frames'].append(row); write_report(args.output,report)
                print(f"D t={t} fast={row['fast_raw']} planner={row['planner_raw']} timing={row['timing']}",flush=True)
        # Deliberately deliver an actual old semantic result after a newer fast frame.
        # Delivery order is simulated; each inference below still executes on the real GPU.
        shared=SharedState(); engine.configure(PROFILES['A'])
        left_raw,leftmetrics=engine.perceive(frames[8].image)
        shared.update_fast(observation(left_raw,'class'),Stamp(8,1))
        engine.configure(STABLE['json_linear_uninstrumented'])
        old_raw,oldmetrics=engine.perceive(frames[8].image); old=parse_flat(old_raw,CurrentObservation)
        draw,dmetrics=engine.plan(old,shared.state.snapshot); original=parse_flat(draw,PlannerDecision)
        engine.configure(PROFILES['A']); new_raw,newmetrics=engine.perceive(frames[13].image)
        shared.update_fast(observation(new_raw,'class'),Stamp(13,2))
        latest_before=shared.combined().model_dump(mode='json')
        shared.update_semantic(old,Stamp(8,1))
        robot=MockRobot(); resolved=execute_resolved(robot,original,shared)
        report['late_delivery_dry_run']={'note':'Real GPU outputs, simulated delayed result delivery; 13s prediction is not GT correctness.',
            'old_raw':old_raw,'old_metrics':oldmetrics,'initial_fast_raw':left_raw,'initial_fast_metrics':leftmetrics,
            'new_fast_raw':new_raw,'new_fast_metrics':newmetrics,'geometry_before_old_delivery':latest_before,
            'geometry_after_old_delivery':shared.combined().model_dump(mode='json'),
            'planner_raw':draw,'planner_metrics':dmetrics,'planner_decision':original.model_dump(mode='json',exclude_none=True),
            'resolved_final_decision':resolved.model_dump(mode='json',exclude_none=True) if resolved else None,
            'mock_robot_output':robot.last_output,'state':shared.state.snapshot.model_dump(mode='json')}
        report['status']='complete'; write_report(args.output,report)
    return 0


if __name__=='__main__': raise SystemExit(main())
