"""Phase 3.2 default video path: current-image Perception -> State -> text Planner."""
from pathlib import Path
from dataclasses import asdict
from time import perf_counter
from adapters.video_source import VideoSource, validate_timestamps
from adapters.robot_mock import MockRobot
from brain.split_brain import SplitBrain
from brain.split_schemas import PlannerDecision
from brain.split_prompt import PERCEPTION_SYSTEM, PERCEPTION_USER, PLANNER_SYSTEM
from brain.images import prepare_image
from brain.telemetry import GpuSampler
from run_video import write_report


def evaluate_split_video(video, timestamps, config, report_path, save_frames=False, monitor_gpu=False, engine=None):
    report={'mode':'perception-state-planner', 'timestamps':validate_timestamps(timestamps),
            'config':{**asdict(config),'cache_dir':str(config.cache_dir)},
            'perception_system_prompt':PERCEPTION_SYSTEM,'perception_user_prompt':PERCEPTION_USER,
            'planner_system_prompt':PLANNER_SYSTEM,'status':'running','frames':[]}
    write_report(report_path,report)
    try:
        with VideoSource(video) as source, GpuSampler(monitor_gpu) as sampler:
            report['video']=source.metadata
            if engine is None:
                from brain.split_vlm import SplitQwenVLM
                engine=SplitQwenVLM(config)
            if hasattr(engine,'load'):
                begin=perf_counter();engine.load();end=perf_counter()
                report['model_load']={'status':'pass',**engine.last_metrics,'driver_observation':sampler.summarize(begin,end)}
            brain,robot=SplitBrain(engine),MockRobot()
            for timestamp in report['timestamps']:
                frame=source.frame_at(timestamp);saved=None
                if save_frames:
                    saved=Path('reports/frames')/report_path.stem/f'{timestamp:07.3f}s.png'
                    saved.parent.mkdir(parents=True,exist_ok=True);prepare_image(frame.image,config).save(saved)
                begin=perf_counter();result=brain.decide(frame.image);end=perf_counter()
                entry={'timestamp':timestamp,'actual_timestamp':frame.actual_timestamp,'frame_index':frame.frame_index,
                       'saved_input_frame':str(saved.resolve()) if saved else None,
                       'gpu_observation':sampler.summarize(begin,end),**result}
                entry['transition']=result['state_after']['transition']
                robot.execute(PlannerDecision.model_validate(result['planner']['decision']))
                entry['mock_robot_output']=robot.last_output
                report['frames'].append(entry);write_report(report_path,report)
                print(entry,flush=True)
            report['status']='pass' if all(not f['perception']['fallback'] and not f['planner']['fallback'] for f in report['frames']) else 'failed'
    except Exception as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
    finally:
        write_report(report_path,report)
    return report
