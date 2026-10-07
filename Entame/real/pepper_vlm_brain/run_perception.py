"""Phase 3.2 image-only Perception benchmark. No annotations or Memory dependency."""
import argparse
from dataclasses import asdict
from pathlib import Path
from adapters.video_source import VideoSource, validate_timestamps
from brain.images import prepare_image
from brain.split_brain import Perception
from brain.split_prompt import PERCEPTION_SYSTEM, PERCEPTION_USER
from brain.telemetry import GpuSampler
from config import Config, MODEL_2B, MODEL_4B
from run_video import write_report
from time import perf_counter


def evaluate_perception(video, timestamps, config, report_path, engine=None):
    report={'mode':'current-image-only perception', 'config':{**asdict(config),'cache_dir':str(config.cache_dir)},
            'system_prompt':PERCEPTION_SYSTEM, 'user_prompt':PERCEPTION_USER,
            'timestamps':validate_timestamps(timestamps),'status':'running','frames':[]}
    write_report(report_path,report)
    try:
        with VideoSource(video) as source, GpuSampler(True) as sampler:
            if engine is None:
                from brain.split_vlm import SplitQwenVLM
                engine=SplitQwenVLM(config)
            begin=perf_counter();engine.load();end=perf_counter()
            report['model_load']={'status':'pass',**engine.last_metrics,'driver_observation':sampler.summarize(begin,end)}
            perception=Perception(engine)
            report['video']=source.metadata
            for timestamp in report['timestamps']:
                frame=source.frame_at(timestamp)
                saved=Path('reports/frames')/report_path.stem/f'{timestamp:07.3f}s.png'
                saved.parent.mkdir(parents=True,exist_ok=True)
                prepare_image(frame.image,config).save(saved)
                begin=perf_counter();result=perception.observe(frame.image);end=perf_counter()
                entry={'timestamp':timestamp,'actual_timestamp':frame.actual_timestamp,'frame_index':frame.frame_index,
                       'saved_input_frame':str(saved.resolve()),'source_image_size':list(frame.image.size),
                       'gpu_observation':sampler.summarize(begin,end),**result.to_dict()}
                report['frames'].append(entry);write_report(report_path,report)
                print(entry,flush=True)
            report['status']='pass' if all(not f['fallback'] for f in report['frames']) else 'failed'
    except Exception as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
        if 'model_load' not in report:
            report['model_load']={'status':'failed',**getattr(engine,'last_metrics',{}),'error':report['error']}
        print(report['error'],flush=True)
    finally:
        write_report(report_path,report)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video',type=Path)
    parser.add_argument('--timestamps',default='3,7,8,9,9.5,10,10.25,12,13')
    parser.add_argument('--model',choices=[MODEL_2B,MODEL_4B],default=MODEL_2B)
    parser.add_argument('--4bit',dest='nf4',action='store_true')
    parser.add_argument('--long-edge',type=int,choices=[384,448,512],default=384)
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    config=Config(model_name=args.model,quantize_4bit=args.nf4,max_image_side=args.long_edge,
                  max_pixels=args.long_edge**2,max_new_tokens=96)
    result=evaluate_perception(args.video,args.timestamps.split(','),config,args.report)
    return 0 if result['status']=='pass' else 1

if __name__=='__main__': raise SystemExit(main())
