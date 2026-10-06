"""Matched frozen timestamps; no ground-truth reads in inference."""
import argparse
from pathlib import Path
from time import perf_counter,sleep
import numpy as np
from adapters.video_source import VideoSource
from hybrid.process import WorkerProcess
from run_video import write_report
from benchmark_phase35 import TIMESTAMPS


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('video',type=Path)
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--confidence',type=float,default=.25)
    parser.add_argument('--backends',default='yolo,qwen')
    parser.add_argument('--output-dir',type=Path,default=Path('reports/phase36/selected'))
    args=parser.parse_args()
    if not 0<args.confidence<=1: parser.error('Confidence must be in (0,1]')
    with VideoSource(args.video) as source:
        frames={t:source.frame_at(t) for t in TIMESTAMPS}
        for backend in args.backends.split(','):
            worker=WorkerProcess(backend,Path('.cache/phase36/yolo11n.pt'),confidence=args.confidence); info=worker.ready()
            report={'backend':backend,'worker':info,'video':source.metadata,'runs':[],'status':'running'}
            path=args.output_dir/f'{backend}.json'
            try:
                schedules=[TIMESTAMPS]*args.repeats+[[8,10,12]]
                for schedule in schedules:
                    rows=[]; report['runs'].append({'timestamps':schedule,'results':rows})
                    for timestamp in schedule:
                        f=frames[timestamp]; capture=perf_counter()
                        worker.submit({'kind':'fast','generation':f.frame_index,'source_timestamp':f.actual_timestamp,
                            'capture_timestamp':capture,'bgr':np.asarray(f.image)[:,:,::-1].copy()})
                        result=None
                        while result is None: result=worker.poll(); sleep(.001)
                        result['requested_timestamp']=timestamp; rows.append(result); write_report(path,report)
                        print(backend,timestamp,result['observation'],result['inference_latency_s'],flush=True)
                report['status']='complete'; write_report(path,report)
            finally: worker.close()


if __name__=='__main__': main()
