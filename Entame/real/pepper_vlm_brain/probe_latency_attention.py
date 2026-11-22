"""Read actual SDPA operator choices without installing or changing packages."""
import argparse
from dataclasses import replace
from pathlib import Path
import torch
from collections import Counter
from torch.utils._python_dispatch import TorchDispatchMode
from adapters.video_source import VideoSource
from latency.engine import LatencyQwen
from latency.profiles import PROFILES
from latency.runtime import LatencyBrain
from run_video import write_report


class OperatorTrace(TorchDispatchMode):
    """Observe dispatched ATen operators; no CUPTI collector or environment changes."""
    def __init__(self): super().__init__(); self.counts=Counter(); self.backends=Counter()
    def __torch_dispatch__(self,func,types,args=(),kwargs=None):
        self.counts[str(func)]+=1
        if str(func)=='aten.scaled_dot_product_attention.default':
            backend=torch.ops.aten._fused_sdp_choice.default(*args,**(kwargs or {}))
            self.backends[(int(backend),str(args[0].dtype),tuple(args[0].shape))]+=1
        return func(*args,**(kwargs or {}))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video',type=Path)
    parser.add_argument('--attention',choices=['default','flash','efficient','efficient_expanded'],default='default')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report={'attention':args.attention,'status':'running','note':'ATen dispatch probe excluded from latency samples; no CUPTI.'}
    write_report(args.output,report)
    try:
        profile=replace(PROFILES['class_constrained'],attention=args.attention)
        engine=LatencyQwen(profile); engine.load()
        report['environment']={'torch':torch.__version__,'cuda':torch.version.cuda,'gpu':torch.cuda.get_device_name(),
                               'cudnn_version':torch.backends.cudnn.version(),'cudnn_benchmark':torch.backends.cudnn.benchmark,
                               'flash_enabled':torch.backends.cuda.flash_sdp_enabled(),
                               'flash_compiled_available':torch.backends.cuda.is_flash_attention_available(),
                               'math_enabled':torch.backends.cuda.math_sdp_enabled(),
                               'mem_efficient_enabled':torch.backends.cuda.mem_efficient_sdp_enabled(),
                               'vision_attn':engine.model.config.vision_config._attn_implementation,
                               'text_attn':engine.model.config.text_config._attn_implementation}
        with VideoSource(args.video) as source: image=source.frame_at(3).image
        report['warmup']=LatencyBrain(engine).decide(image)
        write_report(args.output,report)
        with OperatorTrace() as trace:
            report['sample']=LatencyBrain(engine).decide(image)
        rows=[{'name':name,'count':count} for name,count in trace.counts.items()]
        report['operators']=rows
        names={-1:'ERROR',0:'MATH',1:'FLASH_ATTENTION',2:'EFFICIENT_ATTENTION',3:'CUDNN_ATTENTION',4:'OVERRIDEABLE'}
        report['selected_backends']=[{'id':key[0],'name':names.get(key[0],'UNKNOWN'),'dtype':key[1],'query_shape':key[2],'count':count}
                                     for key,count in trace.backends.items()]
        report['sdpa_operators']=[e for e in rows if 'attention' in e['name'].lower() or 'flash' in e['name'].lower()]
        report['status']='pass' if not any(report['sample'][s]['fallback'] for s in ['perception','planner']) else 'failed'
    except Exception as exc: report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
    finally: write_report(args.output,report)
    print(report.get('sdpa_operators',report.get('error')),flush=True)
    return 0 if report['status']=='pass' else 1


if __name__=='__main__': raise SystemExit(main())
