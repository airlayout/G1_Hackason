"""Real GPU patch output/timing comparison; no labels and no altered model weights."""
import argparse
from pathlib import Path
from time import perf_counter
import torch
from adapters.video_source import VideoSource
from brain.images import prepare_image
from latency.engine import LatencyQwen
from latency.profiles import PROFILES
from latency.patch_linear import linear_patch_forward
from run_video import write_report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video',type=Path); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); report={'status':'running'}
    try:
        engine=LatencyQwen(PROFILES['json']); engine.load()
        with VideoSource(args.video) as source: image=prepare_image(source.frame_at(3).image,engine.config)
        system,user=engine.profile.prompt('perception')
        messages=[{'role':'system','content':[{'type':'text','text':system}]},
                  {'role':'user','content':[{'type':'image','image':image},{'type':'text','text':user}]}]
        engine.processor.image_processor.size={'shortest_edge':engine.config.min_pixels,'longest_edge':engine.config.max_pixels}
        inputs=engine.processor.apply_chat_template(messages,tokenize=True,add_generation_prompt=True,return_dict=True,return_tensors='pt').to('cuda:0')
        module=engine.model.model.visual.patch_embed; pixels=inputs.pixel_values
        times=[]
        with torch.inference_mode():
            for repeat in range(3):
                torch.cuda.synchronize(); start=perf_counter(); original=module(pixels); torch.cuda.synchronize(); conv_s=perf_counter()-start
                start=perf_counter(); linear=linear_patch_forward(module,pixels); torch.cuda.synchronize(); linear_s=perf_counter()-start
                times.append({'repeat':repeat,'conv3d_s':conv_s,'linear_s':linear_s})
            error=(original.float()-linear.float()).abs()
            report.update(dtype=str(original.dtype),shape=list(original.shape),timing=times,
                          max_abs_error=error.max().item(),rmse=(error.square().mean().sqrt()).item(),
                          exact_fraction=(original==linear).float().mean().item(),weight_instance_id=id(module.proj.weight))
            torch.testing.assert_close(original,linear,rtol=.02,atol=.02)
        report['status']='pass'
    except Exception as exc: report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
    finally: write_report(args.output,report)
    print(report,flush=True)
    return 0 if report['status']=='pass' else 1


if __name__=='__main__': raise SystemExit(main())
