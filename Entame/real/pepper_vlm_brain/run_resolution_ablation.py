"""Reproduce the pre-architecture ablation; fixed legacy context is evaluation input only."""
import json, hashlib
from pathlib import Path
from adapters.video_source import VideoSource
from brain.vlm import QwenVLM
from brain.decision import Brain
from config import Config


def main():
    baseline=json.loads(Path('reports/video_IMG_7121_memory384_legacy.json').read_text())
    context=next(f['memory_before'] for f in baseline['frames'] if f['timestamp']==10)
    with VideoSource(Path(r'C:\Users\slowh\Downloads\IMG_7121.mp4')) as source:
        image=source.frame_at(10).image
    engine=QwenVLM()
    rows=[]
    for edge in (384,448,512):
        engine.config=Config(max_image_side=edge,max_pixels=edge*edge)
        engine.load()
        engine.processor.image_processor.size={'shortest_edge':4096,'longest_edge':edge*edge}
        result=Brain(engine).decide(image,context)
        row={'edge':edge,'person_visible_legacy_proxy':result.decision.target.value=='PERSON' and result.decision.action.value!='SEARCH',**result.to_dict()}
        rows.append(row)
        print(json.dumps(row),flush=True)
        Path('reports/phase31_resolution_ablation.json').write_text(json.dumps({'source_pixel_sha256':hashlib.sha256(image.tobytes()).hexdigest(),'timestamp_s':10,'memory_context':context,'visibility_note':'Legacy Decision-derived proxy; not an Observation measurement','results':rows},indent=2),encoding='utf-8')

    return 1 if any(row["fallback"] for row in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
