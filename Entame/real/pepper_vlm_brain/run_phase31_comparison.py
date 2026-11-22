"""Reproduce Phase 3.1 benchmark schedules; no ground truth is passed to inference."""
from pathlib import Path
from config import Config
from brain.vlm import QwenVLM
from run_video import evaluate_video


def main():
    cfg=Config(max_image_side=384,max_pixels=384**2,max_new_tokens=224,combined_output=True)
    engine=QwenVLM(cfg)
    reports=[]
    for temporal in (False,True):
        name='temporal' if temporal else 'single'
        reports.append(evaluate_video(Path(r'C:\Users\slowh\Downloads\IMG_7121.mp4'),[2,6,8,10,12,16,20,24],cfg,Path(f'reports/phase31_{name}_final.json'),save_frames=True,monitor_gpu=True,engine=engine,use_memory=True,temporal=temporal))

    # Holdout timestamps only. Human presence labels live in a separate evaluator file.
    reports.append(evaluate_video(Path(r'C:\Users\slowh\Downloads\IMG_7121.mp4'),[3,7,9,9.5,10.25,13],cfg,
                   Path('reports/phase31_additional.json'),save_frames=True,monitor_gpu=True,
                   engine=engine,use_memory=False,temporal=False))
    return 0 if all(report["status"] == "pass" for report in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())
