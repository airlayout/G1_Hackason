"""Selected MP4 frames use legacy actions or combined Observation/Decision with optional pairs."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
from time import perf_counter
from adapters.video_source import VideoSource, validate_timestamps, interval_timestamps
from adapters.robot_mock import MockRobot
from brain.decision import Brain, DecisionResult
from brain.images import prepare_image
from brain.telemetry import GpuSampler
from brain.memory import Memory
from brain.prompt import system_prompt, user_prompt, observation_system_prompt, observation_user_prompt
from config import Config, MODEL_2B, PROJECT_ROOT


def write_report(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(report, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def evaluate_video(path, timestamps, config, report_path, save_frames=False,
                   monitor_gpu=False, engine=None, use_memory=False, temporal=False):
    report = {"created_at": datetime.now(timezone(timedelta(hours=9))).isoformat(),
              "mode": "temporal selected-frame pair" if temporal else "single-image inference from selected video frames",
              "temporal_enabled": temporal, "combined_output": config.combined_output, "memory_enabled": use_memory,
              "config": {**asdict(config), "cache_dir": str(config.cache_dir)},
              "system_prompt": observation_system_prompt(temporal) if config.combined_output else system_prompt(config.prompt_profile),
              "first_frame_system_prompt": observation_system_prompt(False) if config.combined_output else None,
              "user_prompt_without_memory": observation_user_prompt(has_previous=temporal) if config.combined_output else user_prompt(None, config.prompt_profile),
              "timestamps": timestamps, "status": "running", "frames": []}
    write_report(report_path, report)
    try:
        if (use_memory or temporal) and not config.combined_output:
            raise ValueError("Memory/temporal mode requires explicit Observation output")
        with VideoSource(path) as source, GpuSampler(monitor_gpu) as sampler:
            report["video"] = source.metadata
            with source.path.open("rb") as stream:
                report["video"]["sha256"] = hashlib.file_digest(stream, "sha256").hexdigest()
            values = (interval_timestamps(source.metadata["duration_s"], 5)
                      if timestamps is None else validate_timestamps(timestamps))
            if any(t >= source.metadata["duration_s"] for t in values):
                raise ValueError("Selected timestamp is outside the video duration")
            report["timestamps"] = values
            if engine is None:
                from brain.vlm import QwenVLM
                engine = QwenVLM(config)
            brain, robot = Brain(engine, combined=config.combined_output), MockRobot()
            memory = Memory() if use_memory else None
            previous_image, previous_timestamp, previous_saved = None, None, None
            for timestamp in values:
                frame = source.frame_at(timestamp)
                saved = None
                if save_frames:
                    saved = PROJECT_ROOT / "reports" / "frames" / report_path.stem / f"{frame.frame_index:06d}_{timestamp:09.3f}s.png"
                    saved.parent.mkdir(parents=True, exist_ok=True)
                    prepare_image(frame.image, config).save(saved)
                begin = perf_counter()
                before = memory.context() if memory is not None else None
                result = brain.decide(frame.image, before, previous_image=previous_image if temporal else None)
                end = perf_counter()
                if memory is not None:
                    memory.update(result.decision, observation=result.observation, perception_valid=not result.fallback)
                entry = {"timestamp": timestamp, "previous_selected_timestamp": previous_timestamp if temporal else None,
                         "previous_saved_input_frame": previous_saved if temporal else None, "actual_timestamp": frame.actual_timestamp,
                         "frame_index": frame.frame_index, "saved_input_frame": str(saved) if saved else None,
                         "source_image_size": list(frame.image.size),
                         "memory_before": before, "memory_after": memory.context() if memory is not None else None,
                         "gpu_observation": sampler.summarize(
                             max(begin, end - result.metrics.get("inference_latency_s", end - begin)), end),
                         **result.to_dict()}
                previous_image, previous_timestamp, previous_saved = frame.image.copy(), timestamp, str(saved) if saved else None
                report["frames"].append(entry)
                write_report(report_path, report)
                print(json.dumps(entry, ensure_ascii=True), flush=True)
                robot.execute(result.decision)
            report["status"] = "pass" if all(not f["fallback"] for f in report["frames"]) else "failed"
    except KeyboardInterrupt:
        report["status"] = "interrupted"
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
        result = DecisionResult(fallback=True, error=report["error"])
        print(json.dumps(result.to_dict()), flush=True)
        MockRobot().execute(result.decision)
    finally:
        write_report(report_path, report)
    return report


def main():
    parser = argparse.ArgumentParser(description="Local video -> selected images/pairs -> VLM Observation and Decision")
    parser.add_argument("video", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--interval", type=float, default=None)
    mode.add_argument("--timestamps", help="Strictly increasing seconds, e.g. 2,6,8,10,12,16,20,24")
    parser.add_argument("--save-frames", action="store_true")
    parser.add_argument("--monitor-gpu", action="store_true")
    parser.add_argument("--memory", action="store_true", help="Pass previous validated observations to the VLM")
    parser.add_argument("--legacy", action="store_true", help="Phase 2A action-only path; split is default")
    parser.add_argument("--temporal", action="store_true", help="Previous selected image + current image, one generation")
    parser.add_argument("--observation", action="store_true", help="Combined Observation + Decision output")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--model", default=MODEL_2B)
    parser.add_argument("--revision", default=None)
    parser.add_argument("--4bit", dest="quantize_4bit", action="store_true")
    parser.add_argument("--long-edge", type=int, choices=[384, 448, 512], default=384)
    parser.add_argument("--max-new-tokens", type=int, default=None)
    parser.add_argument("--prompt-profile", choices=["legacy", "compact"], default="legacy")
    args = parser.parse_args()
    experimental = args.temporal or args.observation
    split = not args.legacy and not experimental
    if args.legacy and (args.temporal or args.observation or args.memory):
        parser.error("--legacy cannot be combined with Memory/Observation/temporal flags")
    try:
        config = Config(model_name=args.model, revision=args.revision, quantize_4bit=args.quantize_4bit,
                        max_image_side=args.long_edge, max_pixels=args.long_edge**2,
                        max_new_tokens=args.max_new_tokens or (224 if experimental else 96),
                        prompt_profile=args.prompt_profile, combined_output=experimental)
        if args.timestamps:
            timestamps = validate_timestamps(args.timestamps.split(","))
        else:
            with VideoSource(args.video) as source:
                timestamps = interval_timestamps(source.metadata["duration_s"],
                                                 args.interval if args.interval is not None else 5)
    except (ValueError, RuntimeError) as exc:
        parser.error(str(exc))
    stamp = datetime.now(timezone(timedelta(hours=9))).strftime("%Y%m%d_%H%M%S_%f")
    report_path = args.report or PROJECT_ROOT / "reports" / f"video_run_{stamp}.json"
    if split:
        from run_brain import evaluate_split_video
        report = evaluate_split_video(args.video, timestamps, config, report_path, args.save_frames, args.monitor_gpu)
    else:
        report = evaluate_video(args.video, timestamps, config, report_path, args.save_frames, args.monitor_gpu,
                                use_memory=args.memory, temporal=args.temporal)
    print(f"Report: {report_path.resolve()}", flush=True)
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
