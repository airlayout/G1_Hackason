import argparse
import json
from pathlib import Path
from PIL import Image
from brain.decision import Brain, DecisionResult
from adapters.robot_mock import MockRobot
from config import Config, MODEL_2B


def main():
    parser = argparse.ArgumentParser(description="One image -> one validated high-level action")
    parser.add_argument("image", type=Path)
    parser.add_argument("--model", default=MODEL_2B)
    parser.add_argument("--revision", default=None)
    parser.add_argument("--4bit", dest="quantize_4bit", action="store_true")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--prompt-profile", choices=["legacy", "compact"], default="legacy")
    parser.add_argument("--repeat", type=int, default=1, help="Reuse model to measure warm inference")
    args = parser.parse_args()
    if args.repeat < 1 or args.repeat > 20:
        parser.error("--repeat must be 1..20")
    runs = []
    try:
        from brain.vlm import QwenVLM
        config = Config(model_name=args.model, revision=args.revision,
                        quantize_4bit=args.quantize_4bit, prompt_profile=args.prompt_profile)
        brain = Brain(QwenVLM(config))
        with Image.open(args.image) as image:
            for _ in range(args.repeat):
                result = brain.decide(image)
                runs.append(result.to_dict())
                if result.fallback:
                    break
    except Exception as exc:
        result = DecisionResult(fallback=True, error=f"Startup/image failed: {type(exc).__name__}: {exc}")
    print("Decision:")
    print(result.decision.model_dump_json(indent=2, exclude_none=True))
    print("Debug:")
    report = result.to_dict()
    if args.repeat > 1:
        report["runs"] = runs
    print(json.dumps(report, indent=2, ensure_ascii=True))
    MockRobot().execute(result.decision)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
    return 1 if result.fallback else 0


if __name__ == "__main__":
    raise SystemExit(main())
