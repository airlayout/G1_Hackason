"""監査済みの時系列cacheから36層モデルの3出力を学習し、重みと評価を保存する。"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import traceback

from g1_e2e.training.runner import Trainer, TrainingConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--device", default="cuda:1")
    parser.add_argument("--epochs", type=int, default=1)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--unroll-steps", type=int, default=8)
    parser.add_argument("--accumulation", type=int, default=4)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--initialize", type=Path, help="別の収集段階の学習済み重みから開始。optimizerは新規")
    args = parser.parse_args()
    if args.resume and args.initialize:
        parser.error("resumeとinitializeは併用できません")
    if min(args.epochs, args.batch_size, args.unroll_steps, args.accumulation) < 1:
        parser.error("epochs・unroll-steps・accumulationは正数です")
    args.output.mkdir(parents=True, exist_ok=bool(args.resume))
    config = TrainingConfig(
        epochs=args.epochs,
        batch_size=args.batch_size,
        unroll_steps=args.unroll_steps,
        accumulation=args.accumulation,
    )
    manifest = json.loads(args.manifest.read_text())
    (args.output / "run.json").write_text(
        json.dumps(
            {
                "manifest": str(args.manifest.resolve()),
                "device": args.device,
                "training": asdict(config),
                "initialize": str(args.initialize.resolve()) if args.initialize else None,
                "resume": str(args.resume.resolve()) if args.resume else None,
            },
            indent=2,
        )
        + "\n"
    )
    try:
        Trainer(
            manifest, args.output, config, device=args.device, resume=args.resume, initialize=args.initialize
        ).run()
    except Exception:
        (args.output / "failure.json").write_text(
            json.dumps({"error": traceback.format_exc()}, indent=2) + "\n"
        )
        raise


if __name__ == "__main__":
    main()
