"""学習checkpointから、全出力の重みとモデル設定だけを転送用に取り出す。"""

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess

import torch

from g1_e2e.models import ModelConfig, RecurrentPerceiver


def sha256(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def export_checkpoint(checkpoint, output, *, code_commit=None, code_dirty=None):
    checkpoint, output = Path(checkpoint), Path(output)
    if output.exists():
        raise FileExistsError(output)
    saved = torch.load(checkpoint, map_location="cpu", weights_only=True)
    config = ModelConfig.from_dict(saved["model_config"])
    model = RecurrentPerceiver(config)
    model.load_state_dict(saved["model"], assign=True)
    output.mkdir(parents=True)
    weights = output / "model.pt"
    torch.save({"model": model.state_dict(), "model_config": asdict(config)}, weights)
    digest = sha256(weights)
    manifest = {
        "format_version": 1,
        "export_code_commit": code_commit,
        "export_code_dirty": code_dirty,
        "source_checkpoint": checkpoint.name,
        "source_checkpoint_sha256": sha256(checkpoint),
        "training_update": saved.get("update"),
        "parameters": sum(parameter.numel() for parameter in model.parameters()),
        "model_config": asdict(config),
        "weights": {"file": weights.name, "bytes": weights.stat().st_size, "sha256": digest},
        "precision": "source tensor dtypes preserved; no quantization",
        "contents": "all three output heads; optimizer, dataset paths and recurrent runtime state excluded",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    (output / "SHA256SUMS").write_text(
        f"{digest}  model.pt\n{sha256(output / 'manifest.json')}  manifest.json\n"
    )
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("--output", required=True, type=Path, help="未作成の転送用ディレクトリ")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=root, check=True, capture_output=True, text=True
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain", "--", str(root)],
                cwd=root,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        commit = dirty = None
    manifest = export_checkpoint(args.checkpoint, args.output, code_commit=commit, code_dirty=dirty)
    print(json.dumps({"output": str(args.output.resolve()), **manifest}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
