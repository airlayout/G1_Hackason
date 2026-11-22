#!/usr/bin/env bash
# Ubuntu x86_64 / Python 3.12 / NVIDIA GPU setup. Windows uses setup_*.ps1 instead.
# Usage: ./setup_ubuntu.sh [--models 2b|4b|all]
#   --models also downloads the pinned Qwen3-VL revisions from config.py.
#   Without it, the 2B model is downloaded on first QwenVLM.load().
set -euo pipefail
cd "$(dirname "$0")"

MODELS=""
if [ "${1:-}" = "--models" ]; then
    MODELS="${2:?--models needs 2b, 4b or all}"
fi
PYTHON="${PYTHON:-python3.12}"

if [ ! -x .venv/bin/python ]; then
    "$PYTHON" -m venv .venv
fi
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-ubuntu-lock.txt
.venv/bin/python -m pip check

.venv/bin/python - <<'EOF'
import torch
print(f"torch {torch.__version__} / CUDA {torch.version.cuda} / available={torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"GPU {torch.cuda.get_device_name(0)} / capability {torch.cuda.get_device_capability(0)}")
EOF

# Same YOLO11n pin and SHA256 as setup_hybrid.ps1. A mismatched download is not kept.
.venv/bin/python - <<'EOF'
from pathlib import Path
import hashlib
import urllib.request

target = Path(".cache/phase36/yolo11n.pt")
expected = "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1"
target.parent.mkdir(parents=True, exist_ok=True)
if not target.exists():
    partial = target.with_suffix(".pt.partial")
    urllib.request.urlretrieve(
        "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt", partial)
    if hashlib.sha256(partial.read_bytes()).hexdigest() != expected:
        partial.unlink()
        raise SystemExit("YOLO weight hash mismatch")
    partial.replace(target)
if hashlib.sha256(target.read_bytes()).hexdigest() != expected:
    raise SystemExit("YOLO weight hash mismatch")
print(f"Verified {target}")
EOF

# Replaces download_model4b.py, which only runs on Windows (curl.exe).
if [ -n "$MODELS" ]; then
    .venv/bin/python - "$MODELS" <<'EOF'
import sys
from huggingface_hub import snapshot_download
from config import Config, MODEL_2B, MODEL_4B

choice = sys.argv[1]
names = {"2b": [MODEL_2B], "4b": [MODEL_4B], "all": [MODEL_2B, MODEL_4B]}.get(choice)
if names is None:
    raise SystemExit(f"--models must be 2b, 4b or all; got {choice}")
for name in names:
    config = Config(model_name=name, quantize_4bit=name == MODEL_4B)
    path = snapshot_download(name, revision=config.model_revision,
                             cache_dir=str(config.cache_dir),
                             allow_patterns=["*.json", "*.txt", "*.safetensors"])
    print(f"Downloaded {name}@{config.model_revision} -> {path}")
EOF
fi
