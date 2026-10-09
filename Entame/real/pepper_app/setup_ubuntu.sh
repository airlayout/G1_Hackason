#!/usr/bin/env bash
# Ubuntu x86_64 / Python 3.12 setup for the Pepper app. CPU only: no CUDA, no ROS.
set -euo pipefail
cd "$(dirname "$0")"

PYTHON="${PYTHON:-python3.12}"
if [ ! -x .venv/bin/python ]; then
    "$PYTHON" -m venv .venv
fi
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-ubuntu-lock.txt
.venv/bin/python -m pip check
# YOLO11n detect + pose, pinned to Ultralytics assets v8.3.0 and verified by SHA256.
.venv/bin/python -m pepperapp.weights
.venv/bin/python -c "import torch, ultralytics, qi, streamlit; print('torch', torch.__version__, '/ ultralytics', ultralytics.__version__, '/ streamlit', streamlit.__version__)"
