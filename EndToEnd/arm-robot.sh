#!/usr/bin/env bash
# G1の既存Python/SDKを利用する。JetPackやシステムPyTorchを更新しない。
set -euo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARM_PYTHON="${G1_ARM_PYTHON:-python3}"
ARM_SDK="${G1_SDK_PATH:-/home/unitree/unitree_sdk2_python}"
export PYTHONPATH="$ARM_ROOT/src:$ARM_SDK${PYTHONPATH:+:$PYTHONPATH}"
exec "$ARM_PYTHON" -m g1_e2e.vla.robot "$@"
