#!/usr/bin/env bash
# Orinへ渡すファイルだけをまとめる。モデル・仮想環境・Isaac資産は含めない。
set -euo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARM_DEST="$ARM_ROOT/assets/cache/vla/g1-arm-bridge.tar.gz"
mkdir -p "$(dirname "$ARM_DEST")"
tar -czf "$ARM_DEST" -C "$(dirname "$ARM_ROOT")" \
    EndToEnd/arm-robot.sh EndToEnd/configs/vla/pi05.json EndToEnd/configs/vla/groot.json \
    EndToEnd/src/g1_e2e/__init__.py EndToEnd/src/g1_e2e/vla/__init__.py \
    EndToEnd/src/g1_e2e/vla/config.py EndToEnd/src/g1_e2e/vla/control.py \
    EndToEnd/src/g1_e2e/vla/http.py EndToEnd/src/g1_e2e/vla/robot.py
echo "[READY] $ARM_DEST"
