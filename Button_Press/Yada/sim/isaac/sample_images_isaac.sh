#!/usr/bin/env bash
# Isaac Sim でサンプルの画像を作る（sim/isaac/sample_images_isaac.py を起動する）。画面は開かない。
#   bash Button_Press/Yada/sim/isaac/sample_images_isaac.sh --seed 1
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
echo "[INFO] Isaac Sim を起動します（2〜5 分かかる）"
"$ISAAC_SIM/python.sh" "$SCRIPT_DIR/sample_images_isaac.py" --headless --enable_cameras \
    --kit_args="--/app/extensions/registryEnabled=false" "$@"
