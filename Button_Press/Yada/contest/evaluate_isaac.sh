#!/usr/bin/env bash
# Isaac Sim でエージェントを評価する（contest/evaluate_isaac.py を起動する）。
#
#   bash Button_Press/Yada/contest/evaluate_isaac.sh --agent Button_Press/Yada/contest/example_agent --seeds smoke
#   bash Button_Press/Yada/contest/evaluate_isaac.sh --agent ... --seed 3 --viz kit    # 画面で見る
#
# 既定は画面なし（ヘッドレス）。起動に 2〜5 分かかる。ログは _local/button_press_yada/logs/evaluate_isaac.log にも書く。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$FEATURE_DIR/../.." && pwd)"
source "$FEATURE_DIR/sim/isaac/env.sh"

LOG_DIR="$REPO_ROOT/_local/button_press_yada/logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/evaluate_isaac.log"

ARGS=("$@")
# 頭カメラを使うので常に有効にする
if ! printf '%s\n' "$@" | grep -q -- "--enable_cameras"; then
    ARGS+=(--enable_cameras)
fi
# Kit に起動のときに渡す設定（拡張のレジストリを使わない、環境光の強さ。env.sh の ISAAC_KIT_ARGS）
if ! printf '%s\n' "$@" | grep -q -- "--kit_args"; then
    ARGS+=(--kit_args="$ISAAC_KIT_ARGS")
fi

echo "[INFO] Isaac Sim で評価します（起動に 2〜5 分かかる。log: $LOG）"
"$ISAAC_SIM/python.sh" "$SCRIPT_DIR/evaluate_isaac.py" "${ARGS[@]}" 2>&1 | stdbuf -oL -eL tee "$LOG"
