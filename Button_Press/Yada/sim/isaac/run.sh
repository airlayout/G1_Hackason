#!/usr/bin/env bash
# Isaac Sim でエレベーター乗り場のシーンを開く。
#
#   bash Button_Press/Yada/sim/isaac/run.sh                                   # 画面で見る
#   bash Button_Press/Yada/sim/isaac/run.sh --headless --press up --png _local/button_press_yada/isaac_hall
#
# 起動に 2〜5 分かかる。ログは _local/button_press_yada/logs/isaac_hall.log にも書く。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
source "$SCRIPT_DIR/env.sh"

LOG_DIR="$REPO_ROOT/_local/button_press_yada/logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/isaac_hall.log"

ARGS=("$@")
# IsaacLab 3.0 は --viz kit が無いと自動でヘッドレスになり、画面が開かずにすぐ終わる（IsaacSim_Env/run.sh と同じ対策）。
# --headless か --viz を自分で付けたときは足さない
if ! printf '%s\n' "$@" | grep -qE -- "^--(headless|viz|visualizer)"; then
    ARGS+=(--viz kit)
fi
# カメラ（頭カメラ・全体）を使うので常に有効にする
if ! printf '%s\n' "$@" | grep -q -- "--enable_cameras"; then
    ARGS+=(--enable_cameras)
fi
# 拡張のレジストリ（オンライン）に問い合わせると、起動の途中で止まることがある（IsaacSim_Env/run.sh と同じ対策）
if ! printf '%s\n' "$@" | grep -q -- "--kit_args"; then
    ARGS+=(--kit_args="--/app/extensions/registryEnabled=false")
fi

echo "[INFO] Isaac Sim を起動します（2〜5 分かかる。log: $LOG）"
"$ISAAC_SIM/python.sh" "$SCRIPT_DIR/view_hall_isaac.py" "${ARGS[@]}" 2>&1 | stdbuf -oL -eL tee "$LOG"
