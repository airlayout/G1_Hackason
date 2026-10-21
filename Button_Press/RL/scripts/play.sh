#!/usr/bin/env bash
# 学習した方策を Isaac Lab の中で動かして見る（Isaac Lab の rsl_rl の play.py）。
#
#   bash Button_Press/RL/scripts/play.sh --num_envs 16                     # 最新の checkpoint
#   bash Button_Press/RL/scripts/play.sh --num_envs 16 --load_run <フォルダ名> --checkpoint model_1000.pt
#   bash Button_Press/RL/scripts/play.sh --num_envs 4 --headless --video   # 画面なしで動画を残す
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

TASK="G1-ButtonPress-Teacher-v0"
ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --task) TASK="$2"; shift 2 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done
if ! printf '%s\n' "${ARGS[@]+"${ARGS[@]}"}" | grep -q -- "--kit_args"; then
    ARGS+=(--kit_args="$ISAAC_KIT_ARGS")
fi
cd "$RL_OUT_DIR"
"$ISAAC_SIM/python.sh" "$RL_DIR/scripts/run_isaaclab.py" play --task "$TASK" "${ARGS[@]}"
