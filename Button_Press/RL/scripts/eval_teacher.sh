#!/usr/bin/env bash
# 学習した教師を学習環境の中でたくさんの試行で動かし、成功率と押す力の分布を測る（scripts/eval_teacher.py）。
#   bash Button_Press/RL/scripts/eval_teacher.sh --episodes 2000
#   bash Button_Press/RL/scripts/eval_teacher.sh --load_run 2026-10-20_00-14-30 --checkpoint model_900.pt
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
ARGS=("$@")
if ! printf '%s\n' "${ARGS[@]+"${ARGS[@]}"}" | grep -q -- "--kit_args"; then
    ARGS+=(--kit_args="$ISAAC_KIT_ARGS")
fi
cd "$RL_OUT_DIR"
"$ISAAC_SIM/python.sh" "$RL_DIR/scripts/eval_teacher.py" --headless "${ARGS[@]}"
