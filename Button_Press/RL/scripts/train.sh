#!/usr/bin/env bash
# 教師を学習する（Isaac Lab の rsl_rl の train.py を、このパッケージを登録して動かす）。
#
#   bash Button_Press/RL/scripts/train.sh                       # 画面なし。並列数は GPU から決める
#   bash Button_Press/RL/scripts/train.sh --num_envs 64 --max_iterations 10   # 動作の確認
#   bash Button_Press/RL/scripts/train.sh --machine rtx5060ti   # マシンの設定を指定（configs/machines.yaml）
#   bash Button_Press/RL/scripts/train.sh --resume --load_run <フォルダ名>   # 続きから
#
# ログと checkpoint は _local/button_press_rl/logs/rsl_rl/button_press_teacher/<日時>/ に書く。
# 起動に 2〜5 分かかる。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

TASK="G1-ButtonPress-Teacher-v0"
MACHINE=""
ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --machine) MACHINE="$2"; shift 2 ;;
        --task) TASK="$2"; shift 2 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done

if ! printf '%s\n' "${ARGS[@]+"${ARGS[@]}"}" | grep -q -- "--num_envs"; then
    GPU="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || true)"
    NUM_ENVS="$("$ISAAC_SIM/python.sh" - "$RL_DIR/configs/machines.yaml" "$GPU" "$MACHINE" <<'PY'
import sys, yaml
m = yaml.safe_load(open(sys.argv[1]))["machines"]
gpu, name = sys.argv[2], sys.argv[3]
if name:
    print(m[name]["teacher_num_envs"])
else:
    for k, v in m.items():
        if k != "default" and v["match"] and v["match"] in gpu:
            print(v["teacher_num_envs"]); break
    else:
        print(m["default"]["teacher_num_envs"])
PY
)"
    echo "[train] GPU: ${GPU:-不明}、並列の環境の数: $NUM_ENVS（configs/machines.yaml）"
    ARGS+=(--num_envs "$NUM_ENVS")
fi
if ! printf '%s\n' "${ARGS[@]}" | grep -q -- "--kit_args"; then
    ARGS+=(--kit_args="$ISAAC_KIT_ARGS")
fi

LOG="$RL_OUT_DIR/train_$(date +%Y%m%d_%H%M%S).log"
echo "[train] タスク $TASK（ログ: $LOG）"
cd "$RL_OUT_DIR"
"$ISAAC_SIM/python.sh" "$RL_DIR/scripts/run_isaaclab.py" train --task "$TASK" --headless "${ARGS[@]}" 2>&1 \
    | stdbuf -oL -eL tee "$LOG"
