#!/usr/bin/env bash
# Isaac Sim 6.0.1（doorman6・VIRAL6）で DoorMan を短く起動して確かめる。5.1 の学習は pause_training.sh on で止めてから使う。
# 使い方: ./smoke6.sh <名前> <分> <MemoryMax 例 10G> [hydra の引数...]
#   例: ./smoke6.sh t29 6 10G +exp=wbmanip/door_open_homie_29dof_lstm ++num_envs=64
#   生徒（カメラ）: STUDENT=1 ./smoke6.sh s29 10 12G +exp=wbmanip/door_open_homie_29dof_dagger-lstm ++num_envs=16 \
#                     ++teacher_actor_path=<教師の last.pt の絶対パス>
#   STUDENT=1 のときは教師の学習専用の上書き（entropy・num_steps_per_env）を付けない。delta_action_scale は教師に合わせて付ける
# 出力: ~/work_inoue/doorman6/tests/<名前>_<時刻>/train.log。STACK_EVERY=秒 で全スレッドの呼び出しの履歴を stacks.txt に書く
# ⚠️ 書き出しは診断のときだけ使う。10-07 に 90 秒の書き出しの直後に Segmentation fault で落ちた（faulthandler の時間指定はスレッドの多い処理で危うい）
set -uo pipefail
NAME="$1"; MINUTES="$2"; MEM_MAX="$3"; shift 3
B="$HOME/work_inoue/doorman6"
OUT="$B/tests/${NAME}_$(date +%m%d_%H%M)"; mkdir -p "$OUT" "$B/tmp"
export PATH="$HOME/miniforge3/envs/doorman6/bin:$PATH" TMPDIR="$B/tmp" OMNI_KIT_ACCEPT_EULA=YES HYDRA_FULL_ERROR=1 \
  PYTHONUNBUFFERED=1 XDG_RUNTIME_DIR="/run/user/$(id -u)" PYTHONPATH="$B/VIRAL6"
source "$HOME/work_inoue/doorman/gate.sh"   # SIGINT_DEFAULT_EXEC
cd "$B/VIRAL6" || exit 1
TEACHER_ONLY=(++algo.config.entropy_coef=0.001 ++algo.config.num_steps_per_env=32)
[ "${STUDENT:-0}" = 1 ] && TEACHER_ONLY=()
echo "=== [$(date +%T)] start $OUT"
setsid timeout -s INT "$((MINUTES*60))" systemd-run --user --scope --quiet -p MemoryMax="$MEM_MAX" -p MemorySwapMax=0 -p CPUWeight=20 \
  "${SIGINT_DEFAULT_EXEC[@]}" env STACK_FILE="$OUT/stacks.txt" STACK_EVERY="${STACK_EVERY:-1000000}" \
  python "$B/stackdump_run.py" gr00t/rl/train_agent_trl.py "$@" "${TEACHER_ONLY[@]}" ++env.config.delta_action_scale=0.3 \
  ++use_wandb=False ++headless=True ++project_name=port6 \
  ++env.config.reset_from_dataset.motion_file_dir="$HOME/work_inoue/doorman/LAFAN-G1" \
  > "$OUT/train.log" 2>&1 < /dev/null &
PID=$!
peak=0
while kill -0 "$PID" 2>/dev/null; do
  r=$(ps -o rss= -g "$PID" 2>/dev/null | awk '{s+=$1} END{print int(s/1024)}')
  [ "${r:-0}" -gt "$peak" ] && peak=$r
  sleep 3
done
echo "=== [$(date +%T)] end $OUT peak_rss_mb=$peak"
grep -aE "^[A-Za-z]*Error|Traceback" "$OUT/train.log" | tail -3
grep -a "Learning iteration" "$OUT/train.log" | tail -1
