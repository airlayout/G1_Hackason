#!/usr/bin/env bash
# 長い学習を「区切って」回す。共用機（OMEN）で人が戻ったら止めて譲り、空いたら last.pt から続きを学習する。
#
# 使い方:
#   IDLE_GATE=1 REPO=VIRAL29 EXP=wbmanip/door_open_homie_29dof_lstm KIND=door29 PROJECT=door29 MEM_MAX=8G \
#     ./run_train_segments.sh <台数> <終わりの時刻 HH:MM（翌日なら翌日になる）> [追加の hydra 引数...]
#   Isaac Sim 6.0.1 で回すときは WORK=~/work_inoue/doorman6 REPO=VIRAL6 ENV_NAME=doorman6 を足す
#   （チェックポイントは 5.1 と 6.0.1 で共通。RESUME_CKPT に 5.1 の last.pt を渡して続きを学習できる）
#
# 1 区切りの終わり方（reason）:
#   deadline       終わりの時刻に達した → 全体を終える
#   person_returned / others_gpu_*  人が戻った・他人が GPU を使い始めた → 止めて、空くのを待って再開
#   low_mem_*      空きメモリが 1.5 GB を切った → 止めて、空くのを待って再開
#   nan            学習の数字に NaN が出た → 全体を終える（続けても無駄）
#   error          自分で止めていないのに落ちた（Traceback あり）→ 2 回続いたら全体を終える
#   exited         自分で止めていないのに終わった（Traceback なし）→ 同上
# 終わったら最後に「=== ... all_done reason=... last_ckpt=...」を出し、正常なら 0、それ以外は 1 で終わる。
# STOP_FILE（既定 ~/work_inoue/STOP_DOOR_CHAIN）があると、学習を始めずに reason=stop_file で終わる（stage_chain.sh を止めずに組み直すとき）
set -uo pipefail
NUM_ENVS="$1"; UNTIL="$2"; shift 2
BASE="$HOME/work_inoue/doorman"                    # LAFAN-G1・gate.sh・bench はここ
WORK="${WORK:-$BASE}"                              # 作業ツリーの置き場所（6.0.1 版は ~/work_inoue/doorman6）
ENV_BIN="$HOME/miniforge3/envs/${ENV_NAME:-doorman51}/bin"
REPO="${REPO:-VIRAL29}"; EXP="${EXP:-wbmanip/door_open_homie_29dof_lstm}"
KIND="${KIND:-door29}"; PROJECT="${PROJECT:-door29}"; MEM_MAX="${MEM_MAX:-8G}"
NEED_MB=$(( ${MEM_MAX%G} * 1024 + ${NEED_MARGIN_MB:-1536} ))  # 始める時に上乗せする余裕
export PATH="$ENV_BIN:$PATH" OMNI_KIT_ACCEPT_EULA=YES HYDRA_FULL_ERROR=1 XDG_RUNTIME_DIR="/run/user/$(id -u)"
mkdir -p "$WORK/tmp"; export TMPDIR="$WORK/tmp"   # 共用機の /tmp/Assets の権限を避ける
source "$(dirname "$0")/gate.sh"

DEADLINE=$(date -d "$UNTIL" +%s); [ "$DEADLINE" -le "$(date +%s)" ] && DEADLINE=$(date -d "tomorrow $UNTIL" +%s)
MAX_WAIT_MIN=$(( (DEADLINE - $(date +%s)) / 60 )); export MAX_WAIT_MIN
LOGROOT="$WORK/$REPO/logs_rl/$PROJECT"
CKPT="${RESUME_CKPT:-}"
SEG=0; FAILS=0; FINAL="deadline"
log() { echo "=== [$(date +%T)] $*"; }
# RESUME_CKPT=latest: 始める時に LOGROOT の中でいちばん新しい last.pt から続ける（前のキューの続きを夜通し回すとき）
if [ "$CKPT" = latest ]; then
  CKPT=$(find "$LOGROOT" -name last.pt -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
  [ -n "$CKPT" ] || { log "RESUME_CKPT=latest だが $LOGROOT に last.pt が無い"; exit 1; }
fi
STOP_FILE="${STOP_FILE:-$HOME/work_inoue/STOP_DOOR_CHAIN}"
if [ -e "$STOP_FILE" ]; then log "all_done reason=stop_file（$STOP_FILE がある）segments=0 last_ckpt=${CKPT:-none}"; exit 1; fi
log "segments start env=${ENV_NAME:-doorman51} repo=$WORK/$REPO kind=$KIND n=$NUM_ENVS until=$(date -d "@$DEADLINE" '+%m-%d %H:%M') resume=${CKPT:-none}"

latest_ckpt() { find "$LOGROOT" -name last.pt -newermt "@$1" 2>/dev/null | xargs -r ls -t 2>/dev/null | head -1; }

while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  log "wait machine free (need ${NEED_MB}MB, others_gpu<=${OTHERS_GPU_MB}MiB, idle_gate=${IDLE_GATE:-0}) seg=$SEG"
  MAX_WAIT_MIN=$(( (DEADLINE - $(date +%s)) / 60 ))
  if [ "$MAX_WAIT_MIN" -le 1 ] || ! wait_machine_free; then FINAL="deadline"; break; fi

  OUT="$BASE/bench/${KIND}_n${NUM_ENVS}_$(date +%m%d_%H%M)_seg${SEG}"; mkdir -p "$OUT"
  SEG_START=$(date +%s)
  RESUME_ARG=(); [ -n "$CKPT" ] && RESUME_ARG=("++checkpoint=$CKPT")
  log "start seg=$SEG out=$OUT resume=${CKPT:-none}" | tee "$OUT/status"
  cd "$WORK/$REPO" || exit 1
  export PYTHONPATH="$WORK/$REPO"
  nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader -l 10 > "$OUT/gpu.csv" &
  GPU_PID=$!
  setsid systemd-run --user --scope --quiet -p MemoryMax="$MEM_MAX" -p MemorySwapMax=0 -p CPUWeight=20 -p IOWeight=20 \
    "${SIGINT_DEFAULT_EXEC[@]}" python gr00t/rl/train_agent_trl.py +exp="$EXP" \
    ++num_envs="$NUM_ENVS" ++algo.config.entropy_coef=0.001 ++algo.config.num_steps_per_env=32 \
    ++env.config.delta_action_scale=0.3 ++use_wandb=False ++headless=True ++project_name="$PROJECT" \
    ++env.config.reset_from_dataset.motion_file_dir="$BASE/LAFAN-G1" "${RESUME_ARG[@]}" "$@" \
    > "$OUT/train.log" 2>&1 < /dev/null &
  PID=$!
  cd "$BASE"
  REASON=""; WE_STOPPED=0; TICK=0
  while kill -0 "$PID" 2>/dev/null; do
    AVAIL=$(mem_avail_mb)
    echo "$(date +%T),$AVAIL" >> "$OUT/mem.csv"
    if [ "$AVAIL" -lt 1500 ]; then REASON="low_mem_${AVAIL}MB"; break; fi
    if [ "$(date +%s)" -ge "$DEADLINE" ]; then REASON="deadline"; break; fi
    if [ $((TICK % 15)) -eq 0 ] && [ $(( $(date +%s) - SEG_START )) -gt 300 ]; then  # 30 秒おき。起動の 5 分は見ない
      if Y=$(should_yield "$PID"); then REASON="$Y"; break; fi
      if python3 "$BASE/train_progress.py" "$OUT/train.log" --every 1000000000 2>/dev/null | grep -q "nan_rows=[1-9]"; then
        REASON="nan"; break
      fi
    fi
    TICK=$((TICK+1)); sleep 2
  done
  if kill -0 "$PID" 2>/dev/null; then
    WE_STOPPED=1
    kill -INT -- -"$PID" 2>/dev/null
    for _ in $(seq 60); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
    kill -KILL -- -"$PID" 2>/dev/null
  fi
  wait "$PID" 2>/dev/null
  kill "$GPU_PID" 2>/dev/null
  if [ "$WE_STOPPED" -eq 0 ]; then
    if grep -aq "Traceback" "$OUT/train.log"; then
      REASON="error($(grep -aE "^[A-Za-z]*Error" "$OUT/train.log" | tail -1 | cut -c1-100))"
    else
      REASON="exited"
    fi
  fi
  NEW=$(latest_ckpt "$SEG_START"); [ -n "$NEW" ] && CKPT="$NEW"
  log "end seg=$SEG reason=$REASON ckpt=${CKPT:-none}" | tee -a "$OUT/status"
  echo BENCH_DONE >> "$OUT/status"
  SEG=$((SEG+1))
  case "$REASON" in
    deadline) FINAL="deadline"; break ;;
    nan) FINAL="nan"; break ;;
    error*|exited)
      FAILS=$((FAILS+1)); FINAL="$REASON"
      [ "$FAILS" -ge 2 ] && break
      sleep 60 ;;
    *) FAILS=0 ;;  # 譲った・メモリ不足 → 空くのを待って再開
  esac
done
log "all_done reason=$FINAL segments=$SEG last_ckpt=${CKPT:-none}"
[ "$FINAL" = deadline ] && exit 0 || exit 1
