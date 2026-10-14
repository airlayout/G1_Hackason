#!/usr/bin/env bash
# 段 1: DoorMan を短く回し、VRAM・RAM・速さを測る（教師・映像の生徒）。
# 出力は VIRAL*/logs_rl/<PROJECT>/…（既定 PROJECT=bench）。長い学習も <分> を大きくして同じスクリプトで回す。
# 使い方: REPO=VIRAL EXP=wbmanip/door_open_homie_lstm KIND=teacher MEM_MAX=8G ./run_bench.sh <num_envs> <分> [追加の hydra 引数...]
#
# OMEN は共用機。2026-10-06 に 512 台の起動のメモリの山（5 秒で空き 5.9→0.8 GB）の最中に、
# 他人の ChatGPT アプリが落ちた。そこで次の 2 つで他人の処理を守る:
#   1. 学習は systemd-run の cgroup（MemoryMax=MEM_MAX、swap なし）の中で動かす。溢れたら自分だけが止まる
#   2. 始める前に「GPU に他人の計算プロセスが無い」かつ「空きメモリ > MEM_MAX + 1.5 GB」を待つ
set -uo pipefail
NUM_ENVS="${1:-256}"; MINUTES="${2:-20}"; shift 2 || true
BASE="$HOME/work_inoue/doorman"
ENV_BIN="$HOME/miniforge3/envs/doorman51/bin"
EXP="${EXP:-wbmanip/door_open_homie_lstm}"; KIND="${KIND:-teacher}"
REPO="${REPO:-VIRAL}"  # VIRAL＝元の DoorMan、VIRAL29＝指の無い 29 関節版（G1_Hackason/Door/doorman/patches）
MEM_MAX="${MEM_MAX:-8G}"
MEM_MAX_MB=$(( ${MEM_MAX%G} * 1024 ))
NEED_MB=$(( MEM_MAX_MB + 1536 ))
export PATH="$ENV_BIN:$PATH" OMNI_KIT_ACCEPT_EULA=YES HYDRA_FULL_ERROR=1 XDG_RUNTIME_DIR="/run/user/$(id -u)"

source "$(dirname "$0")/gate.sh"

echo "=== [$(date +%T)] wait machine free (need ${NEED_MB}MB, no GPU apps, idle_gate=${IDLE_GATE:-0}) for ${KIND} n=${NUM_ENVS}"
wait_machine_free
TAG="${KIND}_n${NUM_ENVS}_$(date +%m%d_%H%M)"
OUT="$BASE/bench/$TAG"; mkdir -p "$OUT"
cd "$BASE/$REPO"; export PYTHONPATH="$BASE/$REPO"
echo "=== [$(date +%T)] start $TAG mem_max=$MEM_MAX" | tee "$OUT/status"
nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader -l 5 > "$OUT/gpu.csv" &
GPU_PID=$!
setsid systemd-run --user --scope --quiet -p MemoryMax="$MEM_MAX" -p MemorySwapMax=0 -p CPUWeight=20 -p IOWeight=20 \
  "${SIGINT_DEFAULT_EXEC[@]}" python gr00t/rl/train_agent_trl.py +exp="$EXP" \
  ++num_envs="$NUM_ENVS" ${BASE_ARGS:-++algo.config.entropy_coef=0.001 ++algo.config.num_steps_per_env=32 ++env.config.delta_action_scale=0.3} \
  ++use_wandb=False ++headless=True \
  ++project_name="${PROJECT:-bench}" ++env.config.reset_from_dataset.motion_file_dir="$BASE/LAFAN-G1" "$@" \
  > "$OUT/train.log" 2>&1 < /dev/null &
PID=$!
END=$(( $(date +%s) + MINUTES*60 ))
REASON="time_up"
while kill -0 "$PID" 2>/dev/null; do
  AVAIL=$(awk "/MemAvailable/{print int(\$2/1024)}" /proc/meminfo)
  RSS=$(ps -o rss= -g "$PID" 2>/dev/null | awk "{s+=\$1} END{print int(s/1024)}")
  echo "$(date +%T),$AVAIL,$RSS" >> "$OUT/mem.csv"
  if [ "$AVAIL" -lt 1500 ]; then REASON="low_mem_${AVAIL}MB"; break; fi
  if [ "$(date +%s)" -ge "$END" ]; then break; fi
  sleep 2
done
WE_STOPPED=0
if kill -0 "$PID" 2>/dev/null; then
  WE_STOPPED=1
  kill -INT -- -"$PID" 2>/dev/null
  for _ in $(seq 60); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
  kill -KILL -- -"$PID" 2>/dev/null
else
  wait "$PID"; RC=$?
  REASON="exited_rc${RC}"; [ "$RC" -eq 137 ] && REASON="cgroup_oom_killed(MemoryMax=$MEM_MAX)"
fi
kill "$GPU_PID" 2>/dev/null
# Isaac Sim は Python の例外で落ちても終了コード 0 を返すことがある（10/6 実測）。ログで判定する
if [ "$WE_STOPPED" -eq 0 ] && grep -aq "Traceback" "$OUT/train.log"; then  # 止めたときの KeyboardInterrupt は数えない
  REASON="error_traceback($(grep -aE "^[A-Za-z]*Error" "$OUT/train.log" | tail -1 | cut -c1-120))"
fi
echo "=== [$(date +%T)] end $TAG reason=$REASON" | tee -a "$OUT/status"
echo BENCH_DONE >> "$OUT/status"
