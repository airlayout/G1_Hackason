#!/usr/bin/env bash
# 学習したチェックポイントを評価しながら、機体と扉の状態を npz に記録する（カメラは使わない）。
# 記録は Mac の MuJoCo で再生して動画にする（G1_Hackason/Door/sim2sim/replay_states.py）。
# 記録係（patches/untested/0005-state-recorder.patch）を当てた作業ツリー（既定 VIRAL29rec）で動かす。
#
# 使い方: REPO=VIRAL29rec MEM_MAX=8G ./run_eval_record.sh <checkpoint.pt> <出力先> [台数 4] [エピソード数 8] [分 15] [追加の hydra 引数...]
# STOP_WHEN_EPISODES=<回数>: 1 分ごとに count_success_cutoff.py で数え、数えられる回がこの数に届いたら（分の上限より前でも）止める
#   （1 回の起動で台ごとに何回も回す評価用。10-22 に足した。機械が遅い時間に当たると決まった分では回数が足りなくなったため）
set -uo pipefail
CKPT="$1"; OUT="$2"; NUM_ENVS="${3:-4}"; EPISODES="${4:-8}"; MINUTES="${5:-15}"; shift 5 2>/dev/null || shift $#
BASE="$HOME/work_inoue/doorman"                    # LAFAN-G1 と gate.sh はここ
WORK="${WORK:-$BASE}"                              # Isaac Sim 6.0.1 版は WORK=~/work_inoue/doorman6 REPO=VIRAL6 ENV_NAME=doorman6
ENV_BIN="$HOME/miniforge3/envs/${ENV_NAME:-doorman51}/bin"
REPO="${REPO:-VIRAL29rec}"
MEM_MAX="${MEM_MAX:-8G}"
NEED_MB=$(( ${MEM_MAX%G} * 1024 + ${NEED_MARGIN_MB:-1536} ))  # 始める時に上乗せする余裕
export PATH="$ENV_BIN:$PATH" OMNI_KIT_ACCEPT_EULA=YES HYDRA_FULL_ERROR=1 XDG_RUNTIME_DIR="/run/user/$(id -u)" PYTHONUNBUFFERED=1
mkdir -p "$WORK/tmp"; export TMPDIR="$WORK/tmp"   # 共用機の /tmp/Assets の権限を避ける
source "$(dirname "$0")/gate.sh"

echo "=== [$(date +%T)] wait machine free (need ${NEED_MB}MB, others_gpu<=${OTHERS_GPU_MB}MiB, idle_gate=${IDLE_GATE:-0}) for eval_record"
if ! wait_machine_free; then echo "=== [$(date +%T)] end eval_record reason=gate_timeout files=0"; exit 1; fi
mkdir -p "$OUT"
cd "$WORK/$REPO" || exit 1
export PYTHONPATH="$WORK/$REPO"
# 起動の見張り（2026-10-15）: Isaac Sim が起動の最初（Loading experience file の直後）でロック待ちのまま固まることがある
# （CPU 0%・GPU 0%。30 分の締め切りまで何もしない）。評価は始まると必ず「Setting Env is evaluating」を出すので、
# STARTUP_MIN 分（既定 6）たってもこの行が無ければ止めて、1 回だけ起動し直す
STARTUP_MIN="${STARTUP_MIN:-6}"
for ATTEMPT in 1 2; do
  rm -f "$OUT"/states_*.npz "$OUT"/static.npz
  echo "=== [$(date +%T)] start eval_record ckpt=$CKPT out=$OUT n=$NUM_ENVS episodes=$EPISODES attempt=$ATTEMPT"
  # eval_overrides（headless False・1 台）を上書きする。1 台だと ONNX の書き出しに入るので 2 台以上にする
  setsid systemd-run --user --scope --quiet -p MemoryMax="$MEM_MAX" -p MemorySwapMax=0 -p CPUWeight=20 -p IOWeight=20 \
    "${SIGINT_DEFAULT_EXEC[@]}" python gr00t/rl/eval_agent_trl.py +checkpoint="$CKPT" \
    ++num_envs="$NUM_ENVS" ++headless=True ++use_wandb=False \
    ++algo.config.eval.num_eval_episodes="$EPISODES" \
    ++env.config.reset_from_dataset.motion_file_dir="$BASE/LAFAN-G1" \
    ++env.config.record_states.dir="$OUT" ++env.config.record_states.num_envs="$NUM_ENVS" "$@" \
    > "$OUT/eval.log" 2>&1 < /dev/null &
  PID=$!
  START=$(date +%s)
  END=$(( START + MINUTES*60 ))
  REASON="time_up"
  while kill -0 "$PID" 2>/dev/null; do
    AVAIL=$(awk "/MemAvailable/{print int(\$2/1024)}" /proc/meminfo)
    if [ "$AVAIL" -lt 1500 ]; then REASON="low_mem_${AVAIL}MB"; break; fi
    if [ "$(date +%s)" -ge "$END" ]; then break; fi
    if [ -n "${STOP_WHEN_EPISODES:-}" ] && [ $(( $(date +%s) - ${LAST_COUNT:-0} )) -ge 60 ]; then
      LAST_COUNT=$(date +%s)
      # 書きかけの npz に当たると読めずに失敗する。そのときは次の 1 分で数え直す
      if "$ENV_BIN/python" "$BASE/count_success_cutoff.py" "$OUT" --json "$OUT/.partial_count.json" > /dev/null 2>&1; then
        N_EP=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['episodes'])" "$OUT/.partial_count.json" 2>/dev/null)
        if [ -n "$N_EP" ] && [ "$N_EP" -ge "$STOP_WHEN_EPISODES" ]; then REASON="enough_episodes_${N_EP}"; break; fi
      fi
    fi
    if [ $(( $(date +%s) - START )) -ge $(( STARTUP_MIN*60 )) ] && ! grep -aq "Setting Env is evaluating" "$OUT/eval.log"; then
      REASON="startup_hang"; break
    fi
    sleep 2
  done
  [ "$REASON" = startup_hang ] || break
  echo "=== [$(date +%T)] eval_record: ${STARTUP_MIN} 分たっても評価が始まらない（起動で固まった）。止めて起動し直す"
  kill -KILL -- -"$PID" 2>/dev/null
  wait "$PID" 2>/dev/null
  [ "$ATTEMPT" = 2 ] && break
  sleep 5
done
WE_STOPPED=0
if kill -0 "$PID" 2>/dev/null; then
  WE_STOPPED=1
  kill -INT -- -"$PID" 2>/dev/null   # KeyboardInterrupt → atexit で記録の残りを書き出す
  for _ in $(seq 60); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
  kill -KILL -- -"$PID" 2>/dev/null
elif [ "$REASON" != startup_hang ]; then
  REASON="exited"
fi
if [ "$WE_STOPPED" -eq 0 ] && grep -aq "Traceback" "$OUT/eval.log"; then
  REASON="error_traceback($(grep -aE "^[A-Za-z]*Error" "$OUT/eval.log" | tail -1 | cut -c1-120))"
fi
FILES=$(ls "$OUT"/states_*.npz 2>/dev/null | wc -l)
echo "=== [$(date +%T)] end eval_record reason=$REASON files=$FILES"
[ "$FILES" -gt 0 ] || exit 1
