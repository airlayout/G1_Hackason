#!/usr/bin/env bash
# OMEN で simfix_probe.py（パッチ 0071）を 1 回回す。直しの確かめ V1〜V10（10-26）。
#
# 使い方: run_probe.sh <名前> <probe> <台数> [Hydra の上書き...]
#   例: run_probe.sh v2_old step 4 ++robot.asset.fix_base_link=True +probe_args={joint:elbow,amp:0.2}
#       run_probe.sh v10_fix homie 64 $(./simfix_args.sh 1 4 5 7)
# 環境変数: REPO（既定 VIRAL6_fix）・CKPT（既定 45,150）・MEM_MAX（既定 8G）・MINUTES（既定 20）
# 共用機の作法: 自分以外の GPU の使用と空きメモリを gate.sh で待ち、cgroup（systemd-run）で囲う。
#   一時停止の札（PAUSE_DOORMAN_TRAINING）は評価と同じく無視して始める（IGNORE_PAUSE=1）。札は置きも外しもしない
set -uo pipefail
NAME="$1"; PROBE="$2"; NUM_ENVS="$3"; shift 3
B="$HOME/work_inoue/doorman6"
REPO="${REPO:-VIRAL6_fix}"
CKPT="${CKPT:-$HOME/work_inoue/doorman/ckpt/start_1016_1359/model_step_045150.pt}"
MEM_MAX="${MEM_MAX:-8G}"
NEED_MB=$(( ${MEM_MAX%G} * 1024 + 1536 ))
OUT="$B/simfix/${NAME}_$(date +%m%d_%H%M%S)"
mkdir -p "$OUT" "$B/tmp"
export PATH="$HOME/miniforge3/envs/doorman6/bin:$PATH" TMPDIR="$B/tmp" OMNI_KIT_ACCEPT_EULA=YES HYDRA_FULL_ERROR=1 \
  PYTHONUNBUFFERED=1 XDG_RUNTIME_DIR="/run/user/$(id -u)" PYTHONPATH="$B/$REPO"
# 段 3・4（handover_sim.py）が読む実機用のコード（G1_Hackason/Door/real の写し）と重みの npz
export DOOR_REAL_DIR="${DOOR_REAL_DIR:-$B/door_real}" DOOR_POLICY="${DOOR_POLICY:-$B/door_real/door_policy_it45150.npz}"
source "$HOME/work_inoue/doorman/gate.sh"
IGNORE_PAUSE=1
if [ "${SKIP_GATE:-0}" = 1 ]; then
  # まとめて回す時（run_probe_batch.sh）: 最初の 1 回だけ 2 分待つ。以後は他の人の GPU の使用と空きメモリだけをその場で見る
  others=$(others_gpu_mb); avail=$(mem_avail_mb)
  if [ "$others" -gt "$OTHERS_GPU_MB" ] || [ "$avail" -le "$NEED_MB" ]; then
    echo "=== [$(date +%T)] end $NAME reason=machine_busy others_gpu=${others}MiB avail=${avail}MB"; exit 2
  fi
else
  echo "=== [$(date +%T)] wait machine free (need ${NEED_MB}MB) $NAME"
  if ! IGNORE_PAUSE=1 wait_machine_free; then echo "=== [$(date +%T)] end $NAME reason=gate_timeout"; exit 1; fi
fi
cd "$B/$REPO" || exit 1
echo "=== [$(date +%T)] start $NAME probe=$PROBE n=$NUM_ENVS out=$OUT"
printf '%s\n' "$@" > "$OUT/overrides.txt"
setsid timeout -s INT "$(( ${MINUTES:-20} * 60 ))" systemd-run --user --scope --quiet -p MemoryMax="$MEM_MAX" -p MemorySwapMax=0 -p CPUWeight=20 \
  "${SIGINT_DEFAULT_EXEC[@]}" python gr00t/rl/simfix_probe.py +checkpoint="$CKPT" +probe="$PROBE" +probe_out="$OUT" \
  ++num_envs="$NUM_ENVS" ++headless=True ++use_wandb=False \
  ++env.config.reset_from_dataset.enabled=False ++env.config.reset_from_dataset.motion_file_dir="$HOME/work_inoue/doorman/LAFAN-G1" \
  ++env.config.mirror_left_doors=False ++env.config.reset_on_overtime=False ++env.config.max_episode_length_s=200 \
  "$@" > "$OUT/probe.log" 2>&1 < /dev/null
RC=$?
echo "=== [$(date +%T)] end $NAME rc=$RC"
grep -aE "^[A-Za-z]*Error|Traceback" "$OUT/probe.log" | tail -3
grep -a "\[simfix_probe\] RESULT" "$OUT/probe.log" | tail -1 | cut -c1-3000
echo "OUT=$OUT"
