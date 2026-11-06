#!/usr/bin/env bash
# 11-04（画像だけの生徒 V1・V2）: 教師 f3sc 27 回目に扉を開けさせながら、頭のカラーの画像（108×192）と答え（扉の本当の値 17・
# 手掛かり 9・段階・俯角）を 10 Hz で書き出す（env.config.vision_dump、door_vision_feat.VisionDumper）。
# 扉は種（SEED）ごとに台の数だけ作り直されるので、種を替えて何回か回すと扉の種類が増える。速さ（歩/秒）と GPU の山も残す。
#
# 使い方（OMEN の ~/work_inoue/doorman6）: SEED=1 NUM_ENVS=64 MINUTES=12 MAX_FRAMES=8000 bash collect_vis_1104.sh <出力先>
#   KEEP_FRAC: 1 回の書き出しで残す台の割合（既定 0.25）。PITCH: 俯角の範囲（既定 [30.0,70.0]）
#   CAM_TYPES: 描く種類（11-06 カラー＋深度は [{rgb:true},{depth:true}] と ++env.config.vision_dump.depth=True）
#   HAP/VAP: カメラの開口（焦点 1.88。既定 2.6035×1.4621＝D435 のカラーの 69°×42°。11-04 の ds1 は赤外線の 3.567×2.006＝87°×56° だった）
set -uo pipefail
OUT="$1"; shift
B="$HOME/work_inoue/doorman6"
BASE="$HOME/work_inoue/doorman"
REPO="${REPO:-VIRAL6_vis}"
CKPT="${CKPT:-$BASE/ckpt/start_1103_0807/last.pt}"
N="${NUM_ENVS:-64}"
SEED="${SEED:-1}"
MINUTES="${MINUTES:-12}"
MEM_MAX="${MEM_MAX:-11G}"
NEED_MB=$(( ${MEM_MAX%G} * 1024 + ${NEED_MARGIN_MB:-1024} ))
export PATH="$HOME/miniforge3/envs/doorman6/bin:$PATH" OMNI_KIT_ACCEPT_EULA=YES HYDRA_FULL_ERROR=1 \
  XDG_RUNTIME_DIR="/run/user/$(id -u)" PYTHONUNBUFFERED=1 TMPDIR="$B/tmp" PYTHONPATH="$B/$REPO" \
  PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
mkdir -p "$OUT" "$B/tmp"
# 描く種類（既定はカラーだけ）。${VAR:-既定} の既定に } を書くと展開がそこで閉じるので、先に別の変数へ入れる
CAM_TYPES_DEFAULT="[{rgb:true},{depth:false}]"
CAM_TYPES="${CAM_TYPES:-$CAM_TYPES_DEFAULT}"
source "$BASE/gate.sh"
export IGNORE_PAUSE=1
log() { echo "=== [$(date '+%m-%d %T')] $*" | tee -a "$OUT/collect.log"; }
# SKIP_GATE=1: 種を替えて続けて回すとき（collect_seeds_1104.sh）。2 分の空き待ちの代わりに、その場で 1 回だけ確かめる
if [ "${SKIP_GATE:-0}" = 1 ]; then
  O=$(others_gpu_mb); A=$(mem_avail_mb)
  log "gate (1 回) others_gpu=${O}MiB avail=${A}MB need=${NEED_MB}MB"
  [ "$O" -le "$OTHERS_GPU_MB" ] && [ "$A" -gt "$NEED_MB" ] || { log "空いていない"; exit 1; }
else
  log "wait machine free need=${NEED_MB}MB"
  wait_machine_free || { log "空かない"; exit 1; }
fi
cd "$B/$REPO" || exit 1
nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 1000 > "$OUT/gpu.csv" &
MP=$!
trap 'kill "$MP" 2>/dev/null' EXIT
log "start seed=$SEED n=$N minutes=$MINUTES max_frames=${MAX_FRAMES:-0} ckpt=$CKPT"
# 評価の口（eval_agent_trl.py）で f3sc の設定（重みの隣の config.yaml）をそのまま使い、カメラと書き出しだけを足す。
# 回の数は時間で切るので大きく。台ごとに 1 回ではなく続けて回す（eval_num_envs_episodes=False）
setsid systemd-run --user --scope --quiet -p MemoryMax="$MEM_MAX" -p MemorySwapMax=0 -p CPUWeight=20 \
  "${SIGINT_DEFAULT_EXEC[@]}" python gr00t/rl/eval_agent_trl.py +checkpoint="$CKPT" \
  ++num_envs="$N" ++seed="$SEED" ++headless=True ++use_wandb=False \
  ++algo.config.eval.num_eval_episodes=1000000 ++algo.config.eval.eval_num_envs_episodes=False \
  ++env.config.reset_from_dataset.motion_file_dir="$BASE/LAFAN-G1" \
  ++enable_cameras=True ++simulator.config.cameras.enable_cameras=True \
  "++simulator.config.cameras.camera_types=$CAM_TYPES" \
  "++simulator.config.cameras.camera_resolutions=[108,192]" \
  ++simulator.config.cameras.camera_attached_link=torso_link \
  "++simulator.config.cameras.camera_pos_offset=[0.0576,0.0525,0.4299]" \
  "++simulator.config.cameras.camera_rot_offset=[0.90244,0.0,0.43081,0.0]" \
  ++simulator.config.cameras.camera_horizontal_aperture="${HAP:-2.6035}" ++simulator.config.cameras.camera_vertical_aperture="${VAP:-1.4621}" \
  ++domain_rand.randomize_dome_light=True ++domain_rand.randomize_camera_pitch=True \
  "++domain_rand.camera_pitch_range_deg=${PITCH:-[30.0,70.0]}" \
  ++env.config.vision_dump.dir="$OUT/frames" ++env.config.vision_dump.keep_frac="${KEEP_FRAC:-0.25}" \
  ++env.config.vision_dump.max_frames="${MAX_FRAMES:-0}" ++env.config.vision_dump.seed="$SEED" \
  ++env.config.vision_feat.every_n=5 "$@" \
  > "$OUT/eval.log" 2>&1 < /dev/null &
PID=$!
START=$(date +%s)
END=$(( START + MINUTES * 60 ))
REASON=time_up
while kill -0 "$PID" 2>/dev/null; do
  [ "$(date +%s)" -ge "$END" ] && break
  grep -aq "max_frames（" "$OUT/eval.log" && { REASON=max_frames; break; }
  grep -aq "Traceback" "$OUT/eval.log" && { REASON=traceback; sleep 5; break; }
  if [ $(( $(date +%s) - START )) -ge 600 ] && ! grep -aq "Setting Env is evaluating" "$OUT/eval.log"; then REASON=startup_hang; break; fi
  sleep 5
done
kill -0 "$PID" 2>/dev/null || [ "$REASON" != time_up ] || REASON=exited
kill -INT -- -"$PID" 2>/dev/null
for _ in $(seq 30); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
kill -KILL -- -"$PID" 2>/dev/null
log "end reason=$REASON shards=$(ls "$OUT"/frames/shard_*.npz 2>/dev/null | wc -l) gpu_peak=$(awk -F', ' 'BEGIN{m=0}{if($2>m)m=$2}END{print m}' "$OUT/gpu.csv")MiB"
grep -a "記録 " "$OUT/eval.log" | tail -2 | tee -a "$OUT/collect.log"
