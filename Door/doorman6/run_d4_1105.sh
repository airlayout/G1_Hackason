#!/usr/bin/env bash
# 11-05 夜（ユーザー「強化学習を再開して明日の 8 時まで継続して改善を行なって」）: d3（追う段つきの DAgger）の最後の重みから、
# 特徴の遅れ（env.config.vision_feat.delay_steps=[2,4]＝撮ってから方策が使うまで 40〜160 ms。実機の 67〜159 ms に合わせる）を入れて
# DAgger を続ける（d4）。頷き 4°・追う段・扉の頭を固める・隠れ状態を溜めるは d3 と同じ。
# 学習の口が落ちても（11-05 の区切り 7 は RAM の上限で落ちた）、最新の重みから連鎖を起こし直す（最大 RESTARTS 回）。
# 06:45 に今の区切りの始まりを読み、その終わりの 20 秒前（07:20 より後なら 07:15）に止め札を置く。
# 止まったら学習に無い扉（seed 999・256 回）で 4 通り評価し、d4 を実機用に書き出す（07:45 ごろに終わる見込み）。
#
# 使い方（OMEN の ~/work_inoue/doorman6）:
#   setsid nohup bash run_d4_1105.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"
V="$B/vis_1104"
LOG="$V/pipeline.log"
D2="$B/VIRAL6_vis/logs_rl/vis_d2/wbmanip/door_open_homie_29dof_vision_f3_dagger-lstm_lstm-20261105_084344/last.pt"
D3="$B/VIRAL6_vis/logs_rl/vis_d3/wbmanip/door_open_homie_29dof_vision_f3_dagger-lstm_lstm-20261105_172544/last.pt"
ENC="$V/pretrain_color_1105_0807.pt"
STOP="$HOME/work_inoue/STOP_DOOR_VIS"
STOP_ALL="$HOME/work_inoue/STOP_DOOR_VIS_ALL"
RESTARTS="${RESTARTS:-6}"
log() { echo "=== [$(date '+%m-%d %T')] d4: $*"; }
[ -f "$D2" ] && [ -f "$D3" ] && [ -f "$ENC" ] || { log "d2・d3 の重みか ENC が無い"; exit 1; }
NOD=++env.config.vision_feat.nod_deg=4.0
NOD0=++env.config.vision_feat.nod_deg=0.0
DELAY="++env.config.vision_feat.delay_steps=[2,4]"
DELAY0="++env.config.vision_feat.delay_steps=[0,0]"
TRK="++algo.config.actor.door_tracker_hidden=128 ++algo.config.actor.init_student_from=$D2"
TRK="$TRK ++algo.config.actor.freeze_door_head=True ++algo.config.save_actor_hidden_states=True"
rm -f "$STOP" "$STOP_ALL"

# 止め札の係: 06:45 に今の区切りを読み、その終わりで止める（区切りの最後の保存を失わない。stop_at_1800_1105.sh と同じ考え）
(
  CHECK_AT=$(date -d 'tomorrow 06:45' +%s)
  [ "$(date +%H)" -lt 6 ] && CHECK_AT=$(date -d 'today 06:45' +%s)
  HARD=$(( CHECK_AT + 30 * 60 ))  # 07:15
  LATEST=$(( CHECK_AT + 35 * 60 ))  # 07:20
  while [ "$(date +%s)" -lt "$CHECK_AT" ]; do [ -e "$STOP_ALL" ] && exit 0; sleep 30; done
  LINE=$(grep -a "区切り [0-9]* seed=" "$LOG" | tail -1)
  START_T=$(date -d "$(echo "$LINE" | sed -E 's/^=== \[([0-9]+)-([0-9]+) ([0-9:]+)\].*/2026-\1-\2 \3/')" +%s 2>/dev/null || echo 0)
  STOP_AT=$(( START_T + 30 * 60 - 20 ))
  if [ "$START_T" -le 0 ] || [ "$STOP_AT" -lt "$(date +%s)" ] || [ "$STOP_AT" -gt "$LATEST" ]; then STOP_AT=$HARD; fi
  log "止め札の係: 今の区切り ${LINE:0:50}… → $(date -d @"$STOP_AT" '+%m-%d %T') に止める"
  while [ "$(date +%s)" -lt "$STOP_AT" ]; do sleep 5; done
  touch "$STOP" "$STOP_ALL"
  log "止め札の係: 止め札を置いた"
) &
TIMER=$!

newest() { ls -t "$B"/VIRAL6_vis/logs_rl/vis_d4/*/*/last.pt 2>/dev/null | head -1; }
log "始める: d3 から・遅れ [2,4] 歩・頷き 4°・追う段 128・区切り 30 分・種 3200〜"
CK="$D3"
SEED0=3200
for attempt in $(seq 0 "$RESTARTS"); do
  [ -e "$STOP_ALL" ] && break
  log "連鎖 $attempt を起こす ck=$CK seed0=$SEED0"
  ENC="$ENC" CK="$CK" NUM_ENVS=256 ROUNDS=48 ROUND_MIN=30 SEED0="$SEED0" EXTRA_ARGS="$NOD $DELAY $TRK" \
    bash "$B/exp_vis_1104.sh" d4
  [ -e "$STOP_ALL" ] && break
  NEW=$(newest)
  if [ -z "$NEW" ] || [ "$NEW" = "$CK" ]; then
    log "連鎖 $attempt が新しい重みを残さずに終わった（設定の誤りの疑い）。起こし直さない"
    break
  fi
  CK="$NEW"
  SEED0=$(( SEED0 + 20 ))  # 起こし直しは別の扉から
  log "連鎖 $attempt が止め札の前に終わった。最新の重みから起こし直す"
done
kill "$TIMER" 2>/dev/null
touch "$STOP_ALL"

D4=$(newest)
[ -n "$D4" ] || { log "d4 の重みが無い"; exit 1; }
TAG=$(date +%m%d_%H%M)
ev() {  # ev <名前> <重み> <引数...>
  local name="$1" ck="$2"
  shift 2
  bash "$B/eval_vis_1104.sh" "$ck" "$V/eval_${TAG}_$name" 256 "$@" > "$V/eval_${TAG}_$name.log" 2>&1
  log "評価 $name: $(tail -n 1 "$V/eval_${TAG}_$name.log")"
}
log "評価を始める d4=$D4"
ev d4_delay "$D4" "$DELAY" "$NOD0"
ev d3_delay "$D3" "$DELAY" "$NOD0"
ev d4_nodelay "$D4" "$DELAY0" "$NOD0"
ev d4_delay_nod4 "$D4" "$DELAY" "$NOD"

log "実機用に書き出す"
IMG=$(ls "$V"/ds2/s19/frames/shard_*.npz | head -1)
cd "$B/VIRAL6_vis" || exit 1
XDG_RUNTIME_DIR="/run/user/$(id -u)" systemd-run --user --scope --quiet -p MemoryMax=3G -p CPUWeight=20 \
  env CUDA_VISIBLE_DEVICES= OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 PYTHONPATH=".:$B/real_vis" \
  "$HOME/miniforge3/envs/doorman6/bin/python" "$B/real_vis/export_vision_student.py" --ckpt "$D4" --viral . \
  --images "$IMG" --out-dir "$B/vis_export" --name door_vision_d4 2>&1 | grep -a "\[export\]"
log "終わり: $(ls "$B"/vis_export/door_vision_d4* 2>/dev/null | tr '\n' ' ')"
