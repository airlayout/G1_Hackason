#!/usr/bin/env bash
# 11-04 夕（ユーザー「学習を明日まで自律的に継続して」「GPU を増やしても良い」「落ちたら 16 台ずつ減らして」）:
# 画像だけの生徒の PPO を明日まで回し続ける見張り役（keep_vis_1104.sh の代わり）。
#   1. 今の連鎖（OLD_NAME）の次の区切りの終わりで止める（止め札）
#   2. 最新の重み・次の種から、NUM_ENVS 台で連鎖（exp_vis2_1104.sh。どの区切りでも落ちたら止まる）を始める
#   3. 連鎖が落ちて止まったら台数を STEP（16）減らして起こし直す（下限 MIN_ENVS）。全区切りを回し終えたら同じ台数で続ける
# 止め札: ~/work_inoue/STOP_DOOR_VIS（連鎖を止める。見張りも起こし直さずに終わる）・~/work_inoue/STOP_DOOR_VIS_ALL（見張りが終わる）
# 使い方（OMEN の ~/work_inoue/doorman6）:
#   OLD_NAME=p3 BASE_NAME=p3 NUM_ENVS=768 ENC=<V2 の出力> setsid nohup bash keep2_vis_1104.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"
LOG="$B/vis_1104/pipeline.log"
ENC="${ENC:?}"
BASE_NAME="${BASE_NAME:-p3}"
N="${NUM_ENVS:-768}"
STEP="${STEP:-16}"
MIN_ENVS="${MIN_ENVS:-256}"
UNTIL="${UNTIL:-$(date -d 'tomorrow 18:00' +%s)}"
EXTRA="${EXTRA_ARGS:-++env.config.vision_feat.nod_deg=4.0}"
log() { echo "=== [$(date '+%m-%d %T')] keep2: $*"; }
log "見張り開始 old=${OLD_NAME:-なし} base=$BASE_NAME n=$N step=$STEP min=$MIN_ENVS until=$(date -d @$UNTIL '+%m-%d %T')"
if [ -n "${OLD_NAME:-}" ] && pgrep -f "[e]xp_vis_1104.sh $OLD_NAME" > /dev/null; then
  N0=$(grep -ac "区切り [0-9]* 終わり" "$LOG")
  until [ "$(grep -ac "区切り [0-9]* 終わり" "$LOG")" -gt "$N0" ]; do
    pgrep -f "[e]xp_vis_1104.sh $OLD_NAME" > /dev/null || break
    sleep 2
  done
  touch "$HOME/work_inoue/STOP_DOOR_VIS"
  while pgrep -f "[e]xp_vis_1104.sh $OLD_NAME" > /dev/null; do sleep 3; done
  rm -f "$HOME/work_inoue/STOP_DOOR_VIS"
  log "$OLD_NAME を区切りの境目で止めた"
fi
k=0
while [ "$(date +%s)" -lt "$UNTIL" ]; do
  [ -e "$HOME/work_inoue/STOP_DOOR_VIS_ALL" ] && { log "止め札 STOP_DOOR_VIS_ALL があるので終わる"; exit 0; }
  k=$((k + 1))
  NAME="${BASE_NAME}c$k"
  CK=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_"${BASE_NAME}"*/*/*/last.pt 2>/dev/null | head -1)
  LAST_SEED=$(grep -ao "区切り [0-9]* seed=[0-9]*" "$LOG" | tail -1 | grep -o "seed=[0-9]*" | cut -d= -f2)
  SEED0=$(( ${LAST_SEED:-3000} + 1 ))
  log "連鎖 $NAME を始める n=$N ck=$CK seed0=$SEED0"
  START_LINE=$(wc -l < "$LOG")
  EXP=wbmanip/door_open_homie_29dof_vision_f3_ppo-lstm CK="$CK" SEED0="$SEED0" ENC="$ENC" NUM_ENVS="$N" \
    ROUNDS="${ROUNDS:-40}" ROUND_MIN=30 EXTRA_ARGS="$EXTRA" bash "$B/exp_vis2_1104.sh" "$NAME" < /dev/null >> "$LOG" 2>&1
  if [ -e "$HOME/work_inoue/STOP_DOOR_VIS" ]; then log "止め札で止まったので起こし直さずに終わる"; exit 0; fi
  if tail -n +"$START_LINE" "$LOG" | grep -aq "落ちた\|空かない"; then
    NEW_N=$(( N - STEP ))
    [ "$NEW_N" -lt "$MIN_ENVS" ] && { log "台数が下限 $MIN_ENVS を下回るので終わる"; exit 1; }
    log "連鎖 $NAME が落ちた → 台数 $N → $NEW_N で起こし直す"
    N="$NEW_N"
  else
    log "連鎖 $NAME が全区切りを回し終えた → 同じ台数 $N で続ける"
  fi
  sleep 10
done
log "期限に届いたので見張りを終わる"
