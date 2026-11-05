#!/usr/bin/env bash
# 11-04 夕（ユーザー「学習を明日まで自律的に継続して」）: 画像だけの生徒の PPO の連鎖（exp_vis_1104.sh）が止まっていたら、
# 最新の重み・次の種から起こし直す見張り役。5 分おきに見る。起こし直しは MAX_RESTART 回まで（同じ所で落ち続けるのを避ける）。
# 止め札 ~/work_inoue/STOP_DOOR_VIS_ALL があれば見張りも終わる（連鎖そのものは ~/work_inoue/STOP_DOOR_VIS で止まる）。
# 使い方（OMEN の ~/work_inoue/doorman6）:
#   BASE_NAME=p3 ENC=<V2 の出力> setsid nohup bash keep_vis_1104.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"
LOG="$B/vis_1104/pipeline.log"
BASE_NAME="${BASE_NAME:-p3}"
ENC="${ENC:?}"
MAX_RESTART="${MAX_RESTART:-5}"
UNTIL="${UNTIL:-$(date -d 'tomorrow 18:00' +%s)}"
log() { echo "=== [$(date '+%m-%d %T')] keep: $*"; }
log "見張り開始 base=$BASE_NAME max_restart=$MAX_RESTART until=$(date -d @$UNTIL '+%m-%d %T')"
k=0
while [ "$(date +%s)" -lt "$UNTIL" ]; do
  sleep 300
  [ -e "$HOME/work_inoue/STOP_DOOR_VIS_ALL" ] && { log "止め札 STOP_DOOR_VIS_ALL があるので終わる"; exit 0; }
  pgrep -f "[e]xp_vis_1104.sh" > /dev/null && continue
  pgrep -f "[r]estart_vis_1104.sh|[s]witch_ppo_1104.sh|[e]val_break_1104.sh" > /dev/null && continue
  [ -e "$HOME/work_inoue/STOP_DOOR_VIS" ] && { log "連鎖は止め札で止まっている（人が止めた）ので起こさない"; continue; }
  if [ "$k" -ge "$MAX_RESTART" ]; then log "起こし直しが $MAX_RESTART 回に届いたので終わる"; exit 1; fi
  k=$((k + 1))
  NAME="${BASE_NAME}r$k"
  CK=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_"${BASE_NAME}"*/*/*/last.pt 2>/dev/null | head -1)
  LAST_SEED=$(grep -ao "区切り [0-9]* seed=[0-9]*" "$LOG" | tail -1 | grep -o "seed=[0-9]*" | cut -d= -f2)
  SEED0=$(( ${LAST_SEED:-3000} + 1 ))
  log "連鎖が止まっていた → $NAME を起こす（$k 回目）ck=$CK seed0=$SEED0"
  (EXP=wbmanip/door_open_homie_29dof_vision_f3_ppo-lstm CK="$CK" SEED0="$SEED0" ENC="$ENC" NUM_ENVS="${NUM_ENVS:-512}" \
    ROUNDS="${ROUNDS:-40}" ROUND_MIN=30 EXTRA_ARGS="${EXTRA_ARGS:-++env.config.vision_feat.nod_deg=4.0}" \
    setsid -f bash "$B/exp_vis_1104.sh" "$NAME" < /dev/null >> "$LOG" 2>&1 &)
  sleep 60
done
log "期限に届いたので見張りを終わる"
