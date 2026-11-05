#!/usr/bin/env bash
# 11-04: 動いている連鎖（NAME）を、今の区切りの終わり（WAIT_LINE が pipeline.log に出たとき）で止め、最新の重みから設定を替えて始め直す。
# 使い方（OMEN の ~/work_inoue/doorman6）:
#   NAME=p2 NEW_NAME=p3 WAIT_LINE="区切り 0 終わり" EXP=... NUM_ENVS=512 SEED0=2200 ENC=... EXTRA_ARGS="..." \
#     setsid nohup bash restart_vis_1104.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"
LOG="$B/vis_1104/pipeline.log"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
# 先にログを書いてから数える（11-04: 待つ文字を自分のログに書いた行を数えて、待たずに止めていた）
log "restart: 区切りの終わりを待って $NAME を止め、$NEW_NAME（n=${NUM_ENVS} extra=${EXTRA_ARGS:-}）で始め直す"
N0=$(grep -ac "${WAIT_LINE:?}" "$LOG")
until [ "$(grep -ac "$WAIT_LINE" "$LOG")" -gt "$N0" ]; do sleep 2; done
touch "$HOME/work_inoue/STOP_DOOR_VIS"
while pgrep -f "[e]xp_vis_1104.sh $NAME" > /dev/null; do sleep 3; done
CK=$(ls -t "$B"/VIRAL6_vis/logs_rl/"vis_$NAME"/*/*/last.pt | head -1)
rm -f "$HOME/work_inoue/STOP_DOOR_VIS"
log "restart: $NAME を止めた。$NEW_NAME を ck=$CK から始める"
CK="$CK" bash "$B/exp_vis_1104.sh" "$NEW_NAME"
log "restart: $NEW_NAME 終わり"
