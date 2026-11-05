#!/usr/bin/env bash
# 11-05（ユーザー「18 時までユーザが停止というまで、学習は継続して」）: 学習を 18:00 に一番近い区切りの境目で止める。
# 17:40 に今の区切りの始まり（pipeline.log の最後の「区切り N seed=」）を読み、その区切りの終わりの 20 秒前に止め札を置く
# （区切りは 30 分なので 17:40〜18:10 のどこかで止まる。途中で止めて区切りの最後の保存を失わない。switch_ppo_1104.sh と同じ考え）。
# 止め札: ~/work_inoue/STOP_DOOR_VIS（連鎖）と ~/work_inoue/STOP_DOOR_VIS_ALL（見張り役 keep3 が起こし直さない）
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash stop_at_1800_1105.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
set -uo pipefail
LOG="$HOME/work_inoue/doorman6/vis_1104/pipeline.log"
ROUND_MIN="${ROUND_MIN:-30}"
log() { echo "=== [$(date '+%m-%d %T')] stop1800: $*"; }
CHECK_AT=$(date -d 'today 17:40' +%s)
log "17:40 に今の区切りを読み、その終わりで止める"
while [ "$(date +%s)" -lt "$CHECK_AT" ]; do sleep 30; done
LINE=$(grep -a "区切り [0-9]* seed=" "$LOG" | tail -1)
START_T=$(date -d "$(echo "$LINE" | sed -E 's/^=== \[11-([0-9]+) ([0-9:]+)\].*/2026-11-\1 \2/')" +%s 2>/dev/null || echo 0)
STOP_AT=$(( START_T + ROUND_MIN * 60 - 20 ))
LATEST=$(date -d 'today 18:15' +%s)
if [ "$START_T" -le 0 ] || [ "$STOP_AT" -lt "$(date +%s)" ] || [ "$STOP_AT" -gt "$LATEST" ]; then
  STOP_AT=$(date -d 'today 18:00' +%s)  # 区切りが読めない・既に過ぎた・遠すぎるときは 18:00 ちょうど
fi
log "今の区切り: ${LINE:0:60}… → 止め札は $(date -d @"$STOP_AT" +%T)"
while [ "$(date +%s)" -lt "$STOP_AT" ]; do sleep 5; done
touch "$HOME/work_inoue/STOP_DOOR_VIS" "$HOME/work_inoue/STOP_DOOR_VIS_ALL"
log "止め札を置いた（18:00 で学習を終える）"
