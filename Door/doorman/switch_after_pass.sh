#!/usr/bin/env bash
# 走っているチェーンで、ある段が合格したら、次の段に入る前でチェーンを止め、その段の合格の ckpt から別の段の並びで起動し直す。
# 走っている stage_chain.sh は書き換えても読み直さないので、後ろの段の設定（学習に足す引数など）を変えるときに使う。
#
# 使い方（OMEN の ~/work_inoue/doorman で）:
#   EVAL_FIRST=1 ./switch_after_pass.sh <待つ chain の日時> <合格を待つ段の名前> <段>...（stage_chain.sh の段の書き方）
#   例: 等級 2 に合格したら、等級 3〜5 を押す力の頭打ちを上げて回す
#     EVAL_FIRST=1 ./switch_after_pass.sh 1008_1309 g2 g3:door_en1154_g3:3:60:door30:++env.config.push_door_force_max=50 ...
# 段が合格せずにチェーンが終わったら、何もしないで終わる
# 2026-10-08: 段 2-3 の等級 3 以上で、前へ押す力の報酬の頭打ち（20 N）がその等級で要る力より小さかったので足した
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
LOG="chain/${1:?chain の日時}/chain.log"
WAIT_STAGE="${2:?合格を待つ段の名前}"; shift 2
STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

until grep -q "stage=$WAIT_STAGE PASS" "$LOG" || grep -q "chain_done" "$LOG"; do sleep 15; done
PASS_LINE=$(grep "stage=$WAIT_STAGE PASS" "$LOG" | tail -1)
if [ -z "$PASS_LINE" ]; then
  log "段 $WAIT_STAGE は合格しないまま終わった: $(grep chain_done "$LOG" | tail -1)"
  exit 1
fi
touch "$STOP_FILE"
log "段 $WAIT_STAGE が合格した。次の段の前で止める（$STOP_FILE）: $PASS_LINE"
until grep -q "chain_done" "$LOG"; do sleep 15; done
log "$(grep chain_done "$LOG" | tail -1)"
CK="$(echo "$PASS_LINE" | grep -o 'kept=[^ ]*' | cut -d= -f2)/last.pt"
[ -f "$CK" ] || { log "合格の ckpt が無い: $CK"; exit 2; }
log "next chain from $CK stages=$*"
MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$STOP_FILE" \
  ./stage_chain.sh "$CK" "$@"
