#!/usr/bin/env bash
# F3（10-26）の学習し直しを「ストップと言われるまで」続ける見張り（10-26 16:40 ユーザー「ストップというまで継続して学習して」）。
#
# exp_f3_retrain_1026.sh（stage_chain.sh）は、成功率が合格の線を越えるか回数の上限で止まる。止まったら、最後の重みから
# 合格で止まらない形（PASS_RATE=1.1）・回数の上限なし（ROUNDS=9999）で続きを始め直す。
# 止め方: ~/work_inoue/STOP_DOOR_F3 を置く → 今の回の評価まで済ませて学習が止まり、この見張りも続きを始めずに終わる。
#         すぐ止めるなら、この見張りと exp_f3・stage_chain・run_train_segments のプロセスを止める（学習は last.pt まで残る）。
# 落ち続けるとき（10 分以内に終わるのが 3 回続く）・連鎖が「学習が進まない」（stalled）で止まったときは、始め直さずに終わる。
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./f3_keepalive_1026.sh > logs_f3_keepalive_1026.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
STOP_FILE="$HOME/work_inoue/STOP_DOOR_F3"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
running() { ps -eo args | grep -q "[e]xp_f3_retrain_1026.sh"; }
fast_fail=0
log "keepalive start"
while true; do
  while running; do sleep 30; done
  if [ -e "$STOP_FILE" ]; then log "止め札（${STOP_FILE}）があるので続きを始めずに終わる"; exit 0; fi
  L=$(ls -td chain/*/ | head -1)
  if grep -q "chain_done result=stalled" "$L/chain.log" 2>/dev/null; then
    log "前の連鎖が「学習が進まない」（始まるのに重みが増えない）で止まった（${L}）。始め直しても同じなので終わる（反復の上限・起動の失敗を直してから起こす）"
    exit 1
  fi
  LAST=$(grep -o 'last_ckpt=[^ ]*' "$L/chain.log" 2>/dev/null | tail -1 | cut -d= -f2)
  [ -z "$LAST" ] && LAST=$(tail -1 "$L/results.tsv" 2>/dev/null | cut -f7)
  if [ -z "$LAST" ] || [ ! -f "$LAST" ]; then log "最後の重みが見つからない（chain=${L}）。終わる"; exit 1; fi
  START=$(date +%s)
  log "続きを始める ck=${LAST}（前の chain=${L}）"
  CK="$LAST" ROUNDS=9999 PASS_RATE=1.1 ./exp_f3_retrain_1026.sh >> logs_f3_retrain_1026.log 2>&1 < /dev/null
  if [ $(( $(date +%s) - START )) -lt 600 ]; then
    fast_fail=$((fast_fail + 1))
    log "10 分以内に終わった（${fast_fail} 回目）"
    [ "$fast_fail" -ge 3 ] && { log "3 回続けて早く終わったので、始め直さずに終わる。logs_f3_retrain_1026.log を見る"; exit 1; }
  else
    fast_fail=0
  fi
done
