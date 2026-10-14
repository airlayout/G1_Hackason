#!/usr/bin/env bash
# 指定したチェーン（chain/<日時>）が段 2-2 で不合格のまま終わったら、最新の last.pt から同じ段の並びで起動し直す（2026-10-08 の夜間用）
# 使い方: ./relaunch_chain.sh <chain の日時（例 1008_0157）>
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
LOG="chain/${1:?chain の日時}/chain.log"
until grep -q "chain_done" "$LOG"; do sleep 60; done
RES=$(grep -o "chain_done result=[^ ]*" "$LOG" | tail -1)
echo "=== [$(date '+%m-%d %T')] chain $1: $RES"
case "$RES" in
  *fail_at_s22*)
    CK=$(ls -t "$HOME"/work_inoue/doorman6/VIRAL6/logs_rl/door_s22/wbmanip/*/last.pt | head -1)
    echo "=== [$(date '+%m-%d %T')] relaunch from $CK"
    MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
      ./stage_chain.sh "$CK" s22:door:6:60 g1:door_en1154_g1:3:60 g2:door_en1154_g2:3:60 g3:door_en1154_g3:3:60 \
      g4:door_en1154_g4:3:60 g5:door_en1154_g5:3:60 ;;
  *) echo "=== 起動し直さない" ;;
esac
