#!/usr/bin/env bash
# 指定したチェーン（chain/<日時>）が終わったら、そのチェーンの最後のチェックポイントから、渡した段の並びで次のチェーンを起動する。
# 使い方: ./after_chain.sh <待つ chain の日時> <段>...（stage_chain.sh の段の書き方）
#   REQUIRE=all_pass: 待ったチェーンの結果がこれでなければ起動しない（段 2-2 に合格したときだけ等級 1〜5 へ進む）
#   EVAL_FIRST などの stage_chain.sh の環境変数はそのまま渡る
# 2026-10-08: 段 2-2 の合格の線を「レバーを下げて扉を 30° 以上」（door30）に変えたときの引き継ぎに使った
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
LOG="chain/${1:?chain の日時}/chain.log"; shift
until grep -q "chain_done" "$LOG"; do sleep 60; done
DONE=$(grep "chain_done" "$LOG" | tail -1)
RES=$(echo "$DONE" | grep -o "result=[^ ]*" | cut -d= -f2)
echo "=== [$(date '+%m-%d %T')] chain_done result=$RES"
if [ -n "${REQUIRE:-}" ] && [ "$RES" != "$REQUIRE" ]; then
  echo "=== [$(date '+%m-%d %T')] result が $REQUIRE でないので次のチェーンは起動しない"
  exit 1
fi
# 待ったチェーンの最後のチェックポイント（合格ならその ckpt）。書いていない古いログだけ、段 2-2 の最新の last.pt
CK=$(echo "$DONE" | grep -o "last_ckpt=[^ ]*" | cut -d= -f2)
[ -f "$CK" ] || CK=$(ls -t "$HOME"/work_inoue/doorman6/VIRAL6/logs_rl/door_s22/wbmanip/*/last.pt | head -1)
echo "=== [$(date '+%m-%d %T')] next chain from $CK stages=$*"
MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" "$@"
