#!/usr/bin/env bash
# 2026-10-08 14:40: 段 2-3 を「易しい扉を混ぜて学習し、純粋な等級の扉で合否を見る」に切り替える（ユーザー判断）。
# 純粋な等級 1 で 1 回学習すると、レバーを下げて扉が 2° 以上動いた回が 27 → 4/128 に減った（誰も開けられず、開ける押しの手がかりが無い）。
# 走っている等級のチェーン（1008_1309）は止め札で 2 回目の評価の後に止まる。止まったら、
# 2 回目が等級 1 で明らかに良ければ（扉 30° が MIN_BETTER 回以上）その ckpt から、そうでなければ反復 25,400（標準の扉で 78.1%）から始める。
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
OLD="chain/1008_1309"
BEST_DEFAULT="$HOME/work_inoue/doorman/ckpt/start_1008_1221/last.pt"
MIN_BETTER="${MIN_BETTER:-10}"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

until grep -q "chain_done" "$OLD/chain.log"; do sleep 15; done
DONE=$(grep "chain_done" "$OLD/chain.log" | tail -1)
log "$DONE"
LAST=$(echo "$DONE" | grep -o "last_ckpt=[^ ]*" | cut -d= -f2)
LAST_SUCC=$(awk -F'\t' '$1=="g1" {s=$5} END {split(s, a, "/"); print a[1] + 0}' "$OLD/results.tsv")
if [ "$LAST_SUCC" -ge "$MIN_BETTER" ] && [ -f "$LAST" ]; then CK="$LAST"; else CK="$BEST_DEFAULT"; fi
log "等級 1 の最後の評価の成功 $LAST_SUCC（$MIN_BETTER 以上ならその ckpt）→ 始める ckpt: $CK"

CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
CAP4="++env.config.push_door_force_max=75,++env.config.push_unlatched_force_max=75,++rewards.reward_scales.push_door_force=0.08"
CAP5="++env.config.push_door_force_max=100,++env.config.push_unlatched_force_max=100,++rewards.reward_scales.push_door_force=0.06"
EVAL_FIRST=1 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" \
  "m1:door_en1154_upto_g1:3:60:door30::door_en1154_g1" \
  "m2:door_en1154_upto_g2:3:60:door30::door_en1154_g2" \
  "m3:door_en1154_upto_g3:3:60:door30:$CAP3:door_en1154_g3" \
  "m4:door_en1154_upto_g4:3:60:door30:$CAP4:door_en1154_g4" \
  "m5:door_en1154_upto_g5:3:60:door30:$CAP5:door_en1154_g5"
