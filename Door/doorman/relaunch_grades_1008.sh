#!/usr/bin/env bash
# 2026-10-08 13:08: 等級のチェーン（1008_1306）を 26,050 から始めてしまったので、止めて 25,400（標準の扉で 78.1%）から始め直す
cd "$HOME/work_inoue/doorman" || exit 1
until grep -q "chain_done" chain/1008_1306/chain.log; do sleep 15; done
grep "chain_done" chain/1008_1306/chain.log
EVAL_FIRST=1 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh ckpt/start_1008_1221/last.pt g1:door_en1154_g1:3:60:door30 g2:door_en1154_g2:3:60:door30 \
  g3:door_en1154_g3:3:60:door30 g4:door_en1154_g4:3:60:door30 g5:door_en1154_g5:3:60:door30
