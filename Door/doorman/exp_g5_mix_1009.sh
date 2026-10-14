#!/usr/bin/env bash
# 2026-10-09 段 2-3 の確かめ（実験 E3）: 等級 5 の扉が 1 度も開かないので、成功から学べないのか（見込み B）。
# E2（chain/1009_0917）: 等級 5 だけで学習・1 N あたり 0.12 → 1 回目・2 回目とも 0/119（押す力は 20 → 17 N）
# E3: 1 N あたりは E2 と同じ 0.12・上限 100 N。学習する扉だけを「等級 4 と 5 を混ぜた扉」（door_en1154_g4g5、26〜54 N·m）に替える。
# 合否は純粋な等級 5 の扉で見る（7 つ目の欄）。土台は E1・E2 と同じ反復 34,950。1 時間を 3 回まで
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
until grep -q "chain_done" chain/1009_0917/chain.log; do sleep 30; done
grep "chain_done" chain/1009_0917/chain.log
CK="$HOME/work_inoue/doorman/ckpt/g4_pass_it34950/last.pt"
W12="++env.config.push_door_force_max=100,++env.config.push_unlatched_force_max=100,++rewards.reward_scales.push_door_force=0.12,++rewards.reward_scales.push_door_unlatched=12.0"
EVAL_FIRST=0 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" "g5mix:door_en1154_g4g5:3:60:door30:$W12:door_en1154_g5"
