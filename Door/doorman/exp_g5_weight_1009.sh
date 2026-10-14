#!/usr/bin/env bash
# 2026-10-09 段 2-3 の確かめ（実験 E2）: 等級 5 の学習が崩れたのは「押す力の 1 N あたりの点数を下げすぎた」からか。
# 前回（E1、chain/1008_2309 の g5）: 上限 100 N・1 N あたり 0.06（push_door_force 0.06・push_door_unlatched 6/100）→ 21 → 0 → 0 → 0/119
# 今回（E2）: 上限 100 N はそのまま、1 N あたりを等級 3（合格）と同じ 0.12 に戻す（push_door_force 0.12・push_door_unlatched 12/100）。
# 土台も前回と同じ反復 34,950（ckpt/g4_pass_it34950、等級 5 で 21/127）。1 時間を 3 回まで、80% で合格。学習前の評価は前回と同じなので省く
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/g4_pass_it34950/last.pt"
W12="++env.config.push_door_force_max=100,++env.config.push_unlatched_force_max=100,++rewards.reward_scales.push_door_force=0.12,++rewards.reward_scales.push_door_unlatched=12.0"
EVAL_FIRST=0 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" "g5w12:door_en1154_g5:3:60:door30:$W12"
