#!/usr/bin/env bash
# 2026-10-09 段 2-3 の確かめ（実験 E4・対照）: 反復 34,950 から学習を続けると、何をしても崩れるのか（見込み D）、
# それとも E1〜E3 の「上限 100 N」の点数の付け方が崩しているのか（見込み C）。
# E1〜E3 は 34,950 から上限 100 N で学習して、1 時間で等級 5 の 21 回の成功が 0 になり、E3 では等級 4 も 104 → 0/119 になった。
# E4: 34,950 を、成功した等級 3 の学習（10-08 23:14〜、14% → 89%）とまったく同じ設定（上限 50 N・push_door_force 0.12・
# push_door_unlatched は既定 6）と同じ等級 3 の扉で 1 時間学習する。等級 3 の 114/128 が保たれれば D ではない
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/g4_pass_it34950/last.pt"
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
EVAL_FIRST=0 PASS_RATE=1.01 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" "c3:door_en1154_g3:1:60:door30:$CAP3"
