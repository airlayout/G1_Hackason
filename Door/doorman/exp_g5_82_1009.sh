#!/usr/bin/env bash
# 2026-10-09 段 2-3 の案 2: 等級 4 の扉を 8 割・等級 5 の扉を 2 割にして学習する（door_en1154_g4g5_82、台 i % 5 == 4 が等級 5）。
# E3（exp_g5_mix_1009.sh）は 26〜54 N·m 一様で等級 5 が 6 割 → 学習の中の成功が少なすぎ、自動調整が 1.0 に張り付いて等級 4 まで失った。
# 見込み: 34,950 は等級 4 で 81%・等級 5 で 16.5% なので学習の中の成功は 0.8×0.81＋0.2×0.165 ≈ 0.68 で 5 割を超え、張り付かない。
# E3 との違いは学習する扉（混ぜる割合）だけ。点数は E3 と同じ W12（上限 100 N・1 N あたり 0.12）、土台も同じ反復 34,950。
# 合否は純粋な等級 5 の扉で見る（7 つ目の欄）。1 時間を 3 回まで
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/g4_pass_it34950/last.pt"
W12="++env.config.push_door_force_max=100,++env.config.push_unlatched_force_max=100,++rewards.reward_scales.push_door_force=0.12,++rewards.reward_scales.push_door_unlatched=12.0"
EVAL_FIRST=0 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" "g5r82:door_en1154_g4g5_82:3:60:door30:$W12:door_en1154_g5"
