#!/usr/bin/env bash
# 2026-10-09 段 2-3 の確かめ（実験 E5）: E1〜E3 の崩れは「上限 100 N の点数の付け方」のせいか（見込み C）、
# 「等級 5 の扉が開かない → 学習中の成功が減る → 誘導の点数の倍率と打ち切りの基準が『失敗中』の値に張り付く → 崩れる」悪い循環か（見込み B）。
# E5: 点数の付け方は合格した等級 3 の学習と同じ（上限 50 N・push_door_force 0.12・push_door_unlatched は既定 6）のまま、扉だけ等級 5 にする。
# 土台は E1〜E4 と同じ反復 34,950。1 時間を 1 回。合否は見ず（PASS_RATE 1.01）、等級 5 の成功と押す力が保たれるかを見る
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
until grep -q "chain_done" "$(ls -td chain/*/ | head -1)chain.log"; do sleep 30; done
CK="$HOME/work_inoue/doorman/ckpt/g4_pass_it34950/last.pt"
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
EVAL_FIRST=0 PASS_RATE=1.01 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" "c5:door_en1154_g5:1:60:door30:$CAP3"
