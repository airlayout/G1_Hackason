#!/usr/bin/env bash
# 2026-10-10 段 2-3 の案 3: 学習中の自動調整を止める（倍率 reward_penalty_scale と打ち切りの基準 termination_level を出発点の値に固定）。
# 案 2（exp_g5_82_1009.sh、chain/1009_1613）は倍率が約 650 反復で 1.0 に張り付き、どの扉も「20° ほど開けて止まる」になって等級 4 まで失った（104 → 14/119）。
# 案 2 との違いは「変化の速さ（degree）を 0 にする」だけ。値は ckpt の env_state_dict から戻るので、反復 34,950 の 0.31 のまま動かない。
# ⚠️ reward_penalty_curriculum=False は倍率を掛けなくなる（＝1.0 相当）ので固定にならない。degree=0 で止める。
# 扉（door_en1154_g4g5_82）・点数（W12）・土台（34,950）・合否を見る扉（純粋な等級 5）は案 2 と同じ。1 時間を 3 回まで
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/g4_pass_it34950/last.pt"
W12="++env.config.push_door_force_max=100,++env.config.push_unlatched_force_max=100,++rewards.reward_scales.push_door_force=0.12,++rewards.reward_scales.push_door_unlatched=12.0"
FIX="++rewards.reward_penalty_degree=0,++env.config.termination_curriculum.degree=0"
EVAL_FIRST=0 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" "g5fix:door_en1154_g4g5_82:3:60:door30:$W12,$FIX:door_en1154_g5"
