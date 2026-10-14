#!/usr/bin/env bash
# 2026-10-08 19:45: 段 2-3 の延長。等級ごとに 3 回の予算では等級 1 に届かなかった（50 → 42 → 79 → 89/128、chain/1008_1625）。
# 決まりどおりの答え（等級 1 未満・80% で開けられる上限 約 9.9 N·m）は残したうえで、伸び続けているので等級 1 をあと 3 回まで続け、
# 合格すれば等級 2〜5 へ進む。土台は反復 30,750（ckpt/start_1008_1934。標準の扉で 113/128＝段 2-2 に合格した ckpt と同じ）。
# 等級 1 の続きは段の名前を g1b にする（合格の ckpt は ckpt/g1b_pass_it<反復>/）。等級 3〜5 は押す力の報酬の頭打ちを上げる（0031）
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/start_1008_1934/last.pt"
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
CAP4="++env.config.push_door_force_max=75,++env.config.push_unlatched_force_max=75,++rewards.reward_scales.push_door_force=0.08"
CAP5="++env.config.push_door_force_max=100,++env.config.push_unlatched_force_max=100,++rewards.reward_scales.push_door_force=0.06"
EVAL_FIRST=1 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" \
  "g1b:door_en1154_g1:3:60:door30" \
  "g2:door_en1154_g2:3:60:door30" \
  "g3:door_en1154_g3:3:60:door30:$CAP3" \
  "g4:door_en1154_g4:3:60:door30:$CAP4" \
  "g5:door_en1154_g5:3:60:door30:$CAP5"
