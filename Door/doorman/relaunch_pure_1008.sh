#!/usr/bin/env bash
# 2026-10-08 16:30: 段 2-3 を純粋な等級の扉の学習に戻す。
# 易しい扉を混ぜた扉（door_en1154_upto_g1）で 1 回学習すると、等級 1 は 50/128 → 5/128 に落ちた。
# 易しい扉は弱い押しで開いて完了の報酬が取れるので、前へ押す力の報酬が 0.47〜0.87 → 0.22〜0.43 に下がった。
# 純粋な等級 1 の 2 回目（反復 27,600、ckpt/start_1008_1514）は 0/119 → 50/128 に伸びていたので、そこから等級 1〜5 を回し直す。
# 等級 3〜5 は前へ押す力の報酬の頭打ちを 50・75・100 N に上げ、重みを下げて 1 ステップの上限を保つ（パッチ 0031）
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/start_1008_1514/last.pt"
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
CAP4="++env.config.push_door_force_max=75,++env.config.push_unlatched_force_max=75,++rewards.reward_scales.push_door_force=0.08"
CAP5="++env.config.push_door_force_max=100,++env.config.push_unlatched_force_max=100,++rewards.reward_scales.push_door_force=0.06"
EVAL_FIRST=1 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" \
  "g1:door_en1154_g1:3:60:door30" \
  "g2:door_en1154_g2:3:60:door30" \
  "g3:door_en1154_g3:3:60:door30:$CAP3" \
  "g4:door_en1154_g4:3:60:door30:$CAP4" \
  "g5:door_en1154_g5:3:60:door30:$CAP5"
