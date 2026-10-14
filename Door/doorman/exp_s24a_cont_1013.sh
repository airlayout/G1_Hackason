#!/usr/bin/env bash
# 段 2-4a の続き（2026-10-13 18:10）。exp_s24a_1013.sh の 4 回で 55.5 → 69.5 → 41.4 → 52.3 → 79.7%（合格の線 80% まであと 1 回）。
# 同じ設定のまま、4 回目の重み（反復 39,150）からあと最大 2 回。変えるのは回数と出発点だけ。元の説明:
# 段 2-4a（2026-10-13 のユーザー判断「段 2-4 の学習を始めて」）。ばらつきは 1 つずつ足すので、まず段 2-4 の決まりだけを入れる:
#   扉 door_en1154_g13_shaped（等級 1〜3 を台ごとに 1/3 ずつ・実物の形のクローザー・寸法と重さとレバーと向きは段 2-3 のまま）、
#   通り抜けは扉 70° 以上（door_through_angle_deg）、90° を越えると開いたまま止まるので失敗（door_hold_open_angle_deg）
# 出発点は反復 34,950（段 2-3 で等級 4 まで合格した重み）。報酬は 34,950 を作った学習と同じ（押す力の頭打ち 50 N）。
# 学習前の評価（この決まりで）: 通り抜け 71/128（55.5%）、区間 5 に届いた 95（eval_s24a_r0.log、records/start_1013_1350 付近）。
# 合格は同じ扉で 128 回中 80% が扉の先 1.5 m まで通り抜ける。1 回 60 分 × 最大 4 回、合格で止まる。
# OMEN の前の人（ユーザー本人）がいても回す（IDLE_GATE=0、10-13 のユーザー判断）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_s24a_cont_1013.sh > exp_s24a_cont_1013.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/s24a_r4_it39150/last.pt"
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
RULES="++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90"
IDLE_GATE=0 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" "s24a:door_en1154_g13_shaped:2:60:through:$CAP3,$RULES"
