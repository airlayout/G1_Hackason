#!/usr/bin/env bash
# 11-14（段 F の 4・計画 8 節）: 足踏みを 2d から（台本 walk_from "2d"）。体の動きは 2d の入りから、実機の足踏みの 3a の記録（歩き出しから）を
#   再生する（15・16・19・21 回目）。比べる相手は s0_f_chain_1114b.sh の f0（3a から足踏み）。WAIT_FOR の続きの終わりを待って始める
# 使い方: WAIT_FOR=i setsid nohup ./s0_f4_chain_1114.sh > s0/f4_chain_1114.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
if [ -n "${WAIT_FOR:-}" ]; then
  until grep -q "${WAIT_FOR} chain 終わり\|止め札" "s0/${WAIT_FOR}_chain_1114.log" 2>/dev/null; do sleep 30; done
fi
[ -e "$HOME/work_inoue/STOP_REPLAY_1114" ] && { echo "=== 止め札があるので止める"; exit 0; }
N=48 TAGNAME="f_f4w2d" RS="$HOME/work_inoue/doorman/real_script_1114" PARAMS=script_params_f4w2d_sim.json LABELS=15,16,19,21 METHOD=vel \
  START_SUB=2d HOLD_FROM=2a START_RANGE="[-0.42,-0.37,-0.346,-0.306,-6.0,-4.0]" MORE="++env.config.pelvis_replay.offset_s=0.0" MINUTES=20 ./s0_replay_1114.sh
echo "=== [$(date "+%m-%d %T")] f4 chain 終わり"
