#!/usr/bin/env bash
# 11-14（段 F・計画 8 節）: 同じ体の揺れ（3a 足踏みの 15・16・19・21 回目の pelvis を再生）の下で、形を 1 つずつ変えて比べる。
#   WAIT_FOR の続き（s0/replay_chain_<TAG>.log の「終わり」）を待ってから始める。台本は real_script_1114（4a を足した版）
#   b（11-14）: 置く所を実機の立ち位置に（s0_replay_chain_1114b.sh と同じ）
# 使い方: WAIT_FOR=r3 setsid nohup ./s0_f_chain_1114b.sh > s0/f_chain_1114.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
if [ -n "${WAIT_FOR:-}" ]; then
  until grep -q "chain ${WAIT_FOR} 終わり\|止め札" "s0/replay_chain_${WAIT_FOR}.log" 2>/dev/null; do sleep 30; done
fi
RS="$HOME/work_inoue/doorman/real_script_1114"
L="${LABELS:-15,16,19,21}"
for G in "f0:script_params_f0_sim.json:3a:0.0" "f1dy4:script_params_f1dy4_sim.json:3a:0.0" "f2h30:script_params_f2h30_sim.json:3a:0.0" \
         "f2h35:script_params_f2h35_sim.json:3a:0.0" "f3p5:script_params_f3p5_sim.json:3a:0.0" "f4walk2d:script_params_f0_sim.json:2d:1.0" \
         "f1real:script_params_f1real_dy4_sim.json:3a:0.0"; do
  [ -e "$HOME/work_inoue/STOP_REPLAY_1114" ] && { echo "=== 止め札があるので止める"; exit 0; }
  IFS=: read -r name params start off <<< "$G"
  N=48 TAGNAME="f_${name}" RS="$RS" PARAMS="$params" LABELS="$L" METHOD=vel START_SUB="$start" HOLD_FROM=2a \
    START_RANGE="[-0.42,-0.37,-0.346,-0.306,-6.0,-4.0]" MORE="++env.config.pelvis_replay.offset_s=$off" MINUTES=20 ./s0_replay_1114.sh
done
echo "=== [$(date "+%m-%d %T")] f chain 終わり"
