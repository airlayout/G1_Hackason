#!/usr/bin/env bash
# 11-14（段 I・計画 13.4・⚠️ 推測）: 合わせた扉で、3c〜段 4 の入り（4a・pass_through）を、純正に似せた脚（0095）＋押されて下がる脚（0114）で回す。
#   WAIT_FOR の続きの「終わり」を待ってから始める。クローザーの仮定 3 通り（計画 13.5）・前の速さ・半身・押されて下がる口の有無
# 使い方: WAIT_FOR=g setsid nohup ./s0_i_chain_1114.sh > s0/i_chain_1114.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
if [ -n "${WAIT_FOR:-}" ]; then
  until grep -q "${WAIT_FOR} chain 終わり\|止め札" "s0/${WAIT_FOR}_chain_1114.log" 2>/dev/null; do sleep 30; done
fi
RS="$HOME/work_inoue/doorman/real_script_1114"
RESP_EVAL=$(grep -A1 '評価の引数' "$HOME/work_inoue/doorman/fit_loco_resp_1107.txt" | tail -1)
PUSH="++env.config.loco_push.enabled=True ++env.config.loco_push.f_th=${F_TH:-30.0} ++env.config.loco_push.v_back=${V_BACK:--0.3} ++env.config.loco_push.t_back=0.6 ++env.config.loco_push.sway_vy=${SWAY_VY:--0.25} ++env.config.loco_push.sway_wz=0.1 ++env.config.loco_push.sway_s=0.5"
LEG="[-0.37,-0.32,-0.346,-0.306,-6.0,-4.0]"
TW="[-0.44,-0.39,-0.186,-0.136,-6.0,-2.0]"
for G in "c02on:door_room4_fit_1114:script_params_i_dy4pass_sim.json:on:0:$LEG" \
         "c02off:door_room4_fit_1114:script_params_i_dy4pass_sim.json:off:0:$LEG" \
         "l02on:door_room4_fit_1114_light:script_params_i_dy4pass_sim.json:on:0:$LEG" \
         "h02on:door_room4_fit_1114_heavy:script_params_i_dy4pass_sim.json:on:0:$LEG" \
         "c03on:door_room4_fit_1114:script_params_i_v03_sim.json:on:0:$LEG" \
         "ctwon:door_room4_fit_1114:script_params_i_twist_sim.json:on:1:$TW"; do
  [ -e "$HOME/work_inoue/STOP_REPLAY_1114" ] && { echo "=== 止め札があるので止める"; exit 0; }
  IFS=: read -r name task params push twist range <<< "$G"
  EXTRA="$RESP_EVAL"
  [ "$push" = on ] && EXTRA="$EXTRA $PUSH"
  N=32 TAGNAME="i_${name}" TASK="$task" RS="$RS" PARAMS="$params" METHOD=off EP_S=75 TWIST="$twist" START_RANGE="$range" \
    MORE="$EXTRA" MINUTES=25 ./s0_replay_1114.sh
done
echo "=== [$(date "+%m-%d %T")] i chain 終わり"
