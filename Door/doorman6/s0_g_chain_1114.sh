#!/usr/bin/env bash
# 11-14（段 G・計画 13.1・13.2）: 11-13 朝の 3c の代表 3 形を、純正に似せた脚（0095 の応え方の層）で、押されて下がる口（0114 の
#   loco_push）の有り・無しで回す（再生なし）。WAIT_FOR の続きの「終わり」を待ってから始める
# 使い方: WAIT_FOR=f setsid nohup ./s0_g_chain_1114.sh > s0/g_chain_1114.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
if [ -n "${WAIT_FOR:-}" ]; then
  until grep -q "${WAIT_FOR} chain 終わり\|止め札" "s0/${WAIT_FOR}_chain_1114.log" 2>/dev/null; do sleep 30; done
fi
RS="$HOME/work_inoue/doorman/real_script_1114"
RESP_EVAL=$(grep -A1 '評価の引数' "$HOME/work_inoue/doorman/fit_loco_resp_1107.txt" | tail -1)
PUSH="++env.config.loco_push.enabled=True ++env.config.loco_push.f_th=${F_TH:-30.0} ++env.config.loco_push.v_back=${V_BACK:--0.3} ++env.config.loco_push.t_back=0.6 ++env.config.loco_push.sway_vy=${SWAY_VY:--0.25} ++env.config.loco_push.sway_wz=0.1 ++env.config.loco_push.sway_s=0.5"
for G in "ga:script_params_ga_sim.json" "gb:script_params_gb_sim.json" "gc:script_params_gc_sim.json"; do
  IFS=: read -r name params <<< "$G"
  for P in off on; do
    [ -e "$HOME/work_inoue/STOP_REPLAY_1114" ] && { echo "=== 止め札があるので止める"; exit 0; }
    EXTRA="$RESP_EVAL"
    [ "$P" = on ] && EXTRA="$EXTRA $PUSH"
    N=32 TAGNAME="g_${name}_push${P}" RS="$RS" PARAMS="$params" METHOD=off EP_S=40 \
      START_RANGE="[-0.40,-0.34,-0.365,-0.285,-6.0,6.0]" MORE="$EXTRA" MINUTES=20 ./s0_replay_1114.sh
  done
done
echo "=== [$(date "+%m-%d %T")] g chain 終わり"
