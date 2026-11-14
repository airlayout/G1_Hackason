#!/usr/bin/env bash
# 11-14（段 I・5 回目・弱い脚）: 3a までは実機の pelvis の動きを再生し、3c からは pelvis を「純正の脚の応え方」で動かす
#   （0113e cmd_follow・⚠️ 推測: 台本の歩きの指令 → 遅れ 0.26 秒・倍率 0.6・0.1 m/s 未満は動かない → pelvis の速さ。手の押し返しが
#   f_th を越えたら 0.5 秒 0.2 m/s で下がる。3a を脚を止めて回した組は、3c の歩き出しに実機の足踏みの歩き出しの揺れ（15 回目の記録の
#   最初の 0.8 秒の横・向き）を足す）。HOMIE の脚では 3c の歩き出しに 4〜9 cm 下がって前へ出なかった（g_ga_off）。
#   3 回目: 押し返しの効き（押すほど前へ出ない fwd_per_n 0.01 m/s/N・把手の側へ回される turn_per_n 0.014 rad/s/N）を入れた。
#   入れないと扉は 3 秒で 30° 開いた（実機の 1 回目は 8°・g2_ga）。⚠️ 効きの値は 11-13 朝の 1・15 回目からの大まかな合わせ
#   4 回目: 押し返しの力をレバーと手の接触センサから取る（cmd_follow.src contact・0113g）。3 回目は台本の見積もりで動かしたが、
#   シムの 3c では見積もりが本当の力より約 10 N 片寄り、押していないのに体を 73° 回した（g3_ga）。i4_c02s は押し返しの効きなし（強い脚）
#   5 回目: 段 I を「弱い脚」（押し返しは台本の見積もり src est・効きあり。シムの見積もりは 3c で約 10 N 片寄るので、押していない時も
#   下がり・回る＝下から挟む形）で回す。4 回目の接触センサ（レバーだけを見る）は 3c で手が板を押すと 0 で、強い脚と同じになった（g4_ga）
#   WAIT_FOR の続きの「終わり」か止め札を待ってから、止め札を外して始める
# 使い方: WAIT_FOR=gi4 setsid nohup ./s0_gi5_chain_1114.sh > s0/gi5_chain_1114.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
if [ -n "${WAIT_FOR:-}" ]; then
  until grep -q "${WAIT_FOR} chain 終わり\|止め札" "s0/${WAIT_FOR}_chain_1114.log" 2>/dev/null; do sleep 20; done
  while pgrep -f "eval_agent_trl" > /dev/null; do sleep 10; done
fi
rm -f "$HOME/work_inoue/STOP_REPLAY_1114"
RS="$HOME/work_inoue/doorman/real_script_1114"
CMD="++env.config.pelvis_replay.release_sub=3c ++env.config.pelvis_replay.cmd_follow.enabled=True ++env.config.pelvis_replay.cmd_follow.gain=0.6 ++env.config.pelvis_replay.cmd_follow.delay_s=0.26 ++env.config.pelvis_replay.cmd_follow.dead=0.1 ++env.config.pelvis_replay.cmd_follow.back_v=-0.2 ++env.config.pelvis_replay.cmd_follow.back_s=0.5 ++env.config.pelvis_replay.cmd_follow.src=est"
SWAY="++env.config.pelvis_replay.cmd_follow.sway_label=15 ++env.config.pelvis_replay.cmd_follow.sway_s=0.8"
LEG="[-0.42,-0.37,-0.346,-0.306,-6.0,-4.0]"
TW="[-0.49,-0.44,-0.186,-0.136,-6.0,-2.0]"
stop() { [ -e "$HOME/work_inoue/STOP_REPLAY_1114" ] && { echo "=== 止め札があるので止める"; exit 0; }; }
# 名前:場面:定数:押し返しの線 N:揺れ:半身:置く所:再生する回:回の長さ
for G in "i5_c02w:door_room4_fit_1114:script_params_i_dy4pass_sim.json:30:0:0:$LEG:15,16:75:0.014:0.01" \
         "i5_h02w:door_room4_fit_1114_heavy:script_params_i_dy4pass_sim.json:30:0:0:$LEG:15,16:75:0.014:0.01" \
         "i5_l02w:door_room4_fit_1114_light:script_params_i_dy4pass_sim.json:30:0:0:$LEG:15,16:75:0.014:0.01"; do
  stop
  IFS=: read -r name task params fth sway twist range labels ep turn fwd <<< "$G"
  EXTRA="$CMD ++env.config.pelvis_replay.cmd_follow.f_th=$fth ++env.config.pelvis_replay.cmd_follow.turn_per_n=$turn ++env.config.pelvis_replay.cmd_follow.fwd_per_n=$fwd"
  [ "$sway" = 1 ] && EXTRA="$EXTRA $SWAY"
  N=32 TAGNAME="$name" TASK="$task" RS="$RS" PARAMS="$params" METHOD=vel LABELS="$labels" START_SUB=3a HOLD_FROM=2a EP_S="$ep" \
    TWIST="$twist" START_RANGE="$range" MORE="$EXTRA" MINUTES=25 ./s0_replay_1114.sh
done
echo "=== [$(date "+%m-%d %T")] gi5 chain 終わり"
