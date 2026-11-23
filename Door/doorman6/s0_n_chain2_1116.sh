#!/usr/bin/env bash
# 11-16（計画 docs/plan/2026-11-16-g1-door-push-line-two-hands-sim.md・⚠️ シムだけ）: 3c の押し方の新案（力の線を体の中心に通す・
#   両手で打ち消す・柔らかい腕）。3a までは実機の足踏みの再生、3c からは pelvis を純正の応え方で動かし（cmd_follow）、一歩ごとの揺れ
#   （前後 1.3 cm・横 1.8 cm・向き 2°・1 Hz。11-13 実機の 3c の 90% の振れ）を足す。板の接触センサで体を回す力 Mz を測る（パッチ 0115）
# 組の書き方（環境変数 RUNS に空白区切り）: 名前:定数:半身(0/1):柔らかい腕の倍率(0 は切):右手(0/1):脚(strong/p07/p20):台数:蝶番の角度を台本へ(0/1)
#   （s0_n_chain_1116.sh に最後の欄を足した写し。走っている台本を上書きしないため別の名前）
#   脚 strong＝押されても止まらず回されない／p07・p20＝本当の押し返しで前が遅れ（0.01 m/s/N）、Mz で回される（0.07・0.20 rad/s/(N·m)）・
#   押し返し 30 N で 0.5 秒下がる
# 使い方: RUNS="n_b0:script_params_i_viapass_sim.json:0:0:0:strong:32 …" setsid nohup ./s0_n_chain2_1116.sh > s0/n3_chain_1116.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
if [ -n "${WAIT_FOR:-}" ]; then
  until grep -q "${WAIT_FOR} 終わり\|止め札" "s0/${WAIT_FOR}.log" 2>/dev/null; do sleep 20; done
  while pgrep -f "eval_agent_trl" > /dev/null; do sleep 10; done
fi
rm -f "$HOME/work_inoue/STOP_REPLAY_1114"
RS="$HOME/work_inoue/doorman/real_script_1114"
CMD="++env.config.pelvis_replay.release_sub=3c ++env.config.pelvis_replay.cmd_follow.enabled=True ++env.config.pelvis_replay.cmd_follow.gain=0.6 ++env.config.pelvis_replay.cmd_follow.delay_s=0.26 ++env.config.pelvis_replay.cmd_follow.dead=0.1 ++env.config.pelvis_replay.cmd_follow.back_v=-0.2 ++env.config.pelvis_replay.cmd_follow.back_s=0.5 ++env.config.pelvis_replay.cmd_follow.src=panel"
OSC="++env.config.pelvis_replay.cmd_follow.osc_x=${OSC_X:-0.013} ++env.config.pelvis_replay.cmd_follow.osc_y=${OSC_Y:-0.018} ++env.config.pelvis_replay.cmd_follow.osc_yaw_deg=${OSC_YAW:-2.0} ++env.config.pelvis_replay.cmd_follow.osc_hz=1.0"
PANEL="++env.config.panel_contact.enabled=True ++env.config.script_teacher.panel=True"
LEG="[-0.42,-0.37,-0.346,-0.306,-6.0,-4.0]"
TW="[-0.49,-0.44,-0.186,-0.136,-6.0,-2.0]"
stop() { [ -e "$HOME/work_inoue/STOP_REPLAY_1114" ] && { echo "=== 止め札があるので止める"; exit 0; }; }
for G in ${RUNS}; do
  stop
  IFS=: read -r name params twist soft helper leg n ext <<< "$G"
  case "$leg" in
    strong) L="++env.config.pelvis_replay.cmd_follow.f_th=1000 ++env.config.pelvis_replay.cmd_follow.fwd_per_n=0.0 ++env.config.pelvis_replay.cmd_follow.turn_per_nm=0.0" ;;
    p07) L="++env.config.pelvis_replay.cmd_follow.f_th=30 ++env.config.pelvis_replay.cmd_follow.fwd_per_n=0.01 ++env.config.pelvis_replay.cmd_follow.turn_per_nm=0.07" ;;
    p20) L="++env.config.pelvis_replay.cmd_follow.f_th=30 ++env.config.pelvis_replay.cmd_follow.fwd_per_n=0.01 ++env.config.pelvis_replay.cmd_follow.turn_per_nm=0.20" ;;
    *) echo "脚の名前がおかしい: $leg"; exit 1 ;;
  esac
  EXTRA="$CMD $OSC $PANEL $L"
  [ "$soft" != 0 ] && EXTRA="$EXTRA ++env.config.script_teacher.comply.enabled=True ++env.config.script_teacher.comply.kp_scale=$soft"
  [ "$helper" = 1 ] && EXTRA="$EXTRA ++env.config.script_teacher.helper_arm=True"
  [ "${ext:-0}" = 1 ] && EXTRA="$EXTRA ++env.config.script_teacher.door_angle=True"
  if [ "$twist" = 1 ]; then range="$TW"; labels=19,21; else range="$LEG"; labels=15,16; fi
  N="$n" TAGNAME="$name" TASK=door_room4_fit_1114 RS="$RS" PARAMS="$params" METHOD=vel LABELS="$labels" START_SUB=3a HOLD_FROM=2a \
    EP_S="${EP_S:-75}" TWIST="$twist" START_RANGE="$range" MORE="$EXTRA" MINUTES="${MINUTES:-25}" ./s0_replay_1114.sh
done
echo "=== [$(date "+%m-%d %T")] ${CHAIN_NAME:-n_chain_1116} 終わり"
