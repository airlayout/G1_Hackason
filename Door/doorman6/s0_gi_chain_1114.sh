#!/usr/bin/env bash
# 11-14（段 G・I・F の残り）: 3a までは実機の pelvis の動きを再生し（0113）、3c に入ったら放して（0113d release_sub）純正に似せた脚（0095）
#   ＋押されて下がる脚（0114）で歩かせる。HOMIE の脚だけだと 3a で手の反動に耐えられず 32 台中 0 台しか 3c に入れなかった（11-14 g_ga_pushoff）。
#   G: 11-13 朝の 3c の代表 3 形（1・8・15 回目の形と、その回の 3a の動き）を、押されて下がる口の有り・無しで。
#   I（⚠️ 推測）: 段 4 の入り（4a）。3a の動きは足踏みの 15・16 回目（半身は 19・21 回目）。クローザーの仮定 3 通り・前の速さ・半身。
#   F の残り: 足踏みを 2d から（f4w2d）・当てる点 4 cm で経由点据え置き（f1via）。止め札 STOP_REPLAY_1114 で次へ進まない
# 使い方: setsid nohup ./s0_gi_chain_1114.sh > s0/gi_chain_1114.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
RS="$HOME/work_inoue/doorman/real_script_1114"
RESP_EVAL=$(grep -A1 '評価の引数' "$HOME/work_inoue/doorman/fit_loco_resp_1107.txt" | tail -1)
PUSH="++env.config.loco_push.enabled=True ++env.config.loco_push.f_th=30.0 ++env.config.loco_push.v_back=-0.3 ++env.config.loco_push.t_back=0.6 ++env.config.loco_push.sway_vy=-0.25 ++env.config.loco_push.sway_wz=0.1 ++env.config.loco_push.sway_s=0.5"
REL="++env.config.pelvis_replay.release_sub=3c"
LEG="[-0.42,-0.37,-0.346,-0.306,-6.0,-4.0]"
TW="[-0.49,-0.44,-0.186,-0.136,-6.0,-2.0]"
stop() { [ -e "$HOME/work_inoue/STOP_REPLAY_1114" ] && { echo "=== 止め札があるので止める"; exit 0; }; }
# 名前:場面:定数:押されて下がる:半身:置く所:再生する回:回の長さ
for G in "g_ga_off:door_room4_fit_1114:script_params_ga_sim.json:off:0:$LEG:1:45" \
         "g_ga_on:door_room4_fit_1114:script_params_ga_sim.json:on:0:$LEG:1:45" \
         "g_gb_off:door_room4_fit_1114:script_params_gb_sim.json:off:0:$LEG:8:45" \
         "g_gb_on:door_room4_fit_1114:script_params_gb_sim.json:on:0:$LEG:8:45" \
         "g_gc_off:door_room4_fit_1114:script_params_gc_sim.json:off:0:$LEG:15:45" \
         "g_gc_on:door_room4_fit_1114:script_params_gc_sim.json:on:0:$LEG:15:45" \
         "i_c02on:door_room4_fit_1114:script_params_i_dy4pass_sim.json:on:0:$LEG:15,16:75" \
         "i_c02off:door_room4_fit_1114:script_params_i_dy4pass_sim.json:off:0:$LEG:15,16:75" \
         "i_l02on:door_room4_fit_1114_light:script_params_i_dy4pass_sim.json:on:0:$LEG:15,16:75" \
         "i_h02on:door_room4_fit_1114_heavy:script_params_i_dy4pass_sim.json:on:0:$LEG:15,16:75" \
         "i_c03on:door_room4_fit_1114:script_params_i_v03_sim.json:on:0:$LEG:15,16:75" \
         "i_ctwon:door_room4_fit_1114:script_params_i_twist_sim.json:on:1:$TW:19,21:75"; do
  stop
  IFS=: read -r name task params push twist range labels ep <<< "$G"
  EXTRA="$RESP_EVAL $REL"
  [ "$push" = on ] && EXTRA="$EXTRA $PUSH"
  N=32 TAGNAME="$name" TASK="$task" RS="$RS" PARAMS="$params" METHOD=vel LABELS="$labels" START_SUB=3a HOLD_FROM=2a EP_S="$ep" \
    TWIST="$twist" START_RANGE="$range" MORE="$EXTRA" MINUTES=25 ./s0_replay_1114.sh
done
stop
N=48 TAGNAME="f_f4w2d" RS="$RS" PARAMS=script_params_f4w2d_sim.json LABELS=15,16,19,21 METHOD=vel START_SUB=2d HOLD_FROM=2a \
  START_RANGE="$LEG" MORE="++env.config.pelvis_replay.offset_s=0.0" MINUTES=20 ./s0_replay_1114.sh
stop
N=48 TAGNAME="f_f1via" RS="$RS" PARAMS=script_params_f1via_sim.json LABELS=15,16,19,21 METHOD=vel START_SUB=3a HOLD_FROM=2a \
  START_RANGE="$LEG" MORE="++env.config.pelvis_replay.offset_s=0.0" MINUTES=20 ./s0_replay_1114.sh
echo "=== [$(date "+%m-%d %T")] gi chain 終わり"
