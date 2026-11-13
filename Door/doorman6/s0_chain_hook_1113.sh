#!/usr/bin/env bash
# 11-13 夜: 手首を下げてレバーに掛ける形（door_script の hook_*）を、掛ける 0°・35° で比べる（Room 4 の寸法の扉・32 台ずつ・順に）
#   定数は手元の script_params.json（半身）・script_params_run1.json（1 回目の再現）＋シムの緩め（via_tol 0.08・reach_tol 0.04・
#   raise_stall_s 5・pelvis_move_max 0.15・lean 0）。TRACE=32 で全台の毎周期（接触の割合・前への力を回ごとに出す）
cd "$HOME/work_inoue/doorman" || exit 1
export TASK=door_room4_like_left_realwall N=32 TRACE=32 MORE="++env.config.script_teacher.tau_src=applied"
TW="[-0.44,-0.38,-0.205,-0.125,-6.0,6.0]"; R1="[-0.40,-0.34,-0.365,-0.285,-6.0,6.0]"
TAGNAME=s13tw_h0 PARAMS=script_params_twist_h0_sim.json TWIST=1 START_RANGE="$TW" ./s0_script_1113.sh
TAGNAME=s13tw_h35 PARAMS=script_params_twist_h35_sim.json TWIST=1 START_RANGE="$TW" ./s0_script_1113.sh
TAGNAME=s13r1_h0 PARAMS=script_params_run1_h0_sim.json START_RANGE="$R1" ./s0_script_1113.sh
TAGNAME=s13r1_h35 PARAMS=script_params_run1_h35_sim.json START_RANGE="$R1" ./s0_script_1113.sh
echo "=== [$(date "+%m-%d %T")] 4 本とも終わり"
