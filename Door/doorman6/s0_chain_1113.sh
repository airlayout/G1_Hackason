#!/usr/bin/env bash
# 11-13: Room 4 の寸法の扉（door_room4_like_left_realwall）で、台本の 2 本を順に回す（並べると RAM が足りない）
#   A: 1 回目の再現（script_params_run1_sim.json）  B: 半身（script_params_twist_sim.json・腰 yaw のひねり）
cd "$HOME/work_inoue/doorman" || exit 1
export TASK=door_room4_like_left_realwall N=32 MORE="++env.config.script_teacher.tau_src=applied"
TAGNAME=s13r4a PARAMS=script_params_run1_sim.json START_RANGE="[-0.40,-0.34,-0.365,-0.285,-6.0,6.0]" ./s0_script_1113.sh
TAGNAME=s13r4b PARAMS=script_params_twist_sim.json TWIST=1 START_RANGE="[-0.44,-0.38,-0.205,-0.125,-6.0,6.0]" ./s0_script_1113.sh
echo "=== [$(date "+%m-%d %T")] 2 本とも終わり"
