#!/usr/bin/env bash
# 11-14（段 D・E）: 4 つの組（7 cm・10 cm・10 cm 足踏み・半身）を、方法（METHODS の並び）ごとに続けて回す。
#   止めるのは 2a から（HOMIE の 2a の 5 cm の流れを消す）・置く所は実機の構え。止め札 STOP_REPLAY_1114 で次の組へ進まない
#   b（11-14）: 置く所を実機の 2a の終わりの立ち位置にした（実機は右へ約 5° 回って立っていた。シムの ±6° 一様では 2b の届きが変わる）
# 使い方: METHODS="vel door" TAG=r3 setsid nohup ./s0_replay_chain_1114b.sh > s0/replay_chain_r3.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
TAG="${TAG:-r1}"
for M in ${METHODS:-vel door}; do
  for G in "7:script_params_rep7_sim.json:1,3,7:24:0:[-0.42,-0.37,-0.346,-0.306,-6.0,-4.0]" \
           "10:script_params_rep10_sim.json:8,9,11,12,13,14:36:0:[-0.42,-0.37,-0.346,-0.306,-6.0,-4.0]" \
           "10w:script_params_rep10w_sim.json:15,16:24:0:[-0.42,-0.37,-0.346,-0.306,-6.0,-4.0]" \
           "t:script_params_rept_sim.json:19,21:24:1:[-0.49,-0.44,-0.186,-0.136,-6.0,-2.0]"; do
    [ -e "$HOME/work_inoue/STOP_REPLAY_1114" ] && { echo "=== 止め札があるので止める"; exit 0; }
    IFS=: read -r name params labels n twist range <<< "$G"
    N="$n" TAGNAME="${TAG}_${M}_g${name}" PARAMS="$params" LABELS="$labels" METHOD="$M" TWIST="$twist" START_RANGE="$range" \
      HOLD_FROM=2a MINUTES=20 ./s0_replay_1114.sh
  done
done
echo "=== [$(date "+%m-%d %T")] chain ${TAG} 終わり"
