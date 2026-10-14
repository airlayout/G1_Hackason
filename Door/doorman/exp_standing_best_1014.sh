#!/usr/bin/env bash
# 10-14 16:10: 実物の向き（把手が左＋鏡写し）で最良だった重みを、立った姿勢から 256 回評価する
set -uo pipefail
cd $HOME/work_inoue/doorman || exit 1
export IDLE_GATE=0 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE=$HOME/work_inoue/PAUSE_DOOR_STAGE2B COUNT_ARGS="--criterion through"
for SPEC in "B_r3:ckpt/s24LB_r3_it40600/last.pt" "A_r4:ckpt/s24LA_r4_it41100/last.pt"; do
  NAME=${SPEC%%:*}; CK=${SPEC#*:}
  echo "=== [$(date "+%m-%d %T")] 立った姿勢から・$NAME ck=$CK"
  EXTRA_ARGS="++env.config.task.name=door_en1154_g13_shaped_left ++env.config.reset_from_dataset.enabled=False" ./eval_from_start.sh "$CK" 256 > eval_standing_${NAME}.log 2>&1
  grep -a "eval_from_start out=" eval_standing_${NAME}.log | sed "s/^/  [$NAME] /"
  sleep 61
done
echo "=== [$(date "+%m-%d %T")] exp_standing_best_done"
