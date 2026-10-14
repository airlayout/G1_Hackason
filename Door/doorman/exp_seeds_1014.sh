#!/usr/bin/env bash
# 10-14 16:30: 80.5% がたまたまかを見る。反復 40,600（B の最良）と 41,100（A の最良）を、立った姿勢から乱数の種 1・2 で 256 回ずつ
set -uo pipefail
cd $HOME/work_inoue/doorman || exit 1
export IDLE_GATE=0 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE=$HOME/work_inoue/PAUSE_DOOR_STAGE2B COUNT_ARGS="--criterion through"
for SPEC in "B_s1:ckpt/s24LB_r3_it40600/last.pt:1" "A_s1:ckpt/s24LA_r4_it41100/last.pt:1" "B_s2:ckpt/s24LB_r3_it40600/last.pt:2" "A_s2:ckpt/s24LA_r4_it41100/last.pt:2"; do
  IFS=: read NAME CK SEED <<< "$SPEC"
  echo "=== [$(date "+%m-%d %T")] $NAME ck=$CK seed=$SEED"
  EXTRA_ARGS="++env.config.task.name=door_en1154_g13_shaped_left ++env.config.reset_from_dataset.enabled=False ++seed=$SEED" ./eval_from_start.sh "$CK" 256 > eval_seed_${NAME}.log 2>&1
  grep -a "eval_from_start out=" eval_seed_${NAME}.log | sed "s/^/  [$NAME] /"
  sleep 61
done
echo "=== [$(date "+%m-%d %T")] exp_seeds_done"
