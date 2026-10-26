#!/usr/bin/env bash
# V2 run2（10-16）: run1 の推定で閉ループを回して画像を集め（学習の俯角 44°。試験の 41° は混ぜない）、
# それを足し、扉が大きく開いた画像を 3 倍・扉の角度の誤差を 2 倍に数えて学習し直し、41° の閉ループで評価する。
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
until grep -aq "exp_v2_eval_1016 done" v2_queue_1016c.log; do sleep 10; done   # 切り分けの評価が終わってから
echo "=== [$(date +%T)] run2: collect closed-loop images (run1, pitch 44)"
EST=v2model/run1/estimator.pt PITCH=44 MINUTES=20 COND="v2" TAG=v2dagger_1016 SEED=7 ./exp_v2_eval_1016.sh
echo "=== [$(date +%T)] run2: train"
OUT=v2model/run2 EPOCHS=8 EXTRA="--extra records/v2dagger_1016/v2/vision --open-oversample 3 --angle-weight 2.0" ./exp_v2_train_1016.sh
echo "=== [$(date +%T)] run2: closed loop (pitch 41)"
EST=v2model/run2/estimator.pt PITCH=41 MINUTES=14 COND="v2" TAG=v2eval_1016_run2 ./exp_v2_eval_1016.sh
echo "=== [$(date +%T)] exp_v2_run2_1016 done"
