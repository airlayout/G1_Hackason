#!/usr/bin/env bash
# V2 の続き（10-17）: 10-16 に止めた所から。
#   1. 1 回目の推定（run1）で閉ループを回して画像を集める（学習の俯角 44°。試験の 41° は混ぜない）
#   2. それを足し、扉が大きく開いた画像を 3 倍・扉の角度の誤差を 2 倍に数えて学習し直す（run2）
#   3. run2 で閉ループ（41°）
#   4. 本当の値を同じ回し方（16 台・カメラあり・1 回の起動で何回も）で測る（比べる元）
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
echo "=== [$(date +%T)] 1. 閉ループの画像集め（run1・44°）"
EST=v2model/run1/estimator.pt PITCH=44 MINUTES=20 COND="v2" TAG=v2dagger_1016 SEED=7 ./exp_v2_eval_1016.sh
echo "=== [$(date +%T)] 2. 学習し直し（run2）"
OUT=v2model/run2 EPOCHS=8 EXTRA="--extra records/v2dagger_1016/v2/vision --open-oversample 3 --angle-weight 2.0" ./exp_v2_train_1016.sh
echo "=== [$(date +%T)] 3. run2 の閉ループ（41°）"
EST=v2model/run2/estimator.pt PITCH=41 MINUTES=14 COND="v2" TAG=v2eval_1017_run2 ./exp_v2_eval_1016.sh
echo "=== [$(date +%T)] 4. 本当の値（同じ回し方・41°）"
EST=v2model/run1/estimator.pt PITCH=41 MINUTES=14 COND="truth" TAG=v2eval_1016_run1 ./exp_v2_eval_1016.sh
echo "=== [$(date +%T)] exp_v2_resume_1017 done"
