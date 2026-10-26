#!/usr/bin/env bash
# 10-20: 押して開ける間（区間 4 以降）に、頭のカメラが扉を見ていないことが成功率をどれだけ下げているかを、評価だけで測る。
# 条件は exp_v2_eval_1016.sh の e1・e2a・e2b・e3（パッチ 0060）。比べる元は 10-17 の v2（全部画像、種 0・1）と truth（本当の値）。
# 推定は 2 回目（run2）・俯角 41°（学習に無い）・16 台・1 条件 14 分。種は 10-17 と同じ 0 と 1。
#
# 使い方（OMEN の ~/work_inoue/doorman で）: SEEDS="0 1" CONDS="e1 e2a e2b e3" ./exp_push_1020.sh
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
for S in ${SEEDS:-0 1}; do
  echo "=== [$(date +%T)] seed=$S conds=${CONDS:-e1 e2a e2b e3}"
  EST=v2model/run2/estimator.pt PITCH=41 MINUTES="${MINUTES:-14}" SEED="$S" COND="${CONDS:-e1 e2a e2b e3}" \
    TAG="push_1020_seed$S" ./exp_v2_eval_1016.sh
done
echo "=== [$(date +%T)] exp_push_1020 done"
