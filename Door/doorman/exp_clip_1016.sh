#!/usr/bin/env bash
# 段 5（10-16）: 関節の目標を可動域の内側に切る（実機で端の止めに全力で押し付けないため、パッチ 0051）と成功率がどう変わるか。
# 最良の生徒 45,150 を、段 2.5 の評価と同じ条件（256 台・立った姿勢・時間 2 倍・学習の扉）で、切らない／端ちょうど／端から 0.05 rad の 3 通り。
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK=ckpt/s25rl_best_it45150_ovr6/last.pt
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
for M in none 0.0 0.05; do
  X="$BASE_ARGS"
  [ "$M" != none ] && X="$X ++env.config.clip_targets_to_limits_margin=$M"
  echo "=== [$(date +%T)] clip margin=$M"
  IDLE_GATE="${IDLE_GATE:-1}" COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="$X" ./eval_from_start.sh "$CK" 256 2>&1 | grep -E "eval_from_start out|成功|Traceback|Error" | sed "s/^/[margin=$M] /"
done
echo "=== [$(date +%T)] exp_clip_1016 done"
