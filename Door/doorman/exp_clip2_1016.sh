#!/usr/bin/env bash
# 段 5（10-16）: 関節の目標を可動域の端から 0.05 rad 内側で切る（パッチ 0051）を、別の扉の組（seed=1）とb2 の扉（door_en1154_g13_shaped_left_lever。重さは既定）でも確かめる。
# 切らないときの値は 10-15 の評価: seed=1 82.0%・b2 76.2%（45,150）
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK=ckpt/s25rl_best_it45150_ovr6/last.pt
BASE_ARGS="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05"
run() {
  echo "=== [$(date +%T)] $1"
  IDLE_GATE="${IDLE_GATE:-1}" COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="$BASE_ARGS $2" ./eval_from_start.sh "$CK" 256 2>&1 | grep -E "eval_from_start out|成功|Traceback" | sed "s/^/[$1] /"
}
run seed1 "++env.config.task.name=door_en1154_g13_shaped_left_weight ++seed=1"
run b2 "++env.config.task.name=door_en1154_g13_shaped_left_lever"  # exp_s25rl_1015.sh の「b2 の扉」
echo "=== [$(date +%T)] exp_clip2_1016 done"
