#!/usr/bin/env bash
# 段階を踏む比べ（10-15 夜）の後: 段階なしの 1 回目（反復 44,150、76.6%）が最良なので、別の扉の組（seed=1）と b2 の扉で確かめる
cd "$HOME/work_inoue/doorman" || exit 1
until grep -aq "exp_s25rl_done" exp_s25base_1015.log; do sleep 20; done
sleep 5
CK=$HOME/work_inoue/doorman/ckpt/start_1015_2137/last.pt
EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
export COUNT_ARGS="--criterion through" IDLE_GATE=0
echo "=== [$(date '+%m-%d %T')] best44150 seed=1"
EXTRA_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight $EVAL_EXTRA ++seed=1" ./eval_from_start.sh "$CK" 256 > best44150_seed1.log 2>&1
grep -a "eval_from_start out=" best44150_seed1.log | sed "s/^/  [best seed=1] /"
echo "=== [$(date '+%m-%d %T')] best44150 b2"
EXTRA_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_lever $EVAL_EXTRA" ./eval_from_start.sh "$CK" 256 > best44150_b2.log 2>&1
grep -a "eval_from_start out=" best44150_b2.log | sed "s/^/  [best b2] /"
echo "=== [$(date '+%m-%d %T')] after_base_done"
