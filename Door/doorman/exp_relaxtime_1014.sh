#!/usr/bin/env bash
# 10-14 17:00: 評価だけ時間を 2 倍に緩める（ユーザー「時間制限が厳しい・シビアな時間を求められるものではない」）。
# 区間の持ち時間 [250,100,100,100,300,200] → [500,200,200,200,600,400]（通しの締め切り 17・21 s → 34・42 s）、1 回の上限 30 → 60 s。
# 反復 40,600（B の最良）・把手が左＋鏡写し・立った姿勢から・256 回。比べる相手は同じ条件で時間がそのままの 206/256（80.5%）
set -uo pipefail
cd $HOME/work_inoue/doorman || exit 1
export IDLE_GATE=0 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE=$HOME/work_inoue/PAUSE_DOOR_STAGE2B COUNT_ARGS="--criterion through" MINUTES=30
EXTRA_ARGS="++env.config.task.name=door_en1154_g13_shaped_left ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"   ./eval_from_start.sh ckpt/s24LB_r3_it40600/last.pt 256 > eval_relaxtime_B.log 2>&1
grep -a "eval_from_start out=\|区間 s 以上\|end eval_record" eval_relaxtime_B.log
echo "=== [$(date "+%m-%d %T")] exp_relaxtime_done"
