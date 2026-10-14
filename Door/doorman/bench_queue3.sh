#!/usr/bin/env bash
# 段 2-1 の起動確認のやり直し（10/6 19:12 はラッチ無しのドアに .*latch.* の初期角が残っていて落ちた）。
# bench_queue2.sh（映像の生徒の計測）が終わるのを待ってから流す。
set -uo pipefail
BASE="$HOME/work_inoue/doorman"
cd "$BASE"
until grep -q QUEUE_DONE bench_queue2.log 2>/dev/null; do sleep 30; done
REPO=VIRAL29 EXP=wbmanip/door_open_homie_29dof_lstm KIND=t29 MEM_MAX=8G ./run_bench.sh 256 12
echo QUEUE3_DONE
