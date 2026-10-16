#!/usr/bin/env bash
# 段階を踏む比べ（10-15 夜）: s25cur（段階あり）が終わったら、同じ出発・同じ回数で s25base（段階なし）を回す
cd "$HOME/work_inoue/doorman" || exit 1
until grep -aq "exp_s25rl_done" exp_s25cur_1015.log; do sleep 20; done
sleep 5
CK=$HOME/work_inoue/doorman/ckpt/s25rl_pass_it43650/last.pt ROUNDS=2 PASS_RATE=1.1 NAME=s25base IDLE_GATE=0 ./exp_s25rl_1015.sh > exp_s25base_1015.log 2>&1
