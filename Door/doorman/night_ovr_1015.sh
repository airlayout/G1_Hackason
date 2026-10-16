#!/usr/bin/env bash
# 10-15 夜の比べ: 最良の生徒（反復 44,150）から、押し切りの減点 −6.0（s25ovr）と −3.0 のまま（s25base2）を 30 分 × 2 回ずつ。
# 強化学習を続けると押し切りに流れる（12.10）のを、減点を強めて止められるか
cd "$HOME/work_inoue/doorman" || exit 1
CK=$HOME/work_inoue/doorman/ckpt/s25rl_best_it44150/last.pt
CK=$CK ROUNDS=2 PASS_RATE=1.1 NAME=s25ovr OVR=-6.0 IDLE_GATE=0 ./exp_s25rl_1015.sh > exp_s25ovr_1015.log 2>&1
CK=$CK ROUNDS=2 PASS_RATE=1.1 NAME=s25base2 IDLE_GATE=0 ./exp_s25rl_1015.sh > exp_s25base2_1015.log 2>&1
echo "=== [$(date '+%m-%d %T')] night_ovr_done"
