#!/usr/bin/env bash
# 2026-10-06 夜のキュー。同時に 2 本は動かさない（RAM 15 GiB の共用機）。どれも画面の前の人の離席を待ち、
# 人が戻ったら止めて譲る（run_train_segments.sh）。
#   1. 記録の確認: 段 2-1 の確認の学習（245 反復）のチェックポイントを評価しながら状態を記録（15 分まで）
#   2. 段 2-1 の学習: 29 関節版・ラッチ無し・512 台。朝 8:30 まで、譲ったら last.pt から続きを学習
set -uo pipefail
BASE="$HOME/work_inoue/doorman"
cd "$BASE"
export IDLE_GATE=1
UNTIL="${UNTIL:-08:30}"

CKPT="$BASE/VIRAL29/logs_rl/bench/wbmanip/door_open_homie_29dof_lstm_lstm-20261006_195326/last.pt"
MAX_WAIT_MIN=600 MEM_MAX=6G ./run_eval_record.sh "$CKPT" "$BASE/records/t29_it245" 4 8 15 \
  || echo "=== [$(date +%T)] 記録の確認は失敗（学習へ進む）"

# 20:54 に空きが 9.5 GB（画面の前の人のアプリ）。上乗せの余裕は 1.0 GB にする（走行中は空き 1.5 GB 未満で止まる）
REPO=VIRAL29 EXP=wbmanip/door_open_homie_29dof_lstm KIND=door29 PROJECT=door29 MEM_MAX=8G NEED_MARGIN_MB=1024 \
  ./run_train_segments.sh 512 "$UNTIL"
echo "NIGHT_QUEUE_DONE rc=$?"
