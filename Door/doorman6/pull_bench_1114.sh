#!/usr/bin/env bash
# 11-14: OMEN の再生のシムの結果（JSON Lines・毎周期の記録・static.npz）を Mac の置き場へ写す。
#   ./pull_bench_1114.sh <置き場> <JSON Lines の名前（s0/ の下・.jsonl まで）> [records/start_… の名前（無ければ eval.log から探す）]
#   標準出力の最後の行に、写した JSON Lines の道を出す
set -euo pipefail
DST="$1"; J="$2"; R="${3:-}"
if [ -z "$R" ]; then
  R=$(ssh -o BatchMode=yes ubuntu@100.99.102.70 "grep -l '$J' ~/work_inoue/doorman/records/start_1114_*/eval.log | tail -1 | xargs dirname | xargs basename")
fi
D="$DST/$(basename "$J" .jsonl)"
mkdir -p "$D"
scp -q "ubuntu@100.99.102.70:~/work_inoue/doorman/s0/$J*" "$D/"
scp -q "ubuntu@100.99.102.70:~/work_inoue/doorman/records/$R/static.npz" "$D/"
echo "$D/$J"
