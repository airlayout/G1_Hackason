#!/usr/bin/env bash
# 見張り用: run_bench.sh の標準出力のログと、最新の計測ディレクトリの状態を 1 行ずつ出す。
# 同じ行は毎回同じ文字列になるので、見張り側で前回との差分だけを通知にすればよい。
# 使い方: EVERY=500 ./run_status.sh <run_bench の標準出力のログ> <bench/ の接頭辞（例 t29_n256）>
BASE="$HOME/work_inoue/doorman"
cd "$BASE" || exit 1
RUNLOG="$1"; PREFIX="$2"; EVERY="${EVERY:-500}"
grep -aE "^=== " "$RUNLOG" 2>/dev/null
D=$(ls -d bench/${PREFIX}_* 2>/dev/null | tail -1)
[ -z "$D" ] && exit 0
grep -aE "^[A-Za-z]*Error" "$D/train.log" 2>/dev/null | tail -1 | sed "s|^|ERROR $D: |" | cut -c1-220
# EVERY 反復ごとの節目の行（反復が進むと 1 行ずつ増える）
python3 train_progress.py "$D/train.log" --every "$EVERY" 2>/dev/null | grep "^it=" | sed "s|^|PROGRESS |"
python3 train_progress.py "$D/train.log" --every 1000000000 2>/dev/null | grep -q "nan_rows=[1-9]" && echo "NAN_DETECTED $D"
exit 0
