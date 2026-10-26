#!/usr/bin/env bash
# 段 5 の試しを、接続から切り離して起こす（tmux が無いため）。画面の出力は logs/console_<日時>.log
# 機体はリモコンの start まで脱力（WAIT）で待つ。止めるのは リモコンの B（立ったまま）／L2+B（脱力）／./door_stop.sh（立ったまま）
#   ./door_start.sh               扉の試し（run_first.sh）
#   ./door_start.sh run_stand.sh  立つだけの試し
cd ~/door_real || exit 1
RUN="${1:-run_first.sh}"
[ -x "./$RUN" ] || { echo "[door] 起こすスクリプトが無い: $RUN"; exit 1; }
if [ -f logs/door.pid ] && kill -0 "$(cat logs/door.pid)" 2>/dev/null; then echo "[door] もう動いている（pid $(cat logs/door.pid)）"; exit 1; fi
ts=$(date +%Y%m%d_%H%M%S)
setsid nohup "./$RUN" > "logs/console_$ts.log" 2>&1 < /dev/null &
echo $! > logs/door.pid
ln -sf "console_$ts.log" logs/console_latest.log
sleep 3
echo "[door] 起こした（pid $(cat logs/door.pid)）。出力:"; cat "logs/console_$ts.log"
