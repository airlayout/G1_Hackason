#!/usr/bin/env bash
# 段 5 の試しを、接続から切り離して起こす（tmux が無いため）。画面の出力は logs/console_<日時>.log
# 機体はリモコンの start まで脱力（WAIT）で待つ。止めるのは リモコンの B（立ったまま）／L2+B（脱力）／./door_stop.sh（立ったまま）
#   ./door_start.sh               扉の試し（run_first.sh）
#   ./door_start.sh run_stand.sh  立つだけの試し
# 11-03: run_stock.sh のときは、動いている間ずっと生の点群を録る（rec_follow.sh。ライダーの追跡の崩れを記録の上で再現するため）。
#   REC=0 で録らない・REC=1 で run_stock.sh 以外でも録る。REC_MAX（既定 900 秒）は記録の上限。出力は logs/rec_<日時>.log
cd ~/door_real || exit 1
RUN="${1:-run_first.sh}"
[ -x "./$RUN" ] || { echo "[door] 起こすスクリプトが無い: $RUN"; exit 1; }
if [ -f logs/door.pid ] && kill -0 "$(cat logs/door.pid)" 2>/dev/null; then echo "[door] もう動いている（pid $(cat logs/door.pid)）"; exit 1; fi
REC="${REC:-auto}"
if [ "${REC}" = auto ]; then
  if [ "${RUN}" = run_stock.sh ] || [ "${RUN}" = run_vision.sh ]; then REC=1; else REC=0; fi  # 11-05: 画像の生徒も答え合わせに録る
fi
if [ "${REC}" = 1 ]; then
  ./rec_follow.sh --check || { echo "[door] 起こさない（生の点群を録れない。REC=0 なら録らずに起こす）"; exit 1; }
fi
ts=$(date +%Y%m%d_%H%M%S)
setsid nohup "./$RUN" > "logs/console_$ts.log" 2>&1 < /dev/null &
echo $! > logs/door.pid
ln -sf "console_$ts.log" logs/console_latest.log
if [ "${REC}" = 1 ]; then
  setsid nohup ./rec_follow.sh "${RUN%.sh}_${ts}" "$(cat logs/door.pid)" "${REC_MAX:-900}" > "logs/rec_$ts.log" 2>&1 < /dev/null &
  ln -sf "rec_$ts.log" logs/rec_latest.log
fi
sleep 3
echo "[door] 起こした（pid $(cat logs/door.pid)）。出力:"; cat "logs/console_$ts.log"
if [ "${REC}" = 1 ]; then
  echo "[door] 生の点群も録っている（logs/rec_$ts.log）:"; cat "logs/rec_$ts.log"
fi
