#!/usr/bin/env bash
# 11-02: 切り離して起こした door_run_stock.py（run_stock.sh）へ PC の指示を送る。
#   ./stock_cmd.sh a      腕を構える（WAIT → ARMS_IN → READY。POST からなら構え直す）
#   ./stock_cmd.sh d      扉の方策（READY のとき。傾き・両足の体重・ライダーの準備がそろっていなければ断る。
#                         11-12 夜: 脚が揺れて体重を判定できない時は 30 秒まで待つ・--script は定数を読み直し、段 0 は 1 度通したら飛ばす）
#   ./stock_cmd.sh d0     --script: 段 0（巻き尺・網で測る）からやり直す
#   ./stock_cmd.sh stop   立ったまま止まる（POST。WAIT のときは終わる）
#   ./stock_cmd.sh p      押す試し（POST のとき。腕は止めた姿勢のまま前へ 0.3 m/s を 2 秒。p0.5 で 0.5 m/s）
#   ./stock_cmd.sh r      腕を戻して純正へ返す（READY・POST のとき）
#   ./stock_cmd.sh q      終わる（WAIT・DAMP のとき。ほかの段では立ったまま止まる）
# 脱力はここからは送らない（リモコンの L2+B）
F=~/door_real/logs/stock_cmd.fifo
[ -p "$F" ] || { echo "[stock] 指示の口が無い（run_stock.sh が動いていない）"; exit 1; }
case "${1:-stop}" in
  a|d|d0|r|q|p|p[0-9]*) c="$1" ;;
  stop) c="" ;;
  *) echo "使い方: $0 a|d|d0|p[速さ]|r|q|stop"; exit 2 ;;
esac
timeout 2 bash -c 'echo "$1" > "$2"' _ "$c" "$F" || { echo "[stock] 送れなかった（読み手がいない）"; exit 1; }
echo "[stock] 送った: ${1:-stop}（出力は logs/console_latest.log）"
