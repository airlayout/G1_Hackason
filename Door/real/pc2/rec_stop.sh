#!/usr/bin/env bash
# rec_take.sh で録っている 1 本を早めに止める（SIGINT。record_dds_to_bag.py は捕まえて db3 と metadata を書き切る）。
#
#   ./rec_stop.sh <名前>
#
# 人が「動かし終わった」と言った所で止める。録る長さを先に当てなくて済む（10-21 の 02c は、録り始める前に動かし終わっていた）
# ⚠️ pgrep/pkill -f は自分のコマンド行にも当たるので、パターンの先頭を [r] のように括る
set -u
NAME="${1:?名前を渡す}"
pkill -INT -f "[r]ecord_dds_to_bag.py .*--name ${NAME}_(li|odom)" || echo "[rec] 記録のプロセスが見つからない: $NAME"
pkill -INT -f "[w]atch_remote.py" || true
pkill -INT -f "[r]ec_ir.py .*_${NAME}_ir" || true
for _ in $(seq 1 20); do
  pgrep -f "[r]ec_take.sh ${NAME} " > /dev/null || break
  sleep 0.5
done
cat ~/mapping_runs/logs_1021/"${NAME}"_take.log 2>/dev/null | tail -8
