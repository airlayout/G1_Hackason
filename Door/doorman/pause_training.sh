#!/usr/bin/env bash
# 夜通し・昼間の学習（run_train_segments.sh）を一時停止・再開する札。
#   ./pause_training.sh on    札を置き、学習が止まるまで待つ（最大 3 分）。止まったら GPU とメモリを使ってよい
#   ./pause_training.sh off   札を外す。学習は空きを確かめてから last.pt の続きで再開する
set -uo pipefail
source "$(dirname "$0")/gate.sh"
case "${1:-}" in
  on)
    touch "$PAUSE_FILE"
    for _ in $(seq 90); do
      ps -eo args | grep -q "[t]rain_agent_trl.py" || { echo "paused（学習は止まっている）"; exit 0; }
      sleep 2
    done
    echo "まだ学習が動いている（止まるまで待つか、ログを確かめる）"; exit 1 ;;
  off) rm -f "$PAUSE_FILE"; echo "resumed（札を外した）" ;;
  *) echo "usage: $0 on|off"; exit 2 ;;
esac
