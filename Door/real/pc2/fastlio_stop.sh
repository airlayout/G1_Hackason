#!/usr/bin/env bash
# 11-10 手順 L2: fastlio_start.sh で起こした口（SIGTERM。止める時に送った電文を npz に書く）と FAST-LIO（SIGINT。
# SIGTERM だと子が孤児になる＝Mapping の run_fastlio_loc_live.sh の注記）を止め、/tmp/pc2_lock を返す。
# 走らせ役が段 1〜2 のときに止めると、0.5 s 後に走らせ役が「届かない」で立ったまま止まる。ふだんは door_stop.sh の後に
cd ~/door_real || exit 1
stop_pid() {  # pid のファイル・送る合図・名前・cmdline に含まれる語（pid の使い回しで別のプロセスへ合図を送らない）
  local pid
  pid=$(cat "$1" 2>/dev/null)
  if [ -z "$pid" ] || ! tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q "$4"; then
    echo "[fastlio] $3 は動いていない"; rm -f "$1"; return 0
  fi
  kill "-$2" "$pid"
  for _ in $(seq 25); do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
  if kill -0 "$pid" 2>/dev/null; then
    echo "[fastlio] ⚠️ $3 が 5 秒で止まらない（pid ${pid}）→ KILL"; kill -KILL "$pid"
  else
    echo "[fastlio] $3 を止めた"
  fi
  rm -f "$1"
}
stop_pid logs/fastlio_node.pid TERM 口 door_fastlio_node
tail -2 logs/fastlio_node_console_latest.log 2>/dev/null
stop_pid logs/fastlio.pid INT FAST-LIO fastlio_mapping
if grep -q "^door_fastlio " /tmp/pc2_lock/owner 2>/dev/null; then
  rm -rf /tmp/pc2_lock && echo "[fastlio] /tmp/pc2_lock を返した"
elif [ -d /tmp/pc2_lock ]; then
  echo "[fastlio] /tmp/pc2_lock は他の持ち主（$(cat /tmp/pc2_lock/owner 2>/dev/null || echo 不明)）なので返さない"
fi
