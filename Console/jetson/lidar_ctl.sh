#!/usr/bin/env bash
# lidar_stream.py の起動・停止・状態確認。**ロボット本体（Jetson）で動かす。**
#   [LIDAR_PORT=8082] ./lidar_ctl.sh start | stop | status
#
# ⚠️ 常駐化・自動再起動は禁止: systemd / cron / rc.local / while ループでの再起動を作らない。
#    起動は start を叩いたときだけ、止めるのは stop だけ。落ちたらそのまま落ちたままにする。
# lidar_stream.py は DDS を購読するだけ（モーター・lidar 本体の設定には触れない）。
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
PIDFILE="$DIR/lidar_stream.pid"
LOG="$DIR/lidar_stream.log"
STOP_WAIT_S=5
START_WAIT_S=3
PORT="${LIDAR_PORT:-8082}"
NIC="${LIDAR_NIC:-eth0}"
PYTHON="${LIDAR_PYTHON:-python3}"  # システムの Python 3.8（cyclonedds / numpy が揃う）

# pidfile の PID が「本当に lidar_stream.py か」を確かめる（PID 再利用で他を殺さない）
running_pid() {
  [ -f "$PIDFILE" ] || return 1
  local pid; pid="$(cat "$PIDFILE")"
  [ -n "$pid" ] && tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q lidar_stream.py || return 1
  echo "$pid"
}

cmd_start() {
  local pid
  if pid="$(running_pid)"; then echo "[lidar] すでに起動中 (pid=$pid)"; return 0; fi
  rm -f "$PIDFILE"
  # setsid: ssh を切っても生き残らせる（ただし再起動はしない）
  setsid nohup "$PYTHON" -u "$DIR/lidar_stream.py" --port "$PORT" --nic "$NIC" >"$LOG" 2>&1 < /dev/null &
  echo $! > "$PIDFILE"
  sleep "$START_WAIT_S"
  if pid="$(running_pid)"; then echo "[lidar] 起動しました (pid=$pid, port=$PORT)"; else
    echo "[lidar] 起動に失敗しました。ログ:"; tail -n 10 "$LOG"; rm -f "$PIDFILE"; return 1
  fi
}

cmd_stop() {
  local pid
  if ! pid="$(running_pid)"; then rm -f "$PIDFILE"; echo "[lidar] 起動していません"; return 0; fi
  kill "$pid"
  for _ in $(seq "$STOP_WAIT_S"); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
  kill -0 "$pid" 2>/dev/null && { kill -9 "$pid"; sleep 1; }
  if kill -0 "$pid" 2>/dev/null; then echo "[lidar] 停止できませんでした (pid=$pid)"; return 1; fi
  rm -f "$PIDFILE"; echo "[lidar] 停止しました"
}

cmd_status() {
  local pid
  if pid="$(running_pid)"; then echo "[lidar] 起動中 (pid=$pid)"; else echo "[lidar] 停止中"; fi
}

case "${1:-}" in
  start) cmd_start ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  *) echo "usage: $0 start|stop|status" >&2; exit 2 ;;
esac
