#!/usr/bin/env bash
# camera_stream.py の起動・停止・状態確認。**ロボット本体（Jetson）で動かす。**
#   ./camera_ctl.sh start | stop | status
#
# ⚠️ 常駐化・自動再起動は禁止: systemd / cron / rc.local / while ループでの再起動を作らない。
#    起動は start を叩いたときだけ、止めるのは stop だけ。落ちたらそのまま落ちたままにする。
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
PIDFILE="$DIR/camera_stream.pid"
LOG="$DIR/camera_stream.log"
BYID=/dev/v4l/by-id
STOP_WAIT_S=5
PYTHON="${CAMERA_PYTHON:-$HOME/miniforge3/envs/lerobot/bin/python}"  # 実績のある cv2(V4L2) 入りの環境

# pidfile の PID が「本当に camera_stream.py か」を確かめる（PID 再利用で他を殺さない）
running_pid() {
  [ -f "$PIDFILE" ] || return 1
  local pid; pid="$(cat "$PIDFILE")"
  [ -n "$pid" ] && tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q camera_stream.py || return 1
  echo "$pid"
}

find_dev() { ls "$BYID"/$1 2>/dev/null | head -n 1; }

cmd_start() {
  local pid
  if pid="$(running_pid)"; then echo "[camera] すでに起動中 (pid=$pid)"; return 0; fi
  rm -f "$PIDFILE"
  local args=() std d435
  std="$(find_dev '*Sunplus*webcam*index0')"
  d435="$(find_dev '*RealSense*435i*index0')"
  [ -n "$std" ] && args+=(--camera "std=$std") || echo "[camera] 標準カメラが見つかりません（配信しません）"
  [ -n "$d435" ] && args+=(--camera "d435i=$d435") || echo "[camera] D435i が見つかりません（配信しません）"
  [ ${#args[@]} -gt 0 ] || { echo "[camera] 配信できるカメラがありません"; return 1; }
  # setsid: ssh を切っても生き残らせる（ただし再起動はしない）
  setsid nohup "$PYTHON" -u "$DIR/camera_stream.py" "${args[@]}" >"$LOG" 2>&1 < /dev/null &
  echo $! > "$PIDFILE"
  sleep 2
  if pid="$(running_pid)"; then echo "[camera] 起動しました (pid=$pid)"; else
    echo "[camera] 起動に失敗しました。ログ:"; tail -n 10 "$LOG"; rm -f "$PIDFILE"; return 1
  fi
}

cmd_stop() {
  local pid
  if ! pid="$(running_pid)"; then rm -f "$PIDFILE"; echo "[camera] 起動していません"; return 0; fi
  kill "$pid"
  for _ in $(seq "$STOP_WAIT_S"); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
  kill -0 "$pid" 2>/dev/null && { kill -9 "$pid"; sleep 1; }
  if kill -0 "$pid" 2>/dev/null; then echo "[camera] 停止できませんでした (pid=$pid)"; return 1; fi
  rm -f "$PIDFILE"; echo "[camera] 停止しました"
}

cmd_status() {
  local pid
  if pid="$(running_pid)"; then echo "[camera] 起動中 (pid=$pid)"; else echo "[camera] 停止中"; fi
}

case "${1:-}" in
  start) cmd_start ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  *) echo "使い方: $0 start|stop|status"; exit 2 ;;
esac
