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
VIDEO_NIC="${CAMERA_NIC:-eth0}"  # VideoClient が使う NIC
PYTHON="${CAMERA_PYTHON:-python3}"  # システムの Python 3.8（cv2 / pyrealsense2 / unitree_sdk2py が揃う）
SDK_DIR="${UNITREE_SDK_DIR:-$HOME/unitree_sdk2_python}"

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
  local args=() std
  std="$(find_dev '*Sunplus*webcam*index0')"
  [ -n "$std" ] && args+=(--camera "std=$std") || echo "[camera] 標準カメラが見つかりません（配信しません）"
  # D435i の RGB は videohub_pc4 が /dev/video4 を専有するため、公式経路 VideoClient で取る（videohub は止めない）
  args+=(--camera "d435i=videoclient:$VIDEO_NIC")
  # 深度は USB インターフェース 1.0（videohub と別）。pyrealsense2 で読む
  args+=(--camera "depth=depth:realsense")
  [ ${#args[@]} -gt 0 ] || { echo "[camera] 配信できるカメラがありません"; return 1; }
  # setsid: ssh を切っても生き残らせる（ただし再起動はしない）
  PYTHONPATH="$SDK_DIR" setsid nohup "$PYTHON" -u "$DIR/camera_stream.py" "${args[@]}" >"$LOG" 2>&1 < /dev/null &
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

# 映像デバイスを掴んでいるプロセスを「デバイス<TAB>PID<TAB>ユーザー<TAB>コマンド」で列挙する。
# root のプロセスは sudo 無しでは fuser で見えないため、ps の引数に /dev/videoN が出るものも拾う（videohub_pc4 など）。
cmd_holders() {
  local dev
  for dev in /dev/video*; do
    [ -e "$dev" ] || continue
    ps -eo pid=,user=,args= | awk -v d="$dev" '{for (i = 3; i <= NF; i++) if ($i == d) { print; break }}' | \
      while read -r pid user args; do printf '%s\t%s\t%s\t%s\n' "$dev" "$pid" "$user" "${args:0:160}"; done
    fuser "$dev" 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+$' | while read -r pid; do
      printf '%s\t%s\t%s\t%s\n' "$dev" "$pid" "$(ps -o user= -p "$pid")" "$(ps -o args= -p "$pid" | cut -c1-160)"
    done
  done | sort -u
}

case "${1:-}" in
  holders) cmd_holders ;;
  start) cmd_start ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  *) echo "使い方: $0 start|stop|status|holders"; exit 2 ;;
esac
