#!/usr/bin/env bash
# 2 台のスマホを並べたステレオ（DROID-SLAM）を、較正から起動まで回す
#
#   bash start_stereo.sh calib --baseline-mm 120 [--pairs 30] 較正。2 台を固定して部屋の景色を写す
#                                                            （止める → 向きを変える、を繰り返す。市松模様は使わない）
#   bash start_stereo.sh solve <calib/日時_scene> --baseline-mm 120   集めた組から解き直す
#   bash start_stereo.sh sync [--secs 20]                    時刻のずれを測る（2 台を一緒に、止める・振るを繰り返す）
#   bash start_stereo.sh start [run_stereo_slam.py の引数]   SLAM と表示（9090）。最新の較正とずれを使う
#   bash start_stereo.sh stop                                止める（全体の最適化 → results/<日時>_stereo/ に保存）
#   bash start_stereo.sh log                                 記録の最後を見る
#
# 左のカメラ（iPhone）は cam1、右（Pixel）は cam2 で送る。左右を逆に置いても較正が気づいて入れ替える。
# 表示はどれも固定の口 http://<IP>:9090/?url=ws://<IP>:9877（start_mediamtx.sh が表示のサーバーを動かす）。
# 単眼の SLAM（start_slam.sh）とは GPU を分け合うので、同時には回さない。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${WORK:-$HOME/work_inoue/phone_slam}"
PY="${PY:-$HOME/miniforge3/envs/droid/bin/python}"
MEM_MAX="${MEM_MAX:-8G}"
TS_IP="${TS_IP:-100.99.102.70}"
LOG_DIR="$WORK/logs"
PIDFILE="$LOG_DIR/stereo.pid"
LATEST="$LOG_DIR/stereo_latest.log"
IDLE_MIN_S="${IDLE_MIN_S:-600}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

log() { printf '[stereo] %s\n' "$*"; }
lan_ip() { ip -4 -o addr show | awk '{print $4}' | cut -d/ -f1 | grep -m1 '^192\.168\.0\.' || true; }
hosts() { echo "$(lan_ip),$TS_IP"; }

desk_in_use() {
  local sid
  sid="$(loginctl list-sessions --no-legend | awk '$4=="seat0"{print $1}' | while read -r s; do
    [[ "$(loginctl show-session "$s" -p Class --value)" == "user" ]] && echo "$s"; done | head -1)"
  [[ -z "$sid" ]] && return 1
  local idle_ms
  idle_ms="$(DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus" gdbus call --session \
    --dest org.gnome.Mutter.IdleMonitor --object-path /org/gnome/Mutter/IdleMonitor/Core \
    --method org.gnome.Mutter.IdleMonitor.GetIdletime 2>/dev/null | sed -E 's/^\(uint64 ([0-9]+),\)$/\1/')"
  [[ "$idle_ms" =~ ^[0-9]+$ ]] || return 0
  log "机のセッション $sid・無操作 $((idle_ms / 1000)) 秒"
  (( idle_ms / 1000 < IDLE_MIN_S ))
}

running_pid() {
  [[ -s "$PIDFILE" ]] || return 0
  local pid; pid="$(cat "$PIDFILE")"
  if kill -0 "$pid" 2>/dev/null; then echo "$pid"; fi
}

cmd_calib() {
  [[ " $* " == *" --baseline-mm "* ]] || log "--baseline-mm が無いので集めるだけ。レンズの間の距離を測ったら solve で解く"
  mkdir -p "$LOG_DIR"
  pgrep -u "$(id -u)" -f "[s]cene_calib.py" >/dev/null && { log "較正はもう動いている"; exit 1; }
  bash "$HERE/start_mediamtx.sh" >/dev/null   # 受け口・送信ページ・表示のサーバー
  local logf="$LOG_DIR/stereo_calib_$(date +%m%d_%H%M%S).log"
  setsid nohup env PYTHONUNBUFFERED=1 OPENCV_LOG_LEVEL=ERROR "$PY" "$HERE/scene_calib.py" capture --hosts "$(hosts)" "$@" \
    >"$logf" 2>&1 </dev/null &
  ln -sfn "$logf" "$LOG_DIR/stereo_calib_latest.log"
  sleep 3
  grep "^\[較正\]" "$logf" | head -3 || true
  log "記録: $logf（tail -f で追う）"
  echo "  様子（固定の口）: http://$TS_IP:9090/?url=ws://$TS_IP:9877"
}

cmd_solve() {
  PYTHONUNBUFFERED=1 OPENCV_LOG_LEVEL=ERROR "$PY" "$HERE/scene_calib.py" solve "$@"
}

cmd_sync() {
  log "2 台を景色に向けたまま、一緒に「止める・振る・止める…」を繰り返す（点滅で測るなら --mode flash）"
  PYTHONUNBUFFERED=1 "$PY" "$HERE/measure_sync.py" "$@"
}

cmd_start() {
  local pid; pid="$(running_pid)"
  [[ -z "$pid" ]] || { log "もう動いている（pid $pid）。先に stop"; exit 1; }
  if [[ -s "$LOG_DIR/slam1.pid" ]] && kill -0 "$(cat "$LOG_DIR/slam1.pid")" 2>/dev/null; then
    log "単眼の SLAM が動いている（GPU を分け合う）。先に bash start_slam.sh stop"; exit 1
  fi
  if [[ "${FORCE:-0}" != 1 ]] && desk_in_use; then
    log "机で OMEN が使われている。GPU を使わないので起こさない（本人が机にいるなら FORCE=1）"; exit 1
  fi
  if [[ " $* " != *" --calib "* ]]; then
    ls "$WORK"/calib/*/calib.json >/dev/null 2>&1 || { log "較正が無い。先に calib"; exit 1; }
  fi
  bash "$HERE/start_mediamtx.sh" >/dev/null
  mkdir -p "$LOG_DIR"
  local stamp; stamp="$(date +%m%d_%H%M%S)"
  local logf="$LOG_DIR/stereo_$stamp.log"
  # 待ち役の bash を ssh のセッションに残す（start_slam.sh と同じ形）
  setsid nohup bash -c '
    systemd-run --user --scope --quiet -p MemoryMax="$1" -p MemorySwapMax=0 \
      env PYTHONUNBUFFERED=1 OPENCV_LOG_LEVEL=ERROR \
      "$2" "$3/run_stereo_slam.py" --hosts "$4" --save-dir "$5" "${@:6}" &
    echo $! > "$0"
    wait $!
  ' "$PIDFILE" "$MEM_MAX" "$PY" "$HERE" "$(hosts)" "$WORK/results/${stamp}_stereo" "$@" >"$logf" 2>&1 </dev/null &
  ln -sfn "$logf" "$LATEST"
  sleep 3
  log "起こした（記録 $logf）。DROID の重みを読んで 2 台の映像を待つ"
  echo "  Mac（Tailscale）: http://$TS_IP:9090/?url=ws://$TS_IP:9877"
}

cmd_stop() {
  local pid; pid="$(running_pid)"
  [[ -n "$pid" ]] || { log "動いていない"; return; }
  kill -INT "$pid"
  for _ in $(seq 120); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
  if kill -0 "$pid" 2>/dev/null; then log "120 秒で終わらない。SIGTERM"; kill -TERM "$pid"; fi
  rm -f "$PIDFILE"
  log "止めた"; grep -E "保存した|終わり" "$LATEST" | tail -2 || true
}

case "${1:-}" in
  calib) shift; cmd_calib "$@" ;;
  solve) shift; cmd_solve "$@" ;;
  sync) shift; cmd_sync "$@" ;;
  start) shift; cmd_start "$@" ;;
  stop) cmd_stop ;;
  log) tail -n 40 "$LATEST" ;;
  *) sed -n '2,12p' "$0"; exit 2 ;;
esac
