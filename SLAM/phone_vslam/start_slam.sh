#!/usr/bin/env bash
# OMEN で SLAM（MASt3R-SLAM）とウェブ表示を起こす・止める
#
#   bash start_slam.sh [源] [run_phone_slam.py への引数…]   起こす（既定の源は rtsp://127.0.0.1:8554/cam1）
#   bash start_slam.sh stop                                  止める（軌跡と点群を保存してから）
#   bash start_slam.sh log                                   記録の最後を見る
#   bash start_slam.sh reset                                 地図を捨てて、いまのこまから作り直す
#   bash start_slam.sh view [結果のフォルダ]                 保存した地図と軌跡を表示に送る（既定は最新）
#
# 表示はどれも固定の口 http://<IP>:9090/?url=ws://<IP>:9877（start_mediamtx.sh が表示のサーバーを動かす）
#
#   例: bash start_slam.sh rtsp://127.0.0.1:8554/cam1
#       bash start_slam.sh ~/work_inoue/phone_slam/rec/cam1_1117_1200.mkv --realtime
#       SLAM_ID=2 bash start_slam.sh rtsp://127.0.0.1:8554/cam2   2 台目（表示は同じ口に別の記録として並ぶ）
#       SLAM_ID=2 bash start_slam.sh stop
#
# OMEN は共用機。机で人が使っている間は GPU を使わない（FORCE=1 で越える。本人が机にいるときだけ）。
# 記憶は MEM_MAX で囲う（溢れたら SLAM だけが止まる。机の人の作業を落とさない）。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${WORK:-$HOME/work_inoue/phone_slam}"
PY="${PY:-$HOME/miniforge3/envs/mast3r/bin/python}"
MEM_MAX="${MEM_MAX:-8G}"
TS_IP="${TS_IP:-100.99.102.70}"
LOG_DIR="$WORK/logs"
SLAM_ID="${SLAM_ID:-1}"            # 2 台目を別に回すときは 2（口・記録・pid を分ける）
export RUNNER="${RUNNER:-run_phone_slam.py}"  # 2 台で 1 つの地図は run_multicam_slam.py（start_multicam.sh が入れる）
PIDFILE="$LOG_DIR/slam$SLAM_ID.pid"
LATEST="$LOG_DIR/slam${SLAM_ID}_latest.log"
RESET_FILE="$LOG_DIR/slam${SLAM_ID}.reset"
IDLE_MIN_S="${IDLE_MIN_S:-600}"   # 机の無操作がこの秒数より短ければ「使用中」
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

log() { printf '[slam] %s\n' "$*"; }

lan_ip() { ip -4 -o addr show | awk '{print $4}' | cut -d/ -f1 | grep -m1 '^192\.168\.0\.' || true; }

desk_in_use() {
  # seat0 に人のセッションがあり、無操作が IDLE_MIN_S 秒より短ければ使用中
  local sid
  sid="$(loginctl list-sessions --no-legend | awk '$4=="seat0"{print $1}' | while read -r s; do
    [[ "$(loginctl show-session "$s" -p Class --value)" == "user" ]] && echo "$s"; done | head -1)"
  [[ -z "$sid" ]] && return 1
  local idle_ms
  idle_ms="$(DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus" gdbus call --session \
    --dest org.gnome.Mutter.IdleMonitor --object-path /org/gnome/Mutter/IdleMonitor/Core \
    --method org.gnome.Mutter.IdleMonitor.GetIdletime 2>/dev/null | sed -E 's/^\(uint64 ([0-9]+),\)$/\1/')"
  [[ "$idle_ms" =~ ^[0-9]+$ ]] || { log "机のセッション $sid はあるが無操作の時間が読めない"; return 0; }
  log "机のセッション $sid・無操作 $((idle_ms / 1000)) 秒"
  (( idle_ms / 1000 < IDLE_MIN_S ))
}

running_pid() {
  [[ -s "$PIDFILE" ]] || return 0
  local pid; pid="$(cat "$PIDFILE")"
  if kill -0 "$pid" 2>/dev/null; then echo "$pid"; fi
}

cmd_start() {
  local src="${1:-rtsp://127.0.0.1:8554/cam1}"; shift || true
  local pid; pid="$(running_pid)"
  [[ -z "$pid" ]] || { log "もう動いている（pid $pid）。先に stop"; exit 1; }
  if [[ "${FORCE:-0}" != 1 ]] && desk_in_use; then
    log "机で OMEN が使われている。GPU を使わないので起こさない（本人が机にいるなら FORCE=1）"; exit 1
  fi
  local busy
  busy="$(nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader || true)"
  [[ -z "$busy" ]] || log "GPU を使っている処理がある（そのまま起こす）: $busy"
  log "空きの記憶 $(free -m | awk '/^Mem:/{print $7}') MiB"
  bash "$HERE/start_mediamtx.sh" >/dev/null   # 受け口・送信ページ・表示のサーバー（起きていれば何もしない）

  mkdir -p "$LOG_DIR"
  local stamp; stamp="$(date +%m%d_%H%M%S)"
  local logf="$LOG_DIR/slam${SLAM_ID}_$stamp.log"
  local save="$WORK/results/${stamp}_slam$SLAM_ID"
  local hosts; hosts="$(lan_ip),$TS_IP"
  # 待ち役の bash を ssh のセッションに残す＝ユーザーの systemd が保たれ、scope の中の SLAM も続く
  # （bash ごと exec で scope に移すと、ssh を切ったときにユーザーの systemd ごと止まる）
  # systemd-run --scope は自分を python に入れ替えるので、$! が python の pid になる
  setsid nohup bash -c '
    systemd-run --user --scope --quiet -p MemoryMax="$1" -p MemorySwapMax=0 \
      env TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1 PYTHONUNBUFFERED=1 OPENCV_LOG_LEVEL=ERROR OPENCV_FFMPEG_LOGLEVEL=8 \
      "$2" "$3/$RUNNER" --source "$4" --hosts "$5" --save-dir "$6" \
      --reset-file "$7" "${@:8}" &
    echo $! > "$0"
    wait $!
  ' "$PIDFILE" "$MEM_MAX" "$PY" "$HERE" "$src" "$hosts" "$save" "$RESET_FILE" "$@" \
    >"$logf" 2>&1 </dev/null &
  ln -sfn "$logf" "$LATEST"
  sleep 2
  log "起こした（記録 $logf・保存先 $save）"
  log "重みを読むのに 20〜40 秒かかる。表示の URL は記録に出る:"
  echo "  Mac（Tailscale）: http://$TS_IP:9090/?url=ws://$TS_IP:9877"
  echo "  同じ Wi-Fi     : http://$(lan_ip):9090/?url=ws://$(lan_ip):9877"
}

cmd_stop() {
  local pid; pid="$(running_pid)"
  [[ -n "$pid" ]] || { log "動いていない"; return; }
  # pid は python 本体（systemd-run が入れ替わった物）。SIGINT で保存してから終わる
  kill -INT "$pid"
  for _ in $(seq 60); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
  if kill -0 "$pid" 2>/dev/null; then log "60 秒で終わらない。SIGTERM"; kill -TERM "$pid"; fi
  rm -f "$PIDFILE"
  log "止めた"; tail -n 3 "$LATEST" || true
}

cmd_view() {
  local dir="${1:-$(ls -td "$WORK"/results/*/ 2>/dev/null | head -1)}"
  shift || true   # 残りは view_result.py へ（例 --conf-thresh 3）
  [[ -n "$dir" && -f "$dir/map.ply" ]] || { log "結果が無い（${dir:-results/ が空}）。SLAM を stop すると保存される"; exit 1; }
  bash "$HERE/start_mediamtx.sh" >/dev/null   # 表示のサーバーが動いていなければ起こす
  "$PY" "$HERE/view_result.py" "$dir" --hosts "$(lan_ip),$TS_IP" "$@"
}

case "${1:-start}" in
  stop) cmd_stop ;;
  log) tail -n 40 "$LATEST" ;;
  reset) touch "$RESET_FILE"; log "地図の作り直しを頼んだ（次のこまで作り直す）" ;;
  view) shift; cmd_view "$@" ;;
  start) shift || true; cmd_start "$@" ;;
  *) cmd_start "$@" ;;
esac
