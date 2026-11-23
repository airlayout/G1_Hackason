#!/usr/bin/env bash
# 2 台のスマホ（cam1・cam2）で 1 つの地図を作る（Multicam-MASt3R-SLAM を直した run_multicam_slam.py）
#
#   bash start_multicam.sh [源…] [引数…]   起こす（既定の源は cam1,cam2 の生の映像）。2 台の映像の録画も始める
#   bash start_multicam.sh stop             止める（カメラごとの軌跡と点群を保存してから）。録画も止める
#   bash start_multicam.sh log              記録の最後を見る
#   bash start_multicam.sh reset            地図を捨てて作り直す
#   bash start_multicam.sh view [結果]      保存した地図と 2 本の軌跡を表示に送る（既定は最新）
#   bash start_multicam.sh rec on|off       録画だけを入れる・切る
#
#   例: bash start_multicam.sh
#       bash start_multicam.sh ~/work_inoue/phone_slam/rec/cam1_….mp4,~/work_inoue/phone_slam/rec/cam2_….mp4 --realtime
#
# スマホ: iPhone は https://<OMEN>:8443/?cam=cam1、Pixel は ?cam=cam2。両手に 1 台ずつ持って歩いてよい。
# 始めは 2 台で同じ所を写す（2 台目が地図に重なる）。重なった後は別々に歩いてよい。
# 表示は単眼と同じ固定の口 http://<IP>:9090/?url=ws://<IP>:9877
#
# 録画は MediaMTX の録画の機能（API で入れ切りする）。$WORK/rec/<名前>_<日時>.mp4 に、送っている間だけ残る。
# 机で人が使っている間は GPU を使わない（start_slam.sh が確かめる。FORCE=1 で越える）。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${WORK:-$HOME/work_inoue/phone_slam}"
API="${MTX_API:-http://127.0.0.1:9997}"
export RUNNER=run_multicam_slam.py
export SLAM_ID="${SLAM_ID:-mc}"
export MEM_MAX="${MEM_MAX:-10G}"
DEFAULT_SRC="rtsp://127.0.0.1:8554/cam1,rtsp://127.0.0.1:8554/cam2"

log() { printf '[multicam] %s\n' "$*"; }

record() {
  local body
  if [[ "$1" == on ]]; then
    mkdir -p "$WORK/rec"
    body="{\"record\": true, \"recordPath\": \"$WORK/rec/%path_%Y%m%d_%H%M%S_%f\", \"recordDeleteAfter\": \"0s\"}"
  else
    body='{"record": false}'
  fi
  if curl -sf -X PATCH "$API/v3/config/paths/patch/all_others" -H "Content-Type: application/json" -d "$body" >/dev/null; then
    log "録画を $1 にした（$WORK/rec/）"
  else
    log "録画を $1 にできなかった（MediaMTX が動いていない？ bash start_mediamtx.sh）"
  fi
}

case "${1:-start}" in
  stop)
    bash "$HERE/start_slam.sh" stop
    record off
    ;;
  log | reset | view) bash "$HERE/start_slam.sh" "$@" ;;
  rec) record "${2:?on か off}" ;;
  start | *)
    [[ "${1:-}" == start ]] && shift
    src="${1:-$DEFAULT_SRC}"
    [[ $# -gt 0 ]] && shift
    bash "$HERE/start_mediamtx.sh" >/dev/null
    extra=()
    if [[ "$src" == rtsp://* ]]; then
      record on
      # 生は MASt3R に渡す画の長い辺を 384 に（512 の約 1.4〜1.6 倍の速さ。TUM の 2 本で誤差 4.0→6.1 cm）。
      # 間に合わないこまを捨てる生では、速さの方が見失いにくさに効く。録画から回し直すときは 512 のまま
      [[ " $* " == *" --img-size "* ]] || extra=(--img-size "${LIVE_IMG_SIZE:-384}")
    fi
    bash "$HERE/start_slam.sh" start "$src" "${extra[@]}" "$@"
    ;;
esac
