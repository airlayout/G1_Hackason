#!/usr/bin/env bash
# スマホの代わりに、TUM の録画（fr1_desk）を MediaMTX の流れとして送る（電話なしで通しを試す）
#
#   bash feed_test.sh [流れの名前=cam1] [秒=0（0 は止めるまで繰り返す）]
#
# スマホの Safari と同じく H.264 で送る。30 fps・640x480。
# 繰り返すと最後のこまから最初のこまへ飛ぶ＝SLAM の見失い→再測位の試しにもなる。
set -euo pipefail

WORK="${WORK:-$HOME/work_inoue/phone_slam}"
FFMPEG="${FFMPEG:-$HOME/miniforge3/envs/mast3r/bin/ffmpeg}"
TUM="${TUM:-$WORK/datasets/tum/rgbd_dataset_freiburg1_desk}"
NAME="${1:-cam1}"
SECS="${2:-0}"

[[ -d "$TUM/rgb" ]] || { echo "[feed] $TUM/rgb が無い（setup_omen.sh tum）" >&2; exit 1; }
dur=()
[[ "$SECS" != 0 ]] && dur=(-t "$SECS")

echo "[feed] $TUM → rtsp://127.0.0.1:8554/$NAME（Ctrl-C で止める）"
exec "$FFMPEG" -hide_banner -loglevel warning -re -stream_loop -1 \
  -framerate 30 -pattern_type glob -i "$TUM/rgb/*.png" "${dur[@]}" \
  -c:v libx264 -preset veryfast -tune zerolatency -pix_fmt yuv420p -g 30 -b:v 2M \
  -f rtsp -rtsp_transport tcp "rtsp://127.0.0.1:8554/$NAME"
