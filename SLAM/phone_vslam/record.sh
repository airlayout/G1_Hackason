#!/usr/bin/env bash
# スマホから届いている流れを、そのまま（符号を解かずに）OMEN に録る
#
#   bash record.sh [流れの名前=cam1] [秒=300]
#
# 録れた物は $WORK/rec/<名前>_<日時>.mkv。終わったら stream_report.py で
# 解像度の動き・fps・こまの抜け（間が空いた所）をまとめる（段 0 の記録）。
# mkv にするのは、Safari が VP8 で送ってきても入れられるようにするため（mp4 には VP8 が入らない）。
set -euo pipefail

WORK="${WORK:-$HOME/work_inoue/phone_slam}"
ENV_BIN="${ENV_BIN:-$HOME/miniforge3/envs/mast3r/bin}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="${1:-cam1}"
SECS="${2:-300}"

mkdir -p "$WORK/rec"
out="$WORK/rec/${NAME}_$(date +%m%d_%H%M%S).mkv"
echo "[rec] rtsp://127.0.0.1:8554/$NAME を $SECS 秒録る → $out"
"$ENV_BIN/ffmpeg" -hide_banner -loglevel warning -rtsp_transport tcp \
  -i "rtsp://127.0.0.1:8554/$NAME" -t "$SECS" -c copy -an "$out"
echo "[rec] 録れた: $out"
"$ENV_BIN/python" "$HERE/stream_report.py" "$out"
