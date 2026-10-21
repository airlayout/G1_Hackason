#!/usr/bin/env bash
# PC2 で深度付きカメラサーバを起動する。install_offline.sh が libusb を取り出していれば、その場所を使う。
#
#   bash ~/button_press/real/depth_server/start_rgbd_server.sh
#   bash ~/button_press/real/depth_server/start_rgbd_server.sh --list-devices
#   PYTHON=/path/to/python bash ~/button_press/real/depth_server/start_rgbd_server.sh   # 別の Python を使うとき
#
# 引数はそのまま rgbd_server.py に渡す。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
# 既定は PC2 のシステムの Python（3.8）。pyrealsense2 はここに入れる（install_offline.sh を参照）
PYTHON="${PYTHON:-/usr/bin/python3}"
LIBUSB_DIR="$ROOT/libusb_local/lib/aarch64-linux-gnu"

if [ -d "$LIBUSB_DIR" ]; then
    echo "[start] 取り出した libusb を使う: $LIBUSB_DIR"
    export LD_LIBRARY_PATH="$LIBUSB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
exec "$PYTHON" -u "$HERE/rgbd_server.py" "$@"
