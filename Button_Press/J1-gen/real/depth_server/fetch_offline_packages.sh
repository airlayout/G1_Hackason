#!/usr/bin/env bash
# PC2（インターネットにつながっていない可能性がある）へ持っていくファイルを、ネットにつながったマシンで
# _local/button_press/offline/ にダウンロードする。
#
#   bash Button_Press/J1-gen/real/depth_server/fetch_offline_packages.sh
#
# ダウンロードするもの:
# - pyrealsense2 の aarch64（Jetson と同じ CPU）用の配布ファイル（wheel）: Python 3.8 / 3.10 / 3.12 用
#   （PC2 の Python の版が未確定のため。3.12 は SETUP.md 3.2 の conda 環境の版）
# - libusb-1.0 の .deb（Ubuntu 20.04 focal、arm64）。pyrealsense2 が使う USB のライブラリ。
#   PC2 には入れず、中身を取り出して使う（install_offline.sh。システムは変えない）
#
# どのマシン（x86_64 でも aarch64 でも）で実行してもよい。pip の --platform で aarch64 用を指定して取る。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
OUT="$REPO_ROOT/_local/button_press/offline"
PIP="${PIP:-$REPO_ROOT/G1_HuggingFace/venv/bin/pip}"

# libusb-1.0-0（Ubuntu 20.04 focal の arm64 版）。SHA256 は ports.ubuntu.com の
# dists/focal/main/binary-arm64/Packages.gz に載っている値（2026-09-28 に確認）
LIBUSB_DEB="libusb-1.0-0_1.0.23-2build1_arm64.deb"
LIBUSB_URL="http://ports.ubuntu.com/pool/main/libu/libusb-1.0/${LIBUSB_DEB}"
LIBUSB_SHA256="1b6ed22a65e408c16f5ab6201dd34b674eebed40995fd9cefb31ecfc328b0472"

mkdir -p "$OUT"
echo "[offline] 保存先: $OUT"

for pv in 3.8 3.10 3.12; do
    echo "[offline] pyrealsense2（Python $pv、aarch64）"
    "$PIP" download pyrealsense2 --only-binary=:all: --platform manylinux2014_aarch64 \
        --python-version "$pv" -d "$OUT" -q
done

echo "[offline] $LIBUSB_DEB"
curl -sSfL -o "$OUT/$LIBUSB_DEB" "$LIBUSB_URL"
echo "$LIBUSB_SHA256  $OUT/$LIBUSB_DEB" | sha256sum -c -

cp "$SCRIPT_DIR/install_offline.sh" "$OUT/"
(cd "$OUT" && sha256sum ./*.whl ./*.deb install_offline.sh > SHA256SUMS)
echo "[offline] 完了:"
ls -l "$OUT"
