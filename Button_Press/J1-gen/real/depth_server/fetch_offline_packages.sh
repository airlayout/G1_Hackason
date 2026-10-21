#!/usr/bin/env bash
# PC2（インターネットにつながっていない可能性がある）へ持っていくファイルを、ネットにつながったマシンで
# _local/button_press/offline/ にダウンロードする。
#
#   bash Button_Press/J1-gen/real/depth_server/fetch_offline_packages.sh
#
# PC2（Ubuntu 20.04、glibc 2.31、aarch64）で動くものだけを用意する（2026-09-29 に PC2 で確かめた）:
# - pyrealsense2 2.55.1（Python 3.8 = PC2 のシステムの Python 用。必要な glibc は 2.17）
#   Python 3.10 / 3.12 用の aarch64 版は、名前に manylinux2014 と付いていても中身が glibc 2.34 / 2.38 を
#   必要とし、PC2 では読み込めなかった（GLIBC_2.38 not found）。そのため conda の lerobot 環境（3.12）ではなく、
#   システムの Python 3.8 で配信サーバを動かす
# - deps_cp38/: 配信サーバが使う numpy / opencv-python-headless / pyzmq / PyYAML の Python 3.8 用
#   （PC2 のシステムの Python に無いときだけ install_offline.sh が入れる。どれも必要な glibc は 2.25 以下）
# - libusb-1.0 の .deb（Ubuntu 20.04 focal、arm64）。PC2 にシステムの libusb が無いときだけ、中身を取り出して使う
#
# どのマシン（x86_64 でも aarch64 でも）で実行してもよい。pip の --platform で aarch64 用を指定して取る。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
OUT="$REPO_ROOT/_local/button_press/offline"
PIP="${PIP:-$REPO_ROOT/G1_HuggingFace/venv/bin/pip}"

PYREALSENSE2="pyrealsense2==2.55.1.6486"
DEPS=("numpy==1.24.4" "opencv-python-headless==4.8.1.78" "pyzmq==27.1.0" "PyYAML==6.0.3")
PLATFORMS=(--platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64 --platform manylinux_2_28_aarch64)

# libusb-1.0-0（Ubuntu 20.04 focal の arm64 版）。SHA256 は ports.ubuntu.com の
# dists/focal/main/binary-arm64/Packages.gz に載っている値（2026-09-28 に確認）
LIBUSB_DEB="libusb-1.0-0_1.0.23-2build1_arm64.deb"
LIBUSB_URL="http://ports.ubuntu.com/pool/main/libu/libusb-1.0/${LIBUSB_DEB}"
LIBUSB_SHA256="1b6ed22a65e408c16f5ab6201dd34b674eebed40995fd9cefb31ecfc328b0472"

rm -rf "$OUT"
mkdir -p "$OUT/deps_cp38"
echo "[offline] 保存先: $OUT"

echo "[offline] $PYREALSENSE2（Python 3.8、aarch64）"
"$PIP" download "$PYREALSENSE2" --no-deps --only-binary=:all: "${PLATFORMS[@]}" --python-version 3.8 -d "$OUT" -q

echo "[offline] ${DEPS[*]}（Python 3.8、aarch64）"
"$PIP" download "${DEPS[@]}" --no-deps --only-binary=:all: "${PLATFORMS[@]}" --python-version 3.8 -d "$OUT/deps_cp38" -q

echo "[offline] $LIBUSB_DEB"
curl -sSfL -o "$OUT/$LIBUSB_DEB" "$LIBUSB_URL"
echo "$LIBUSB_SHA256  $OUT/$LIBUSB_DEB" | sha256sum -c -

cp "$SCRIPT_DIR/install_offline.sh" "$OUT/"
(cd "$OUT" && sha256sum ./*.whl ./deps_cp38/*.whl ./*.deb install_offline.sh > SHA256SUMS)
echo "[offline] 完了:"
ls -l "$OUT" "$OUT/deps_cp38"
