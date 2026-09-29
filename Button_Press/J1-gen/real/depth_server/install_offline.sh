#!/usr/bin/env bash
# PC2 で、コピーしてきたファイルから pyrealsense2 を入れる（インターネット不要）。
# fetch_offline_packages.sh が作ったフォルダ（~/button_press/offline/）の中で実行する:
#
#   bash ~/button_press/offline/install_offline.sh
#   PYTHON=/usr/bin/python3 bash ~/button_press/offline/install_offline.sh   # 使う Python を指定する
#
# やること（sudo は使わない。システムは変えない。変わるのは ~/button_press/ の下と pip のパッケージだけ）:
# 1. ファイルが壊れていないかを SHA256SUMS で確かめる
# 2. 使う Python の版（3.8 / 3.10 / 3.12）に合う pyrealsense2 の wheel を pip で入れる
# 3. libusb-1.0 がシステムに無ければ、.deb の中身を ~/button_press/libusb_local/ に取り出すだけにする
#    （dpkg -x。インストールはしない）。rgbd_server を start_rgbd_server.sh で起動すると、自動でここを使う
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-python3}"
LIBUSB_DIR="$HERE/../libusb_local"

echo "[install] ファイルの確認（SHA256SUMS）"
(cd "$HERE" && sha256sum -c SHA256SUMS)

ver=$("$PYTHON" -c 'import sys; print(f"{sys.version_info[0]}{sys.version_info[1]}")')
arch=$(uname -m)
echo "[install] Python: $("$PYTHON" --version 2>&1)（$("$PYTHON" -c 'import sys; print(sys.executable)')）、CPU: $arch"
if [ "$arch" != "aarch64" ]; then
    echo "[install] ❌ aarch64 ではない（$arch）。この配布ファイルは PC2（Jetson）用" >&2
    exit 1
fi
wheel=$(ls "$HERE"/pyrealsense2-*-cp"$ver"-cp"$ver"-manylinux2014_aarch64.whl 2>/dev/null | head -1 || true)
if [ -z "$wheel" ]; then
    echo "[install] ❌ Python ${ver:0:1}.${ver:1} 用の pyrealsense2 が無い（用意したのは 3.8 / 3.10 / 3.12）" >&2
    exit 1
fi
# conda / venv の Python ならその中に入れる。システムの Python なら --user（~/.local の下）に入れて、
# sudo もシステムの変更もしない
USER_FLAG=""
if [ "$("$PYTHON" -c 'import sys, os; print(int(sys.prefix != sys.base_prefix or bool(os.environ.get("CONDA_PREFIX"))))')" = "0" ]; then
    USER_FLAG="--user"
    echo "[install] conda / venv ではないシステムの Python なので、--user（~/.local の下）に入れる"
fi
echo "[install] pip で入れる: $(basename "$wheel")"
"$PYTHON" -m pip install --no-index --no-deps $USER_FLAG "$wheel"

if ldconfig -p | grep -q 'libusb-1.0.so.0'; then
    echo "[install] libusb-1.0 はシステムに入っている（取り出しは不要）"
else
    echo "[install] libusb-1.0 がシステムに無いので、.deb の中身を取り出す: $LIBUSB_DIR"
    rm -rf "$LIBUSB_DIR"
    dpkg -x "$HERE"/libusb-1.0-0_*_arm64.deb "$LIBUSB_DIR"
fi

echo "[install] 確認:"
if [ -d "$LIBUSB_DIR/lib/aarch64-linux-gnu" ]; then
    export LD_LIBRARY_PATH="$LIBUSB_DIR/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
"$PYTHON" -c 'import pyrealsense2 as rs; print("[install] pyrealsense2", rs.__version__, "OK、RealSense", len(rs.context().query_devices()), "台")'
