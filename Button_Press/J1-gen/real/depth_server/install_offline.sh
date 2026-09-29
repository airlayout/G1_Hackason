#!/usr/bin/env bash
# PC2 で、コピーしてきたファイルから pyrealsense2 を入れる（インターネット不要）。
# fetch_offline_packages.sh が作ったフォルダ（~/button_press/offline/）の中のこのスクリプトを実行する:
#
#   bash ~/button_press/offline/install_offline.sh
#   PYTHON=/path/to/python bash ~/button_press/offline/install_offline.sh   # 別の Python を使うとき
#
# 既定の Python は PC2 のシステムの Python（/usr/bin/python3 = 3.8）。conda の lerobot 環境（3.12）用の
# pyrealsense2 は PC2 の glibc（2.31）では動かなかったため（fetch_offline_packages.sh の説明を参照）。
#
# やること（sudo は使わない。システムは変えない。変わるのは ~/button_press/ の下と pip のパッケージだけ）:
# 1. ファイルが壊れていないかを SHA256SUMS で確かめる
# 2. 使う Python の版に合う pyrealsense2 の wheel を選び、**それが必要とする glibc が PC2 にあるか**を確かめる
#    （無ければ入れずに止める）
# 3. pyrealsense2 を pip で入れる。システムの Python なら --user（~/.local の下）に入れる
# 4. numpy / opencv / pyzmq / PyYAML が読み込めなければ、deps_cp38/ から入れる（あるものは入れない）
# 5. libusb-1.0 がシステムに無ければ、.deb の中身を ~/button_press/libusb_local/ に取り出すだけにする
#    （dpkg -x。インストールはしない）。rgbd_server を start_rgbd_server.sh で起動すると、自動でここを使う
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-/usr/bin/python3}"
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
wheel=$(ls "$HERE"/pyrealsense2-*-cp"$ver"-cp"$ver"-*aarch64*.whl 2>/dev/null | head -1 || true)
if [ -z "$wheel" ]; then
    echo "[install] ❌ Python ${ver:0:1}.${ver:1} 用の pyrealsense2 が無い（用意したのは Python 3.8 用）。" >&2
    echo "[install]    システムの Python（/usr/bin/python3）で実行すること（conda の環境は抜ける: conda deactivate）" >&2
    exit 1
fi

# wheel の中の共有ライブラリが必要とする glibc の最大の版と、PC2 の glibc を比べる
need=$("$PYTHON" - "$wheel" <<'PY'
import re, sys, zipfile
best = (0, 0)
with zipfile.ZipFile(sys.argv[1]) as z:
    for n in z.namelist():
        if ".so" in n:
            for m in re.finditer(rb"GLIBC_(\d+)\.(\d+)", z.read(n)):
                best = max(best, (int(m.group(1)), int(m.group(2))))
print(f"{best[0]}.{best[1]}")
PY
)
have=$(ldd --version 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+$' || echo "0.0")
echo "[install] glibc: 必要 $need、PC2 $have"
if [ "$(printf '%s\n%s\n' "$need" "$have" | sort -V | tail -1)" != "$have" ]; then
    echo "[install] ❌ $(basename "$wheel") は glibc $need 以上が要るが、PC2 は $have。入れずに止める" >&2
    exit 1
fi

# conda / venv の Python ならその中に入れる。システムの Python なら --user（~/.local の下）に入れて、
# sudo もシステムの変更もしない
USER_FLAG=""
if [ "$("$PYTHON" -c 'import sys, os; print(int(sys.prefix != sys.base_prefix or bool(os.environ.get("CONDA_PREFIX"))))')" = "0" ]; then
    USER_FLAG="--user"
    echo "[install] システムの Python なので、--user（~/.local の下）に入れる"
fi
echo "[install] pip で入れる: $(basename "$wheel")"
"$PYTHON" -m pip install --no-index --no-deps $USER_FLAG "$wheel"

if "$PYTHON" -c "import numpy, cv2, zmq, yaml" 2>/dev/null; then
    echo "[install] numpy / opencv / pyzmq / PyYAML はすでに読み込める（入れない）"
else
    echo "[install] numpy / opencv / pyzmq / PyYAML のうち足りないものを deps_cp38/ から入れる"
    for mod_pkg in "numpy:numpy" "cv2:opencv_python_headless" "zmq:pyzmq" "yaml:PyYAML"; do
        mod=${mod_pkg%%:*}; pkg=${mod_pkg##*:}
        if ! "$PYTHON" -c "import $mod" 2>/dev/null; then
            "$PYTHON" -m pip install --no-index --no-deps $USER_FLAG "$HERE"/deps_cp38/"$pkg"-*.whl
        fi
    done
fi

# （grep -q は使わない: pipefail の下では、読み終わる前に閉じた ldconfig が失敗扱いになり、あるのに「無い」と判定した）
if ldconfig -p 2>/dev/null | grep 'libusb-1.0.so.0' > /dev/null || /sbin/ldconfig -p 2>/dev/null | grep 'libusb-1.0.so.0' > /dev/null; then
    echo "[install] libusb-1.0 はシステムに入っている（取り出しは不要）"
    if [ -d "$LIBUSB_DIR" ]; then
        rm -rf "$LIBUSB_DIR"
        echo "[install] 以前に取り出した $LIBUSB_DIR は使わないので消した"
    fi
else
    echo "[install] libusb-1.0 がシステムに無いので、.deb の中身を取り出す: $LIBUSB_DIR"
    rm -rf "$LIBUSB_DIR"
    dpkg -x "$HERE"/libusb-1.0-0_*_arm64.deb "$LIBUSB_DIR"
fi

echo "[install] 確認:"
if [ -d "$LIBUSB_DIR/lib/aarch64-linux-gnu" ]; then
    export LD_LIBRARY_PATH="$LIBUSB_DIR/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
"$PYTHON" -c 'import importlib.metadata as m, pyrealsense2 as rs; print("[install] pyrealsense2", m.version("pyrealsense2"), "OK、RealSense", len(rs.context().query_devices()), "台")'
"$PYTHON" -c 'import numpy, cv2, zmq, yaml; print("[install] numpy", numpy.__version__, "/ opencv", cv2.__version__, "/ pyzmq", zmq.__version__, "OK")'
