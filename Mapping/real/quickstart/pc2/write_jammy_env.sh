#!/usr/bin/env bash
# 展開した jammy の prefix を使うための env.sh と、実行ファイルのラッパを作る。
# setup_jammy_ros.sh から呼ばれるが、単体でも再生成できる。
#
#   bash write_jammy_env.sh ~/jammy_ros
#
# ## 踏んだ罠を 4 つとも塞いである（2026-09-13）
#
# 1. **LD_LIBRARY_PATH を export してはいけない。** そのシェルで動かす focal の
#    コマンド（tail・wc・sed・/bin/sh）まで jammy の libc を掴んで **segfault する**。
#    3 回誤診した（python が落ちていると思ったが、落ちていたのはパイプの相手だった）。
# 2. **PYTHONHOME を設定してはいけない。** 起動時に落ちる。実行ファイル相対で stdlib を解決する。
# 3. **`ros2 launch` は子を execve する。** ELF の PT_INTERP は `/lib/ld-linux-aarch64.so.1`
#    ＝ focal のローダなので GLIBC_2.34 が無くて落ちる ⇒ 実体を退避してラッパを置く。
# 4. **ラッパの shebang は focal の /bin/sh。** 呼び出し側が LD_LIBRARY_PATH を
#    jammy に向けていると **sh 自身が落ちる** ⇒ ラッパの**内側**で export する。
# 5. **`jrun` も LD_LIBRARY_PATH を export してはいけない（2026-09-14 に判明）。**
#    罠 1・4 は「自分のシェル」の話として書いていたが、**子プロセスにも同じことが起きる**。
#    `ros2 launch` を jrun で起こすと LD_LIBRARY_PATH が子に継承され、launch が
#    `mola-cli`（ラッパ）を起こした瞬間に **focal の /bin/sh が jammy の libc を掴んで
#    segfault する**。ラッパの中身は 1 行も実行されない（env ダンプを仕込んでも出ない）。
#    これが「mola-cli は ROS 2 ブリッジ経由だと SIGSEGV」の正体だった。
#    実測: `LD_LIBRARY_PATH=$JAMMY_LIBS /bin/sh -c echo` は **rc=139**、外すと rc=0。
#    ⇒ **`--library-path` だけで足りる。**ld.so の `--library-path` はプロセス全体
#    （dlopen 含む）に効くので、export をやめても機能は落ちない。
#    実測: 外したら `ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py` が
#    30 秒間 died 0 回で回り、BridgeROS2 が全ソースを購読した。
set -uo pipefail

ROOT="${1:-$HOME/jammy_ros}"
PREFIX="$ROOT/rootfs"
ROS="$PREFIX/opt/ros/humble"
LOADER="$PREFIX/lib/ld-linux-aarch64.so.1"
[ -x "$LOADER" ] || { echo "[env] ローダが無い: $LOADER" >&2; exit 1; }

# .so を含むディレクトリを全部集める（実測 134 個ほど）
LIBS=$(find "$PREFIX" -name "*.so*" -type f -printf "%h\n" 2>/dev/null | sort -u | tr "\n" ":")

cat > "$ROOT/env.sh" <<EOF
# jammy の ROS 2 Humble を focal 上で動かす環境（自動生成: write_jammy_env.sh）
# ⚠️ LD_LIBRARY_PATH はここで export しない。ラッパの内側で設定する（罠 1・4）
PREFIX=$PREFIX
ROS=$ROS
LOADER=$LOADER
JAMMY_LIBS="$LIBS"

# ⚠️ PYTHONHOME は入れない（罠 2）
JAMMY_ENV="
AMENT_PREFIX_PATH=$ROS
MOLA_MODULES_LIB_PATH=$ROS/lib:$ROS/lib/aarch64-linux-gnu
PYTHONPATH=$ROS/lib/python3.10/site-packages:$ROS/local/lib/python3.10/dist-packages
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
"

# ⚠️ 実機の DDS に出さない。ループバックに閉じ、domain も分ける。
# 既定のまま回すと再生した /utlidar/cloud_livox_mid360 が実機側に出て live と衝突する
export ROS_DOMAIN_ID=\${G1_JAMMY_DOMAIN:-42}
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'

# jammy の loader で任意の実行ファイルを起動する。
# ⚠️ **LD_LIBRARY_PATH を渡さない（罠 5）。** 渡すと子の focal /bin/sh が落ちる。
# ld.so の --library-path はプロセス全体（dlopen 含む）に効くのでこれで足りる。
jrun() { local exe="\$1"; shift
    env \$JAMMY_ENV \\
        "\$LOADER" --library-path "\$JAMMY_LIBS" "\$exe" "\$@"; }
jros2() { jrun "\$PREFIX/usr/bin/python3.10" "\$ROS/bin/ros2" "\$@"; }
EOF

# ── 実行ファイルにラッパを被せる（罠 3・4）────────────────────────
wrap() {
    local real="$1.real"
    [ -f "$1" ] || return 0
    [ -f "$real" ] || mv "$1" "$real"
    cat > "$1" <<EOF
#!/bin/sh
# jammy の ld.so で実体を起動する。
# ⚠️ LD_LIBRARY_PATH は**この中で**設定する。外で設定すると、この /bin/sh 自身が
#    jammy の libc を掴んで segfault する（罠 4）
LD_LIBRARY_PATH="$LIBS"
export LD_LIBRARY_PATH
exec "$LOADER" --library-path "\$LD_LIBRARY_PATH" "$real" "\$@"
EOF
    chmod +x "$1"
    echo "[env] ラッパ: $(basename "$1")"
}
# ⚠️ **`ros2 launch` が execve する実行ファイルは全部ラップする（罠 3）。**
# ELF の PT_INTERP は `/lib/ld-linux-aarch64.so.1`（＝ focal のローダ）なので、
# そのままだと `error while loading shared libraries: librcl.so` で落ちる。
# 2026-09-14 に Nav2 を入れたら **29 個**増えたので、列挙をやめて全部拾う。
# `.so` と、既にラップ済みの `.real`、ELF でない物（python スクリプト等）は除く。
wrapped=0
for d in "$ROS/bin" "$ROS/lib"; do
    [ -d "$d" ] || continue
    while IFS= read -r f; do
        case "$f" in *.real|*.so|*.so.*) continue ;; esac
        head -c4 "$f" 2>/dev/null | grep -q ELF || continue
        wrap "$f" >/dev/null && wrapped=$((wrapped + 1))
    done <<EOF
$(find "$d" -maxdepth 2 -type f -executable 2>/dev/null)
EOF
done
echo "[env] ラッパ: $wrapped 個"
echo "[env] 書いた: $ROOT/env.sh"
