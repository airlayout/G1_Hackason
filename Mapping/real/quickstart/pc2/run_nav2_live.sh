#!/usr/bin/env bash
# **PC2（機体の Orin NX）で Nav2 を live で動かす。**
#
#   bash run_nav2_live.sh                 # map_server ＋ Nav2
#   G1_PC2_SECONDS=60 bash run_nav2_live.sh
#
# ⚠️ **先に `run_mola_live.sh --map ...` を起こしておくこと。**
# Nav2 は `map -> base_link` の TF が無いと経路を引けない。それを出すのは MOLA である。
#
# ## PC2 に Nav2 を載せるまでに踏んだこと（2026-09-14）
#
# 1. **PC2 にインターネットが無い。** eth0 の既定経路 192.168.123.1 は機体内部の
#    スイッチ上に存在せず、AP 側（10.42.0.1）にも出口が無い。
#    ⇒ **Mac のコンテナ（jammy/arm64）で deb を取り、tar でパイプして配った。**
#    setup_jammy_ros.sh と同じ「空の dpkg status で閉包を全部引く」やり方が使える。
#    実測: 閉包 1006 個 456.8MB のうち、PC2 に無いのは 500 個 187MB だけだった。
# 2. **`ros2 run` が無かった。** MOLA 用の閉包には ros2run が含まれない。
#    map_server は `ros2 run` で起こすので、CLI 一式（run/topic/param/service/
#    node/lifecycle/action）を追加した（8 deb・172KB）。
# 3. ⚠️ **Nav2 のノードも罠 3 を踏む。** ELF の PT_INTERP が focal の ld.so なので
#    `librcl.so: cannot open shared object file` で落ちる。
#    ⇒ write_jammy_env.sh を「$ROS/bin と $ROS/lib の ELF を全部ラップする」形に直した
#    （**71 個**。列挙をやめた）。
# 4. **params の BT パスがコンテナ用だった。** `/work/G1_Hackason/...` を
#    PC2 のパスに書き換えたものを g1_cfg/nav2/ に置く。
set -uo pipefail

JAMMY="${G1_JAMMY_ROOT:-$HOME/jammy_ros}"
CFG="${G1_PC2_CFG:-$HOME/g1_cfg}"
NAV2_PARAMS="${G1_NAV2_PARAMS:-$CFG/nav2/g1_nav2.yaml}"
NAV_MAP="${G1_NAV_MAP:-$CFG/map/nav_map_run.yaml}"
DDS_NIC="${G1_PC2_DDS_NIC:-eth0}"
DOMAIN="${G1_PC2_DOMAIN:-0}"
SECONDS_LIMIT="${G1_PC2_SECONDS:-0}"

say() { echo "[nav2] $*"; }

[ -f "$JAMMY/env.sh" ]   || { echo "[nav2] env.sh が無い: $JAMMY/env.sh" >&2; exit 2; }
[ -f "$NAV2_PARAMS" ]    || { echo "[nav2] params が無い: $NAV2_PARAMS" >&2; exit 2; }

# shellcheck source=/dev/null
. "$JAMMY/env.sh"
export ROS_DOMAIN_ID="$DOMAIN"
# ⚠️ **NIC を 2 つにしたいときは `G1_PC2_DDS_URI` で丸ごと差し替える。**（2026-09-15）
#    eth0 だけだと participant の locator が 192.168.123.164 になり、AP 越しの
#    コンテナ（192.168.123.201）からは DDS が見えない。wlan0 も列挙すると見える。
if [ -n "${G1_PC2_DDS_URI:-}" ]; then
    export CYCLONEDDS_URI="$G1_PC2_DDS_URI"
else
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$DDS_NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
fi

PIDS=""
cleanup() {
    # ⚠️ **SIGINT で落とす。** SIGTERM だと内側の ros2 launch が子を孤児にする
    for p in $PIDS; do kill -INT "$p" 2>/dev/null; done
    sleep 2
    for p in $PIDS; do kill -9 "$p" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

# ── map_server（Nav2 の static layer が読む /map を出す）─────────────
# ⚠️ ライフサイクルノードなので configure/activate を明示的に叩く必要がある。
# 2026-09-07 に取りこぼして静的レイヤが空のままだった前例がある。
if [ -f "$NAV_MAP" ]; then
    say "map_server を起こす（$(basename "$NAV_MAP")）"
    jros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:="$NAV_MAP" -p use_sim_time:=false -p frame_id:=map \
        > /tmp/pc2_mapserver.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 5
    jros2 run nav2_util lifecycle_bringup map_server > /tmp/pc2_mapbringup.log 2>&1
    say "  lifecycle_bringup 完了"
else
    say "⚠️ $NAV_MAP が無い。静的レイヤは空になる"
fi

# ── Nav2 ────────────────────────────────────────────────────────────
say "Nav2 を起こす（$(basename "$NAV2_PARAMS")）"
jros2 launch nav2_bringup navigation_launch.py \
    params_file:="$NAV2_PARAMS" use_sim_time:=false > /tmp/pc2_nav2.log 2>&1 &
NAV2_PID=$!
PIDS="$PIDS $NAV2_PID"

if [ "$SECONDS_LIMIT" != "0" ]; then
    say "${SECONDS_LIMIT} 秒で止める"
    ( sleep "$SECONDS_LIMIT"; kill -INT "$NAV2_PID" 2>/dev/null ) &
fi
say "ログ: /tmp/pc2_nav2.log /tmp/pc2_mapserver.log"
wait "$NAV2_PID"
