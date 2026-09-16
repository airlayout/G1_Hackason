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

# ── 静的レイヤの出どころ（2026-09-16 に octomap を既定にした）──────────
#   octomap    … octomap_server が /projected_map を出す。**机が動いても追従する**
#   map_server … 固定の nav_map_run を /map に出す（従来。育たない）
# ⚠️ どちらを選ぶかは `g1_nav2.yaml` の `static_layer.map_topic` と**一対**である。
STATIC_SOURCE="${G1_STATIC_SOURCE:-octomap}"
OCTOMAP_SEED="${G1_OCTOMAP_SEED:-$CFG/map/seed.bt}"
CLOUD_TOPIC="${G1_CLOUD_TOPIC:-/utlidar/cloud_livox_mid360}"
# 以下 4 つは 2026-09-16 に測って決めた。**既定値は全部危ない**（根拠は下の注記）
OCTOMAP_BAND_MIN="${G1_OCTOMAP_BAND_MIN:-0.23}"
OCTOMAP_BAND_MAX="${G1_OCTOMAP_BAND_MAX:-1.80}"
OCTOMAP_MAX_RANGE="${G1_OCTOMAP_MAX_RANGE:-4.0}"
OCTOMAP_RESOLUTION="${G1_OCTOMAP_RESOLUTION:-0.1}"

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

# ── 静的レイヤの出どころ ─────────────────────────────────────────────
if [ "$STATIC_SOURCE" = "octomap" ] && [ ! -f "$OCTOMAP_SEED" ]; then
    say "⚠️ 種が無い（$OCTOMAP_SEED）。map_server に落とす"
    say "   作り方: Mac で quickstart/octomap_seed_from_nav_map.py runs/<id>"
    STATIC_SOURCE="map_server"
fi

if [ "$STATIC_SOURCE" = "octomap" ]; then
    # ⚠️ **既定値は 4 つとも危ない。**2026-09-16 に測った（quickstart/verify_octomap_seed.sh）:
    #
    #   occupancy_min_z / max_z  既定 ±100 ＝床と天井が入って**部屋全体が障害物になる**。
    #       ここは下の min_obstacle_height / max_obstacle_height と**同じ値**にする。
    #       下限を上げると占有セルが減る: 0.23→19,997 / 0.50→17,613 / 0.85→13,145 /
    #       1.30→7,509。壁の帯も 93.5→82.3 % に落ちる。**上げる理由は無い**
    #   sensor_model.max_range   既定 -1.0 ＝無制限。10 Hz で測った比較:
    #       | max_range | CPU | 種の残存 | 消えた | 空きの増分 |
    #       | -1.0      |10.7%|  96.6 %  |  683   | +15,606    |
    #       | 4.0       | 3.9%|  99.9 %  |   17   |    -615    |
    #       無制限は未知を空きに変えてくれるが、**静止構造を 683 セル消す**。
    #       「消しすぎ」は過去 2 回裏目に出ている（迂回ゼロ 31→7・危険な食い違い 119→307）
    #       ので構造を守る側に取る。**4.0 m**
    #   latch                    既定 true。true だと octomap_full / octomap_binary /
    #       marker_array を**購読者の有無に関わらず毎スキャン出す**ので CPU が 4 倍
    #       （10 Hz で 48.7 % 対 13.5 %）。PC2 は load 5〜7/8 なので false
    #       ⚠️ false ＝ VOLATILE なので `map_subscribe_transient_local: false` と一対
    say "octomap_server を起こす（種 $(basename "$OCTOMAP_SEED") / 帯 ${OCTOMAP_BAND_MIN}-${OCTOMAP_BAND_MAX} m / max_range ${OCTOMAP_MAX_RANGE} m）"
    jros2 run octomap_server octomap_server_node --ros-args \
        -r cloud_in:="$CLOUD_TOPIC" \
        -p octomap_path:="$OCTOMAP_SEED" \
        -p frame_id:=map -p resolution:="$OCTOMAP_RESOLUTION" \
        -p occupancy_min_z:="$OCTOMAP_BAND_MIN" \
        -p occupancy_max_z:="$OCTOMAP_BAND_MAX" \
        -p sensor_model.max_range:="$OCTOMAP_MAX_RANGE" \
        -p latch:=false -p use_sim_time:=false \
        > /tmp/pc2_octomap.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 8
    if grep -q 'loaded (' /tmp/pc2_octomap.log; then
        grep 'loaded (' /tmp/pc2_octomap.log | sed 's/^/[nav2]   /'
    else
        say "⚠️ 種を読めていない。/tmp/pc2_octomap.log を見ること"
    fi
elif [ -f "$NAV_MAP" ]; then
    # ⚠️ ライフサイクルノードなので configure/activate を明示的に叩く必要がある。
    # 2026-09-07 に取りこぼして静的レイヤが空のままだった前例がある。
    say "map_server を起こす（$(basename "$NAV_MAP")）"
    jros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:="$NAV_MAP" -p use_sim_time:=false -p frame_id:=map \
        > /tmp/pc2_mapserver.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 5
    jros2 run nav2_util lifecycle_bringup map_server > /tmp/pc2_mapbringup.log 2>&1
    say "  lifecycle_bringup 完了"
    say "⚠️ この構成では g1_nav2.yaml の static_layer.map_topic を /map に戻すこと"
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
say "ログ: /tmp/pc2_nav2.log /tmp/pc2_octomap.log /tmp/pc2_mapserver.log"
wait "$NAV2_PID"
