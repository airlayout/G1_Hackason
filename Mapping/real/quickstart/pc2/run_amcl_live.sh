#!/usr/bin/env bash
# **PC2（機体の Orin NX）で Nav2 標準の AMCL による事前地図測位を live で動かす。**
#
#   bash run_amcl_live.sh                              # 既定の初期姿勢で起こす
#   bash run_amcl_live.sh --init "0.703 12.966 -57.5"  # x[m] y[m] yaw[deg]
#   G1_PC2_SECONDS=60 bash run_amcl_live.sh            # 秒数を切る（既定 0 = 無限）
#   bash run_amcl_live.sh --stop                       # 全部落とす
#
# ⚠️ `G1_PC2_SECONDS` は **AMCL が activate してから**数え始める。
#    map_server と amcl の lifecycle 待ちに 10 秒ほどかかるので、
#    実際の稼働は「その値 ＋ 約 11 秒 ＋ 後始末 6 秒」になる（実測: 40 → 57 秒）。
#
# ## 何が動くのか
#
#   dog_odom_to_tf.py   /dog_odom -> TF `odom -> base_link`（2D に潰す）
#                       ＋ 静的 TF `base_link -> livox_frame`
#   cloud_to_scan.py    /utlidar/cloud_livox_mid360 -> /scan（base_link 系・帯 1.3〜1.8 m）
#   map_server          ~/g1_cfg/map/nav_map_run.yaml -> /map（res 0.1 m）
#   amcl                /scan ＋ /map -> TF `map -> odom`
#
# ⚠️ **MOLA は起こさない。** `map -> base_link` と衝突して base_link の親が 2 つになる。
# ⚠️ **controller も loco_driver も起こさない。** ここは測位だけ。`/cmd_vel` は出さない。
#
# ## 罠（`pc2/README.md` の「4.」と同じ）
#
# - `jros2` は**シェル関数**。`timeout jros2 ...` は使えない
# - `LD_LIBRARY_PATH` を export しない（罠 5）。`env.sh` の `jrun` に任せる
# - env.sh の既定は domain 42 ＋ CycloneDDS `lo`。**live は domain 0 ＋ NIC eth0**
# - ⚠️ 自作ノードは `jrun $PREFIX/usr/bin/python3.10 <script>` で起こす。
#   ホストの python3 は focal の 3.8 で rclpy が入っていない
# - ⚠️ **このスクリプトを `&` で背景に置くと SIGINT が効かない**（2026-09-15 に踏んだ）。
#   非対話シェルは背景の子の SIGINT を ignore に設定し、それが孫まで継承される。
#   `G1_PC2_SECONDS` の自動停止が効かず 45 秒のつもりが 199 秒動いた。
#   ⇒ 自動停止は**自分自身に SIGTERM** を送り、後始末は INT → TERM → KILL と上げる。
#   ⇒ 前景で走らせているなら Ctrl-C（SIGINT）で普通に止まる。
# - ⚠️ `pkill -f` はパターンを自分のコマンドラインに見つけて**自分を殺す**。
#   `[n]av2_amcl` のように書いて自己一致を外す
set -uo pipefail

JAMMY="${G1_JAMMY_ROOT:-$HOME/jammy_ros}"
CFG="${G1_PC2_CFG:-$HOME/g1_cfg}"
AMCL_PARAMS="${G1_AMCL_PARAMS:-$CFG/nav2/amcl_live.yaml}"
NAV_MAP="${G1_NAV_MAP:-$CFG/map/nav_map_run.yaml}"
# 自作ノードの置き場。既定は $CFG（配布先）。リポジトリから直接動かすなら
# G1_PC2_NODES にそのディレクトリを渡す
NODES="${G1_PC2_NODES:-$CFG}"
DDS_NIC="${G1_PC2_DDS_NIC:-eth0}"
DOMAIN="${G1_PC2_DOMAIN:-0}"
SECONDS_LIMIT="${G1_PC2_SECONDS:-0}"
# 帯。⚠️ 下限 0.2 未満は禁止（床の反射を拾う）。上限は地図の 1.80 を超えないこと
BAND_LO="${G1_SCAN_MIN_HEIGHT:-1.30}"
BAND_HI="${G1_SCAN_MAX_HEIGHT:-1.80}"
# ⚠️ amcl_live.yaml の laser_max_range と**同じ値**にすること
RANGE_MAX="${G1_SCAN_RANGE_MAX:-20.0}"
TF_RATE="${G1_ODOM_TF_RATE:-100}"

INIT=""
STOP=0
while [ $# -gt 0 ]; do
    case "$1" in
        --init) INIT="$2"; shift 2 ;;
        --stop) STOP=1; shift ;;
        -h|--help) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "[amcl] 知らない引数: $1" >&2; exit 2 ;;
    esac
done

say() { echo "[amcl] $*"; }

# ⚠️ 括弧は**自己一致を外す**ため（`[n]av2_amcl` は "nav2_amcl" に一致するが
# このスクリプト自身のコマンドラインには一致しない）。
# ⚠️ `[m]ap_server` は **Nav2 の map_server も巻き込む**。PC2 で何かを起こす前に
#    `mkdir /tmp/pc2_lock` で排他を取る決まりなので、同時には居ない前提である
KILL_RE='[n]av2_amcl|[c]loud_to_scan\.py|[d]og_odom_to_tf\.py|[m]ap_server'

stop_all() {
    pkill -INT -f "$KILL_RE" 2>/dev/null
    sleep 3
    pkill -TERM -f "$KILL_RE" 2>/dev/null
    sleep 2
    pkill -KILL -f "$KILL_RE" 2>/dev/null
    sleep 1
}

# ── --stop ──────────────────────────────────────────────────────────
if [ "$STOP" = "1" ]; then
    stop_all
    say "止めた。残り $(pgrep -c -f "$KILL_RE" 2>/dev/null || echo 0) 個"
    exit 0
fi

[ -f "$JAMMY/env.sh" ]   || { echo "[amcl] env.sh が無い: $JAMMY/env.sh" >&2; exit 2; }
[ -f "$AMCL_PARAMS" ]    || { echo "[amcl] params が無い: $AMCL_PARAMS" >&2; exit 2; }
[ -f "$NAV_MAP" ]        || { echo "[amcl] 地図が無い: $NAV_MAP" >&2; exit 2; }
for f in cloud_to_scan.py dog_odom_to_tf.py; do
    [ -f "$NODES/$f" ] || { echo "[amcl] ノードが無い: $NODES/$f" >&2; exit 2; }
done

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
CLEANED=0
cleanup() {
    [ "$CLEANED" = "1" ] && return
    CLEANED=1
    # 直接の子（jrun / jros2 のラッパ）と、その孫（実体）の両方を落とす。
    # ⚠️ 孫は `jros2 run` の下に居るので PID では届かない。パターンで落とす
    for p in $PIDS; do kill -INT "$p" 2>/dev/null; done
    stop_all
    say "後始末おわり。残り $(pgrep -c -f "$KILL_RE" 2>/dev/null || echo 0) 個"
}
trap cleanup EXIT INT TERM

# ── 1. odom -> base_link と base_link -> livox_frame ────────────────
say "dog_odom_to_tf.py（/dog_odom -> TF odom->base_link・${TF_RATE} Hz）"
jrun "$PREFIX/usr/bin/python3.10" "$NODES/dog_odom_to_tf.py" \
    --rate "$TF_RATE" > /tmp/pc2_odomtf.log 2>&1 &
PIDS="$PIDS $!"

# ── 2. 3D -> 2D ─────────────────────────────────────────────────────
say "cloud_to_scan.py（帯 ${BAND_LO}〜${BAND_HI} m・range_max ${RANGE_MAX}）"
jrun "$PREFIX/usr/bin/python3.10" "$NODES/cloud_to_scan.py" \
    --min-height "$BAND_LO" --max-height "$BAND_HI" --range-max "$RANGE_MAX" \
    > /tmp/pc2_scan.log 2>&1 &
PIDS="$PIDS $!"

# ── 3. map_server ───────────────────────────────────────────────────
# ⚠️ ライフサイクルノードなので configure/activate を明示的に叩く
# ⚠️ **`run_nav2_live.sh` も map_server を起こす。** 両方動かすと同名ノードが 2 つになり、
#    lifecycle のサービス呼び出しがどちらに当たるか不定になる。Nav2 と併走させるときは
#    `G1_SKIP_MAP_SERVER=1` でこちらを黙らせること（2026-09-15）。
if [ "${G1_SKIP_MAP_SERVER:-0}" = 1 ]; then
    say "map_server は起こさない（G1_SKIP_MAP_SERVER=1。Nav2 側が出す）"
else
    say "map_server（$(basename "$NAV_MAP")）"
    jros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:="$NAV_MAP" -p use_sim_time:=false -p frame_id:=map \
        > /tmp/pc2_mapserver.log 2>&1 &
    PIDS="$PIDS $!"
    sleep 5
    jros2 run nav2_util lifecycle_bringup map_server > /tmp/pc2_mapbringup.log 2>&1
    say "  map_server activated"
fi

# ── 4. AMCL ─────────────────────────────────────────────────────────
AMCL_ARGS="--params-file $AMCL_PARAMS"
if [ -n "$INIT" ]; then
    read -r IX IY IYAW_DEG <<EOF
$INIT
EOF
    IYAW="$(awk -v d="$IYAW_DEG" 'BEGIN{printf "%.7f", d*3.141592653589793/180}')"
    say "初期姿勢を上書き: x=$IX y=$IY yaw=${IYAW_DEG} deg ($IYAW rad)"
    AMCL_ARGS="$AMCL_ARGS -p set_initial_pose:=true"
    AMCL_ARGS="$AMCL_ARGS -p initial_pose.x:=$IX -p initial_pose.y:=$IY"
    AMCL_ARGS="$AMCL_ARGS -p initial_pose.z:=0.0 -p initial_pose.yaw:=$IYAW"
fi
say "amcl を起こす（${DDS_NIC} / domain ${DOMAIN}）"
jros2 run nav2_amcl amcl --ros-args $AMCL_ARGS > /tmp/pc2_amcl.log 2>&1 &
AMCL_PID=$!
PIDS="$PIDS $AMCL_PID"
sleep 5
jros2 run nav2_util lifecycle_bringup amcl > /tmp/pc2_amclbringup.log 2>&1
say "  amcl activated"

say "ログ: /tmp/pc2_amcl.log /tmp/pc2_scan.log /tmp/pc2_odomtf.log /tmp/pc2_mapserver.log"
say "確認: jros2 run tf2_ros tf2_echo map base_link   /   jros2 topic hz /scan"

if [ "$SECONDS_LIMIT" != "0" ]; then
    say "${SECONDS_LIMIT} 秒で止める"
    # ⚠️ 送り先は**このスクリプト自身**で、信号は **SIGTERM**（冒頭の注記）。
    # 背景に置かれた子は SIGINT を無視するので `kill -INT $AMCL_PID` は効かない
    ( sleep "$SECONDS_LIMIT"; kill -TERM $$ 2>/dev/null ) &
fi
wait "$AMCL_PID"
