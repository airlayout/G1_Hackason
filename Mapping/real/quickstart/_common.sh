#!/usr/bin/env bash
# Mac 側スクリプトの共有部分。**これは実行するものではなく source するもの。**
#
#   HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#   . "$HERE/_common.sh"
#
# ## なぜ切り出したか
#
# CycloneDDS の URI（XML）が **6 ファイル・8 箇所**に散っていた（2026-09-07 に数えた）。
# しかも 1 文字違うと「G1 が見えない」という同じ症状になり、切り分けに時間がかかる。
# 手で `docker exec -e RMW_IMPLEMENTATION=... -e CYCLONEDDS_URI='<CycloneDDS>...'` を
# 打つ機会も多く、2026-09-07 の 1 セッションで 10 回以上打った。
#
# ## ⚠️ ここは Mac 側だけを扱う。PC2 側と混ぜないこと
#
# | 側 | cyclonedds | 書式 |
# |---|---|---|
# | **Mac のコンテナ（ここ）** | 0.10 系 | `<Interfaces><NetworkInterface name="..."/>` |
# | PC2 の pixi 環境 | 0.10 系 | 同じだが**配布が heredoc 経由**で `$IFACE` の展開の扱いが違う |
# | PC2 の foxy | **0.7** | `<NetworkInterfaceAddress>`。**0.10 の書式では domain 作成ごと失敗する** |
#
# PC2 側（`start_foxglove_bridge.sh` / `start_odom_tf.sh` / `start_restamp.sh` /
# `diagnose_ros2.sh`）はここを source しない。**版が違うものを 1 つにまとめてはいけない。**

# ── 場所と名前 ───────────────────────────────────────────────────────
G1_QUICKSTART_DIR="${G1_QUICKSTART_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
# ワークスペースの根（physical_ai）。コンテナには /work として見せている
G1_REPO_ROOT="${G1_REPO_ROOT:-$(cd "$G1_QUICKSTART_DIR/../../../.." && pwd)}"
G1_WORK="${G1_WORK:-/work/G1_Hackason/Mapping/real}"

G1_MAC_IFACE="${G1_MAC_IFACE:-en8}"       # 有線 NIC。USB アダプタなので挿すまで現れない
G1_MAC_IP="${G1_MAC_IP:-192.168.123.202}" # ⚠️ 固定でない。他の作業者に取られることがある
G1_VM_IP="${G1_VM_IP:-192.168.123.201}"
G1_VM_NIC="${G1_VM_NIC:-col0}"            # colima が bridged で足す NIC
G1_PC1_IP="${G1_PC1_IP:-192.168.123.161}" # ⚠️ ポート全閉。ping は通るが何も開いていない
G1_PC2_IP="${G1_PC2_IP:-192.168.123.164}"
G1_PC2_SSH_PORT="${G1_PC2_SSH_PORT:-22}"
G1_RVIZ_NAME="${G1_RVIZ_NAME:-rviz}"
G1_RVIZ_IMAGE="${G1_RVIZ_IMAGE:-tiryoh/ros2-desktop-vnc:humble}"
G1_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ── CycloneDDS の URI（**唯一の定義箇所**）─────────────────────────
# DDS を必ずブリッジ側の NIC に載せる。VM には NAT の eth0 もあるので、
# 指定しないと CycloneDDS がそちらを選んで G1 が見えない。
g1_dds_uri_live() {
    printf '%s' "<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$G1_VM_NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
}
# offline では col0 が無い。ループバックに閉じないと CycloneDDS が NIC を選べない
g1_dds_uri_offline() {
    printf '%s' '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
}
# mode = live | offline
g1_dds_uri() {
    case "${1:-live}" in
        offline) g1_dds_uri_offline ;;
        *)       g1_dds_uri_live ;;
    esac
}

# ── コンテナでコマンドを走らせる ───────────────────────────────────
# ⚠️ `bash -lc` は使わない。ログインシェルが環境を作り直して XAUTHORITY を落とす
# （2026-09-06 に GUI が上がらなくて踏んだ）。
#
#   g1_exec        live "ros2 topic list"        # 前景。出力が返る
#   g1_exec_bg     live foo.log "ros2 run ..."   # 背景。ログはコンテナの ~ubuntu に出る
#
# 第 1 引数は mode（live / offline）。DDS の載せ先が変わる。
g1_exec() {
    local mode="$1"; shift
    docker exec -u ubuntu \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e CYCLONEDDS_URI="$(g1_dds_uri "$mode")" \
        -e ROS_DOMAIN_ID="$G1_ROS_DOMAIN_ID" \
        "$G1_RVIZ_NAME" bash -c "source /opt/ros/humble/setup.bash && $*"
}

g1_exec_bg() {
    local mode="$1"; local log="$2"; shift 2
    docker exec -d -u ubuntu \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e CYCLONEDDS_URI="$(g1_dds_uri "$mode")" \
        -e ROS_DOMAIN_ID="$G1_ROS_DOMAIN_ID" \
        "$G1_RVIZ_NAME" bash -c "source /opt/ros/humble/setup.bash && $* > /home/ubuntu/$log 2>&1"
}

# root で走らせたいとき（apt など）。DDS は要らない
g1_exec_root() {
    docker exec "$G1_RVIZ_NAME" bash -c "$*"
}

# ── 疎通の測り方 ───────────────────────────────────────────────────
# ⚠️ **ping を使ってはいけない。**
# `ping` は **colima の VM にもコンテナにも入っていない**。
# 「到達しない」ではなく「コマンドが無い」で失敗するので、
# 2026-09-07 に 2 回誤診した（`&&` で判定していたため区別がつかなかった）。
# TCP の接続可否で測る。
g1_tcp_ok() {
    local host="$1" port="$2" timeout="${3:-3}"
    # /dev/tcp は bash の組み込みなので外部コマンドが要らない。
    # timeout(1) も無い環境があるので、bash の SECONDS ではなく read のタイムアウトに頼らず
    # サブシェルを kill する形にする
    (
        exec 3<>"/dev/tcp/$host/$port"
    ) >/dev/null 2>&1 &
    local pid=$!
    local waited=0
    while kill -0 "$pid" 2>/dev/null; do
        [ "$waited" -ge "$((timeout * 10))" ] && { kill -9 "$pid" 2>/dev/null; wait "$pid" 2>/dev/null; return 1; }
        waited=$((waited + 1))
        sleep 0.1
    done
    wait "$pid" 2>/dev/null
}

# Mac 側で 192.168.123.0/24 を持っている NIC の名前を返す（無ければ空）
g1_wired_nic() {
    ifconfig -a 2>/dev/null \
        | awk '/^[a-z0-9]+:/{n=substr($1,1,length($1)-1)} /inet 192\.168\.123\./{print n; exit}'
}

# ── ブリッジ（VM/コンテナ → 実機）が生きているか ───────────────────
# col0 は socket_vmnet 経由で G1 の内蔵スイッチに L2 で載っている。
# **LAN ケーブルを抜き差しするとこの結びつきが黙って外れる。** 紛らわしいのは
#
#   * Mac → PC2 は**通ったまま**（ホストの経路は別）
#   * col0 には IPv4 が**付いたまま**
#   * `ros2 topic list` は daemon が握った**古い結果**を返す
#
# ので、実データが 1 件も来ないところまで行かないと気づけない。
# 判定は check_link.sh 手順 5 の後半（コンテナ → PC2）と**同じものを共有する**。
G1_TCP_PROBE_TIMEOUT="${G1_TCP_PROBE_TIMEOUT:-5}"

# コンテナが起動しているか（--network host なので VM と同じ netns に居る）
g1_container_running() {
    [ "$(docker inspect -f '{{.State.Running}}' "${1:-$G1_RVIZ_NAME}" 2>/dev/null)" = "true" ]
}

# コンテナから TCP が張れるか。⚠️ ping はコンテナに入っていないので使えない
g1_tcp_ok_container() {
    local host="$1" port="$2"
    docker exec "$G1_RVIZ_NAME" timeout "$G1_TCP_PROBE_TIMEOUT" \
        bash -c "exec 3<>/dev/tcp/$host/$port" >/dev/null 2>&1
}

# VM から TCP が張れるか（コンテナが無い／止まっているとき用）。
# ⚠️ `colima ssh -- ...` は argv をそのまま渡す（自分でクォートし直す必要は無い。
# 逆に `bash -c "'...'"` と二重にすると引用符ごと 1 語になって command not found になる）。
# ping は VM にも入っていない。timeout(1) と bash は colima の Ubuntu に入っている
g1_tcp_ok_vm() {
    local host="$1" port="$2"
    colima ssh -- timeout "$G1_TCP_PROBE_TIMEOUT" \
        bash -c "exec 3<>/dev/tcp/$host/$port" >/dev/null 2>&1
}

# どちらの経路で測れるかを返す（container / vm / none）。**ログに出すために名前で返す**
g1_bridge_via() {
    if g1_container_running; then
        echo container
    elif colima status >/dev/null 2>&1; then
        echo vm
    else
        echo none
    fi
}

# 上で決めた経路で PC2 へ TCP を張る。0=生きている / 1=切れている / 2=測れない
g1_bridge_ok() {
    case "${1:-$(g1_bridge_via)}" in
        container) g1_tcp_ok_container "$G1_PC2_IP" "$G1_PC2_SSH_PORT" ;;
        vm)        g1_tcp_ok_vm        "$G1_PC2_IP" "$G1_PC2_SSH_PORT" ;;
        *)         return 2 ;;
    esac
}

# ── 見た目 ─────────────────────────────────────────────────────────
g1_say()  { echo "[${G1_TAG:-g1}] $*"; }
g1_die()  { echo "[${G1_TAG:-g1}] $*" >&2; exit 1; }
g1_ok()   { echo "  OK   $*"; }
g1_bad()  { echo "  NG   $*"; }
g1_warn() { echo "  --   $*"; }
