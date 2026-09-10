#!/usr/bin/env bash
# Mac のコンテナで RViz2 を動かし、G1 の内蔵スイッチ（L2）に載せる。表示はブラウザ。
#
#   bash quickstart/start_rviz_mac.sh          # 起動（冪等。二度打ちしてよい）
#   bash quickstart/start_rviz_mac.sh stop     # コンテナを止める（VM は残す）
#   bash quickstart/start_rviz_mac.sh down     # VM ごと止める
#
# 2026-09-06 に実機で一巡した。LiDAR が 9.97 Hz / 4.58 MB/s で取りこぼしなく届き、
# RViz2 が 31 fps で描画した（M4 / ソフトウェア描画）。
#
# ## なぜコンテナか
# RViz2 は Mac に native で入らない。X 転送（PC2 で RViz2 → XQuartz）は
# コンテナ → VM → macOS と 2 段またぐ配管が要るうえ、indirect GLX の壁で結局
# ソフトウェア描画になる。X11 をコンテナ内で完結させ、ブラウザで見る方が段が少ない。
#
# ## なぜブリッジか
# colima の既定（shared）では VM が NAT の内側に居るため、DDS の discovery が
# G1 の L2 に届かない。bridged で VM を L2 の一員にすると、ROS 2 の通常の分散構成が
# そのまま成立する（橋もトンネルもアドレスの詐称も要らない）。
#
# ## 一度だけ人手が要る
# 初回の `colima start` は /private/etc/sudoers.d/colima を置くために sudo パスワードを
# 聞く。**tty が要るので、初回だけは自分の手で端末から実行すること。**
# 一度入れば以降は聞かれない。
#
# ## ケーブルを抜き差ししたら Mac の IP を疑う（2026-09-06 に踏んだ）
# G1 の内蔵スイッチには**他の作業者も居る。** 抜いている間に 192.168.123.200 を
# 別のマシン（b4:e2:5b:5e:6c:03）に取られ、macOS が重複を検知して IP を放棄した。
# 設定は Manual .200 のまま残るので、サービスを off/on しても永久に戻らない。
#
#   ping -c1 192.168.123.164            # 届かない
#   ifconfig en8 | grep 'inet '         # IPv4 が無い（status は active）
#   colima ssh -- ip neigh show dev col0 # .200 の lladdr が自分の MAC と違えば衝突
#   sudo networksetup -setmanual "AX88179B" 192.168.123.202 255.255.255.0 ""
#
# **VM 側は巻き添えにならない。** col0 は socket_vmnet 経由で L2 に直接載っており、
# ホストの IP とは独立している。実際このとき DDS も RViz2 も動き続けた。
# Mac から .201 に届かない間も、RViz2 は http://localhost/ で見られる
# （colima のポートフォワードが --network host のコンテナにも効く）。
# ## ブリッジが切れたら自分で直す（2026-09-10 に自動化した）
# ケーブルの抜き差しで col0 と G1 の内蔵スイッチの結びつきが外れる。
# **Mac → PC2 は通ったまま**で col0 の IPv4 も残るので、`ros2 topic echo` が
# 何も返さないところまで行かないと気づけない。live のときだけ起動前に検査し、
# 切れていたら `colima stop` してから下の起動処理を通す（`G1_COLIMA_RESTART`）。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# DDS の URI と IP/名前の既定値はここ 1 箇所に置いてある。
# 同じ XML が 6 ファイルに散っていて、1 文字違うと「G1 が見えない」という
# 同じ症状になり切り分けに時間がかかっていた（2026-09-07 に切り出した）。
# ⚠️ **変数を使う前に source すること。** set -u があるので順番を逆にすると
# 「G1_MAC_IFACE: unbound variable」で即死する
# shellcheck source=_common.sh
. "$HERE/_common.sh"

# 既定値は _common.sh。上書きは同じ環境変数（G1_MAC_IFACE など）で効く
IFACE="$G1_MAC_IFACE"                 # 有線 NIC。USB アダプタなので挿すまで現れない
VM_IP="$G1_VM_IP"
VM_NIC="$G1_VM_NIC"                   # colima が bridged で足す NIC の名前
PC2_IP="$G1_PC2_IP"
IMAGE="$G1_RVIZ_IMAGE"
NAME="$G1_RVIZ_NAME"
# ワークスペースの根（physical_ai）。コンテナに /work として見せる。
# colima の VM に $HOME が virtiofs で入っているので、ここからバインドできる
REPO_ROOT="$G1_REPO_ROOT"
RVIZ_CFG="${G1_RVIZ_CFG:-$HERE/rviz/g1_live.rviz}"
# 転送先の名前は渡した設定と揃える。以前は常に g1_live.rviz という名前で
# コピーしていたので、G1_RVIZ_CFG=.../g1_nav.rviz を渡しても RViz2 の
# タイトルもログも g1_live のままで、どちらが出ているのか分からなかった。
RVIZ_CFG_NAME="$(basename "$RVIZ_CFG")"
RVIZ_CFG_DST="/home/ubuntu/$RVIZ_CFG_NAME"

# DDS の URI は _common.sh が唯一の定義箇所。
#   live    : ブリッジ側の NIC に載せる。VM には NAT の eth0 もあるので、
#             指定しないと CycloneDDS がそちらを選んで G1 が見えないことがある
#   offline : col0 が無いので、ループバックに閉じないと NIC を選べない
DDS_URI="$(g1_dds_uri_live)"
DDS_URI_OFFLINE="$(g1_dds_uri_offline)"

say() { echo "[rviz] $*"; }
die() { echo "[rviz] $*" >&2; exit 1; }

# offline: 実機に繋がず、記録した bag を再生して RViz2 で見るためのモード。
# 有線・VM のブリッジ・PC2 への疎通をすべて飛ばし、DDS はコンテナ内に閉じる
OFFLINE=0
if [ "${1:-}" = "offline" ]; then
    OFFLINE=1
    shift
fi

if [ "${1:-}" = "stop" ]; then
    docker rm -f "$NAME" >/dev/null 2>&1 && say "コンテナを止めました（VM は動いたまま）"
    exit 0
fi
if [ "${1:-}" = "down" ]; then
    docker rm -f "$NAME" >/dev/null 2>&1
    colima stop
    say "VM ごと止めました"
    exit 0
fi

# ブリッジ（col0 の L2）が切れていたときに自分で張り直すか。
#   auto   : colima が動いていれば検査し、**壊れている時だけ** colima stop する（既定）
#   always : 検査せず無条件に colima stop してから起動する（VM の再起動に 2 分ほどかかる）
#   never  : 何もしない（2026-09-09 までの挙動）
# ⚠️ offline では col0 そのものが要らないので、どの値でも検査も stop もしない。
COLIMA_RESTART="${G1_COLIMA_RESTART:-auto}"
case "$COLIMA_RESTART" in
    auto|always|never) ;;
    *) die "G1_COLIMA_RESTART=$COLIMA_RESTART は不正。auto / always / never のどれかにすること" ;;
esac
# colima stop を**実際に通ったか**。コンテナの ros2 daemon を作り直す条件になる（手順 5.5）
COLIMA_STOPPED=0

# ブリッジを検査して、切れていたら colima stop する。**張り直しは下の手順 3 に任せる。**
#
# ケーブルを抜き差しすると col0 と G1 の内蔵スイッチの結びつきが外れる。
# 紛らわしいのは **Mac → PC2 は通ったまま**（手順 1 が OK になる）で、col0 にも
# IPv4 が付いたまま（手順 4 が「設定済み」になる）で、しかも `ros2 topic list` は
# daemon が握った**古い結果**を返すので「見えている」ように錯覚すること。
# 実データ（`ros2 topic echo --once`）が 1 件も来ないところまで行かないと気づけない。
#
# ⚠️ `colima restart` 単体では直らないことがある（col0 に IPv4 が付かないまま上がる型）。
# このスクリプトは手順 4 で自分で `ip addr add` するので、**stop → こちらの起動処理**を通す。
repair_bridge_if_broken() {
    if [ "$COLIMA_RESTART" = never ]; then
        say "ブリッジの検査はしない（G1_COLIMA_RESTART=never）"
        return 0
    fi
    if ! colima status >/dev/null 2>&1; then
        say "VM は止まっている。検査は要らない（このまま起動する）"
        return 0
    fi
    if [ "$COLIMA_RESTART" = always ]; then
        say "検査せずブリッジを張り直す（G1_COLIMA_RESTART=always）。VM の再起動に 2 分ほどかかる"
        colima stop || die "colima stop に失敗した"
        COLIMA_STOPPED=1
        return 0
    fi

    # auto: 壊れている時だけ止める。
    # 手順 1 で **Mac → PC2 が通ることは確かめてある**ので、ここで届かなければ
    # 実機が落ちているのではなくブリッジが切れている
    local via
    via="$(g1_bridge_via)"
    say "ブリッジを検査する（$via → PC2 $PC2_IP:$G1_PC2_SSH_PORT の TCP）"
    if g1_bridge_ok "$via"; then
        say "ブリッジは生きている（$via 経由で確認）"
        return 0
    fi
    if [ "$via" = none ]; then
        say "ブリッジを測れなかった（コンテナも VM も居ない）。このまま起動する"
        return 0
    fi
    say "ブリッジが切れている（$via → PC2 $PC2_IP:$G1_PC2_SSH_PORT が張れない） → colima stop してから張り直す"
    say "VM の再起動に 2 分ほどかかる（実測 08:02:26 stop → 08:04:49 起動完了）"
    colima stop || die "colima stop に失敗した"
    COLIMA_STOPPED=1
}

DDS_ACTIVE="$DDS_URI"
if [ "$OFFLINE" = 1 ]; then
    DDS_ACTIVE="$DDS_URI_OFFLINE"
    say "offline モード（実機に繋がない。記録の再生用）"
    colima status >/dev/null 2>&1 || die "VM が動いていない。一度 実機ありで起動するか colima start すること"
else
    # --- 1. 有線がつながっているか ------------------------------------------------
    ifconfig "$IFACE" >/dev/null 2>&1 \
        || die "$IFACE が無い。有線アダプタを挿してから実行すること"
    MAC_IP=$(ifconfig "$IFACE" | awk '/inet /{print $2}')
    [ -n "$MAC_IP" ] || die "$IFACE に IPv4 が無い。ケーブルを確認すること"
    say "$IFACE = $MAC_IP"
    ping -c1 -W1000 "$PC2_IP" >/dev/null 2>&1 || die "PC2 ($PC2_IP) に届かない"

    # --- 2. VM の IP が誰かと衝突していないか -------------------------------------
    # G1 の内蔵スイッチに DHCP は無い（.120/.161/.164/.200 はすべて固定）。
    # 静的に振るので、振る前に空いていることを確かめる。
    if ! colima status >/dev/null 2>&1; then
        if ping -c1 -W800 "$VM_IP" >/dev/null 2>&1; then
            die "$VM_IP は誰かが使っている。G1_VM_IP で別の番号を指定すること"
        fi
    fi

    # --- 2.5. ブリッジが生きているか（切れていたら自分で張り直す）------------------
    # ここで colima stop まで行くと、下の手順 3 が「止まっている」を見てブリッジ付きで
    # 起動し直す。**現地で `colima restart` を打つか迷わなくて済むようにする。**
    repair_bridge_if_broken

    # --- 3. VM をブリッジで起動 ---------------------------------------------------
    if colima status >/dev/null 2>&1; then
        say "VM は起動済み"
    else
        say "VM を起動する（初回は sudo パスワードを聞かれる）"
        # 中断すると datadisk のロックが残り、次回 "in use by instance" で起動できなくなる。
        # そのときは LIMA_HOME=~/.colima/_lima limactl disk unlock colima で外す。
        colima start --cpu "${G1_VM_CPU:-4}" --memory "${G1_VM_MEM:-6}" --disk "${G1_VM_DISK:-24}" \
            --vm-type vz --network-mode bridged --network-interface "$IFACE" --network-address \
            || die "VM の起動に失敗した"
    fi

    colima ssh -- ip link show "$VM_NIC" >/dev/null 2>&1 \
        || die "VM に $VM_NIC が無い。bridged になっていない（colima start のログを見ること）"

    # --- 4. ブリッジ NIC に静的 IP を振る -----------------------------------------
    # DHCP が無いので colima 任せでは IP が付かない。VM を作り直すたびに要る。
    if colima ssh -- ip -4 addr show "$VM_NIC" 2>/dev/null | grep -q "$VM_IP"; then
        say "$VM_NIC = $VM_IP （設定済み）"
    else
        colima ssh -- sudo ip addr add "$VM_IP/24" dev "$VM_NIC" \
            || die "$VM_NIC への IP 付与に失敗した"
        say "$VM_NIC = $VM_IP を付与した"
    fi
    ping -c2 -W1000 "$VM_IP" >/dev/null 2>&1 || die "$VM_IP に届かない。ブリッジが通っていない"

fi

# --- 5. コンテナ --------------------------------------------------------------
# --network host で VM の netns を共有する。こうしないとコンテナが docker0 の
# 内側に入り、DDS のマルチキャストが L2 に出ない。
if docker inspect "$NAME" >/dev/null 2>&1; then
    docker start "$NAME" >/dev/null 2>&1
    say "コンテナは起動済み"
else
    say "コンテナを作る（イメージが無ければ pull に数分）"
    # リポジトリを /work に見せる。記録した bag をコンテナで `ros2 bag play` するのに要る。
    # colima の VM には $HOME が virtiofs で入っているので、そこからバインドできる
    docker run -d --name "$NAME" --network host \
        -v "$REPO_ROOT:/work" \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e CYCLONEDDS_URI="$DDS_ACTIVE" \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        --security-opt seccomp=unconfined --shm-size=512m \
        "$IMAGE" >/dev/null || die "コンテナの起動に失敗した"
    sleep 8
fi

# --- 5.5. ros2 daemon を作り直す（ブリッジを張り直したときだけ）----------------
# ros2 daemon は**先に起動したときの DDS 設定で作ったグラフをキャッシュ**する。
# ブリッジが切れている間に起きた daemon はその状態のグラフを握り続け、経路が直った後も
# 古い答えを返す（「2 件しか見えない」→ daemon stop で 139 件、という実測がある）。
# 落としておけば次に ros2 を呼んだときに作り直される。**失敗しても致命ではない。**
if [ "$COLIMA_STOPPED" = 1 ] && g1_container_running "$NAME"; then
    say "ブリッジを張り直したので ros2 daemon を落とす（次の呼び出しで作り直される）"
    g1_exec live "ros2 daemon stop" >/dev/null 2>&1 || say "ros2 daemon stop は失敗した（続行する）"
fi

# tiryoh のイメージは ros-humble-desktop（RViz2 込み）だが rmw_cyclonedds_cpp は
# 入っていない。PC2 は cyclonedds なので揃える（foxy 同梱の 0.7.0 が Unitree の
# discovery で SIGSEGV する経緯があり、0.10 世代で揃えるのが前提）。
if ! docker exec "$NAME" test -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so; then
    say "rmw_cyclonedds_cpp を入れる（数分）"
    docker exec "$NAME" bash -c \
        'apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp' \
        >/dev/null 2>&1 || die "rmw_cyclonedds_cpp の導入に失敗した"
fi

# MOLA-LO（LiDAR-Inertial Odometry）。これも ros-humble-desktop には入っていない。
# 内蔵SLAM の odom は歩行中に roll/pitch が中央値 10.5° 狂う（2026-09-07 実測）。
# その姿勢は PC1 の中で作られていて手が届かないので、こちらで作り直す。
# MOLA-LO は IMU 融合・scan-to-map・map->odom(REP-105) を 1 本でやる。
# 詳細は docs/plan/2026-09-07_2-mola-lo-map-alignment.md
if [ "${G1_SKIP_MOLA:-0}" != "1" ] \
   && ! docker exec "$NAME" test -f /opt/ros/humble/bin/mola-lidar-odometry-cli; then
    say "MOLA-LO を入れる（39 パッケージ・数分）"
    docker exec "$NAME" bash -c \
        'apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
             ros-humble-mola-lidar-odometry ros-humble-mola-bridge-ros2 \
             ros-humble-mola-state-estimation ros-humble-mrpt-map-server ros-humble-mp2p-icp \
             ros-humble-mola-metric-maps' \
        >/dev/null 2>&1 || die "MOLA-LO の導入に失敗した（G1_SKIP_MOLA=1 で飛ばせる）"
fi

# ros-humble-mola-metric-maps は mola-lidar-odometry の依存に入っていないが、
# パイプライン YAML が実行時に libmola_metric_maps.so をプラグインとして読む。
# 無いと「Could not find 'libmola_metric_maps.so'」で最初の 1 スキャンで fatal に
# なり、以降の観測が全部捨てられて 0 キーフレームで終わる（2026-09-07 に踏んだ）。
# 上の一括導入に足したので、既存コンテナのための追いかけだけ別に見る。
if [ "${G1_SKIP_MOLA:-0}" != "1" ] \
   && ! docker exec "$NAME" test -e /opt/ros/humble/lib/aarch64-linux-gnu/libmola_metric_maps.so; then
    say "mola_metric_maps（MOLA のプラグイン）を入れる"
    docker exec "$NAME" bash -c \
        'apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
             ros-humble-mola-metric-maps' \
        >/dev/null 2>&1 || die "mola_metric_maps の導入に失敗した"
fi

# Nav2 / OctoMap 一式。2026-09-06 に手で apt した分で、コンテナの中にしか無かった。
# docker rm や別マシンでの再構築で消えるので、MOLA-LO と同じ形のガードで残す。
# octomap_server は /occupied_cells_vis_array（MarkerArray）を自分で出すので、
# 3D voxel は RViz2 の標準プラグインで描ける（octomap-rviz-plugins は要らない）。
if [ "${G1_SKIP_NAV2:-0}" != "1" ] \
   && ! docker exec "$NAME" test -f /opt/ros/humble/lib/octomap_server/octomap_server_node; then
    say "Nav2 と OctoMap を入れる（数分）"
    docker exec "$NAME" bash -c \
        'apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
             ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-rviz-plugins \
             ros-humble-octomap-server ros-humble-pointcloud-to-laserscan' \
        >/dev/null 2>&1 || die "Nav2/OctoMap の導入に失敗した（G1_SKIP_NAV2=1 で飛ばせる）"
fi

# --- 6. X（:1）が上がるのを待つ -------------------------------------------------
# コンテナを作り直した／`docker start` した直後は X がまだ無く、RViz2 が
# `qt.qpa.xcb: could not connect to display :1` で即死する（2026-09-10 に踏んだ）。
# コンテナは supervisord → vnc_run.sh → vncserver :1 → Xtigervnc :1 の順に上がる。
#
# ⚠️ **ポート 80（noVNC の websockify）は X より先に開く。** 80 の生死を
# 「X の準備完了」に使ってはいけない。/tmp/.X11-unix/X1 の有無も cookie が読めるかとは別。
# ⚠️ **xdpyinfo は必ず `-u ubuntu` ＋ XAUTHORITY で打つ。** root で打つと X が完全に
# 上がっていても "Authorization required" で必ず失敗し、上限まで待って die する。
# 下の RViz2 の起動と**同じ組み合わせ**で打っている（`bash -lc` は XAUTHORITY を落とすので使わない）。
X_WAIT_INTERVAL_S=2       # 見に行く間隔
X_WAIT_TIMEOUT_S=90       # ここを超えたら諦めて落とす
X_WAIT_REPORT_EVERY_S=10  # 進捗を出す間隔（毎回出すとうるさい）

# xdpyinfo（x11-utils）が無いイメージのための追いかけ。他の apt と同じ形のガード
if ! docker exec "$NAME" bash -c 'command -v xdpyinfo' >/dev/null 2>&1; then
    say "x11-utils を入れる（X の起動待ちに使う xdpyinfo）"
    docker exec "$NAME" bash -c \
        'apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq x11-utils' \
        >/dev/null 2>&1 || true
fi
if docker exec "$NAME" bash -c 'command -v xdpyinfo' >/dev/null 2>&1; then
    X_PROBE=xdpyinfo
else
    X_PROBE=socket
    say "xdpyinfo が無い。X ソケットの有無で代用する（cookie までは見ないので判定は甘い）"
fi

x_display_ready() {
    if [ "$X_PROBE" = xdpyinfo ]; then
        docker exec -u ubuntu \
            -e DISPLAY=:1 -e XAUTHORITY=/home/ubuntu/.Xauthority \
            "$NAME" xdpyinfo >/dev/null 2>&1
    else
        docker exec "$NAME" test -S /tmp/.X11-unix/X1 >/dev/null 2>&1
    fi
}

wait_for_x() {
    local waited=0
    while ! x_display_ready; do
        if [ "$waited" -ge "$X_WAIT_TIMEOUT_S" ]; then
            die "X（:1）が $X_WAIT_TIMEOUT_S 秒たっても上がらない（docker logs $NAME を見ること）"
        fi
        sleep "$X_WAIT_INTERVAL_S"
        waited=$((waited + X_WAIT_INTERVAL_S))
        if [ "$((waited % X_WAIT_REPORT_EVERY_S))" = 0 ]; then
            say "X（:1）の起動を待っている… $waited 秒"
        fi
    done
    if [ "$waited" -gt 0 ]; then
        say "X（:1）が上がった（$waited 秒待った・$X_PROBE で確認）"
    else
        say "X（:1）は上がっている（$X_PROBE で確認）"
    fi
    return 0
}
wait_for_x

# --- 7. RViz2 -----------------------------------------------------------------
[ -f "$RVIZ_CFG" ] || die "$RVIZ_CFG が無い"
docker cp "$RVIZ_CFG" "$NAME:$RVIZ_CFG_DST" >/dev/null \
    || die "設定の転送に失敗した"
docker exec "$NAME" chown ubuntu:ubuntu "$RVIZ_CFG_DST"

docker exec "$NAME" pkill -f rviz2 >/dev/null 2>&1 && sleep 2

# ログは ubuntu のホームに置く。/tmp に置くと、一度でも root で起動を試した残骸が
# root 所有で残り、以後 ubuntu ではリダイレクトできずに bash が rviz2 を実行する前に
# 死ぬ。しかも **`docker exec -d` はそれでも exit 0 を返す**ので気づけない
# （2026-09-06 に踏んだ）。古い内容で誤診しないよう、起動前に必ず消す。
LOG=/home/ubuntu/rviz2.log
docker exec -u ubuntu "$NAME" rm -f "$LOG"

# デスクトップは ubuntu が持っている。root で起動すると X の cookie が引けない。
# `bash -lc` はログインシェルが環境を作り直して XAUTHORITY を落とすので使わない。
docker exec -d -u ubuntu \
    -e DISPLAY=:1 -e XAUTHORITY=/home/ubuntu/.Xauthority \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI="$DDS_ACTIVE" \
    "$NAME" bash -c "source /opt/ros/humble/setup.bash; rviz2 -d $RVIZ_CFG_DST >$LOG 2>&1"

sleep 12
if docker exec "$NAME" pgrep -f rviz2 >/dev/null 2>&1; then
    say "RViz2 が起動した"
else
    docker exec "$NAME" cat "$LOG" >&2 2>/dev/null || echo "[rviz] ログが空。起動そのものが失敗している" >&2
    die "RViz2 が起動しなかった（上のログを見ること）"
fi

cat <<EOM

  ブラウザで開く:  http://$VM_IP/
  VNC のパスワード: ubuntu

  Fixed Frame は map。建図（1801）を始めるまで SLAM の点群は流れないので
  何も映らない。生 LiDAR を見たいときは Displays の "LiDAR raw" を有効にし、
  Fixed Frame を livox_frame にする（G1 は /tf を出さないので両者は重ねられない）。

EOM
