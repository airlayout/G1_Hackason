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
set -uo pipefail

IFACE="${G1_MAC_IFACE:-en8}"          # 有線 NIC。USB アダプタなので挿すまで現れない
VM_IP="${G1_VM_IP:-192.168.123.201}"
VM_NIC="${G1_VM_NIC:-col0}"           # colima が bridged で足す NIC の名前
PC2_IP="${G1_PC2_IP:-192.168.123.164}"
IMAGE="${G1_RVIZ_IMAGE:-tiryoh/ros2-desktop-vnc:humble}"
NAME="${G1_RVIZ_NAME:-rviz}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ワークスペースの根（physical_ai）。コンテナに /work として見せる。
# colima の VM に $HOME が virtiofs で入っているので、ここからバインドできる
REPO_ROOT="$(cd "$HERE/../../../.." && pwd)"
RVIZ_CFG="${G1_RVIZ_CFG:-$HERE/rviz/g1_live.rviz}"

# DDS を必ずブリッジ側の NIC に載せる。VM には NAT の eth0 もあるので、
# 指定しないと CycloneDDS がそちらを選んで G1 が見えないことがある。
DDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$VM_NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
# offline では col0 が無い。ループバックに閉じないと CycloneDDS が NIC を選べない
DDS_URI_OFFLINE="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" priority=\"default\" multicast=\"true\"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>"

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

# tiryoh のイメージは ros-humble-desktop（RViz2 込み）だが rmw_cyclonedds_cpp は
# 入っていない。PC2 は cyclonedds なので揃える（foxy 同梱の 0.7.0 が Unitree の
# discovery で SIGSEGV する経緯があり、0.10 世代で揃えるのが前提）。
if ! docker exec "$NAME" test -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so; then
    say "rmw_cyclonedds_cpp を入れる（数分）"
    docker exec "$NAME" bash -c \
        'apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp' \
        >/dev/null 2>&1 || die "rmw_cyclonedds_cpp の導入に失敗した"
fi

# --- 6. RViz2 -----------------------------------------------------------------
[ -f "$RVIZ_CFG" ] || die "$RVIZ_CFG が無い"
docker cp "$RVIZ_CFG" "$NAME:/home/ubuntu/g1_live.rviz" >/dev/null \
    || die "設定の転送に失敗した"
docker exec "$NAME" chown ubuntu:ubuntu /home/ubuntu/g1_live.rviz

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
    "$NAME" bash -c "source /opt/ros/humble/setup.bash; rviz2 -d /home/ubuntu/g1_live.rviz >$LOG 2>&1"

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
