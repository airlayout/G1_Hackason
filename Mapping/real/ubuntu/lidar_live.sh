#!/usr/bin/env bash
# 実機 G1 の LiDAR をライブで RViz2 に出す（経路(A) = G1 の DDS を購読するだけ）。
#
#   bash Mapping/real/ubuntu/lidar_live.sh
#
# ⚠️ **未検証（2026-09-10 時点）。** 実機が電源オフのため通しで確かめられていない。
#    まず bash Mapping/real/ubuntu/check_lidar.sh で型が解決できるか見ること。
#
# ## なぜ経路(A) を第一候補にするか
#
# ロボット内部と**競合しない**（読むだけ）。経路(B)（Livox ドライバを直接繋ぐ）は
# LiDAR の占有が競合し、キットの docs/05-lidar.md に「ロボットを再起動し起動直後に
# 実行」とある。デモの手順としては弱い。
#
# ## 経路(A) が駄目だったら
#
# Unitree の DDS は ROS 2 の type hash を載せない。Humble では 31fps で出せた実績が
# あるが Jazzy は未検証で、型が解決できなければここは通らない。そのときは
# Teleop/vendor/g1-starter-kit/setup/install_livox.sh を jazzy で通して経路(B) に倒す
# （上流の build.sh は jazzy 対応済み。OMEN にはまだ ~/ws_livox が無い）。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../Demo" && pwd)/lib.sh"
use_ros

RVIZ_CONFIG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/g1_lidar.rviz"

# ロボットの Unitree DDS に合わせる
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
# 有線を明示する。指定しないと WiFi 側でディスカバリして何も見つからない。
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${DEMO_WIRED_IFACE}\"/></Interfaces></General></Domain></CycloneDDS>"

if ! robot_reachable; then
    _die "G1 が見つかりません（${DEMO_G1_PC2}:22 に届かない）。
     電源とケーブルを確認してください。
     実機なしで見せるなら: bash Demo/02_lidar.sh replay"
fi

PIDS=()
cleanup() { printf '\n'; _info "後片付けをします"; stop_tracked_pids "${PIDS[@]}"; }
trap cleanup EXIT INT TERM

_head "LiDAR ライブ（実演2 / 経路(A)）"
_info "RMW           : ${RMW_IMPLEMENTATION}"
_info "インターフェース : ${DEMO_WIRED_IFACE}"
_info "ROS_DOMAIN_ID : 0（ロボットに合わせる）"

# 生の点群は livox_frame、SLAM の点群は map。橋を架けておく。
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id map --child-frame-id livox_frame \
    > /dev/null 2>&1 &
PIDS+=($!)

_info "RViz2 を開きます"
ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}"
