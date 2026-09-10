#!/usr/bin/env bash
# 実機の LiDAR トピックが見えるか、**型が解決できるか**を確かめる。
#
#   bash Mapping/real/ubuntu/check_lidar.sh
#
# ## --no-daemon を必ず付ける理由
#
# ros2 daemon は**先に起動したときの DDS 設定でグラフをキャッシュする**。
# RMW を切り替えた後に daemon 経由で聞くと、古い設定で見た結果を返してくる。
# 「トピックが見えない」「型が違う」の判断を丸ごと誤らせるので、必ず --no-daemon。
#
# ## 型が解決できるかが本題
#
# Unitree の DDS は ROS 2 の type hash を載せない。Humble では
# /utlidar/cloud_livox_mid360 を RViz2 で 31fps 出せた実績があるが、**Jazzy は未検証**。
# ここで解決できなければ経路(B)（Livox ドライバを直接繋ぐ）に倒す。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../Demo" && pwd)/lib.sh"
use_ros

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 有線を明示する。指定しないと WiFi 側でディスカバリして見つからない。
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${DEMO_WIRED_IFACE}\"/></Interfaces></General></Domain></CycloneDDS>"
# ロボットの Unitree DDS は domain 0。購読するのでこちらも 0 に合わせる。
export ROS_DOMAIN_ID=0

_head "LiDAR トピックの確認"
_info "RMW          : ${RMW_IMPLEMENTATION}"
_info "インターフェース: ${DEMO_WIRED_IFACE}"
_info "ROS_DOMAIN_ID: 0（ロボットに合わせる。Demo の既定 ${DEMO_ROS_DOMAIN_ID} ではない）"

if ! robot_reachable; then
    _warn "G1 が見つかりません（${DEMO_G1_PC2}:22 に届かない）。電源を確認してください。"
    _warn "このまま続けますが、何も見えないはずです。"
fi

_head "1. トピック一覧（--no-daemon）"
ros2 topic list --no-daemon 2>&1 | grep -E "utlidar|livox|slam_mapping" || _warn "LiDAR 系のトピックが 1 つも見えません"

for t in /utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360; do
    _head "2. ${t} の型"
    if ros2 topic info --no-daemon "${t}" 2>&1; then
        :
    else
        _ng "型を解決できません → 経路(B)（Livox ドライバ直結）に倒してください"
    fi
done

_head "3. 実際に 1 件届くか（5 秒）"
if timeout 5 ros2 topic echo --no-daemon --once /utlidar/cloud_livox_mid360 --field header 2>&1; then
    _ok "受信できました。ライブ点群が使えます"
else
    _ng "5 秒で 1 件も届きませんでした"
    printf '     切り分け:\n'
    printf '       - ロボットの起動が完了しているか（起動直後は数十秒かかる）\n'
    printf '       - %s に 192.168.123.x が付いているか\n' "${DEMO_WIRED_IFACE}"
    printf '       - Jazzy が Unitree の型を解決できていないなら経路(B) へ\n'
fi
