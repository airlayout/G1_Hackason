#!/usr/bin/env bash
# 記録した LiDAR を再生して RViz2 に出す。**実機は要らない。**
#
#   bash Mapping/real/ubuntu/lidar_replay.sh              # 等速で再生
#   bash Mapping/real/ubuntu/lidar_replay.sh --rate 3     # 3 倍速（591 秒 → 約 3 分）
#   bash Mapping/real/ubuntu/lidar_replay.sh --no-rviz    # RViz2 を開かない
#   bash Mapping/real/ubuntu/lidar_replay.sh <記録名>
#
# ## --loop は使わない
#
# 巻き戻すと時刻が戻り、**TF が全部死ぬ**（TF_OLD_DATA の山になって何も出なくなる）。
# 見せ直したいときは、もう一度このスクリプトを叩くこと。
#
# ## この記録に TF は入っていない
#
# 入っているのは 4 トピックだけで /tf は無い。
#   /unitree/slam_mapping/points  frame_id=map          … SLAM が積んだ地図
#   /unitree/slam_mapping/odom    frame_id=map          … 歩いた軌跡
#   /utlidar/cloud_livox_mid360   frame_id=livox_frame  … 生の 1 スキャン
#   /utlidar/imu_livox_mid360     frame_id=livox_frame
# frame が 2 つに割れているので、map -> livox_frame の静的 TF を出して両方見せる。
# ⚠️ 恒等変換なので**生の点群は原点に貼り付く**（ロボットに追従しない）。
#    見せたい絵は「SLAM が積んだ地図」のほう。生のほうは 1 スキャンの見本。
#
# ## Humble で録った bag は Jazzy でそのままでは再生できない
#
# ros2 bag info は通るのに play だけが落ちる。play は **db3 の中の metadata テーブル**を
# 読むが、Humble の db3 にはそれが無いため（2026-09-10 実測）。
# repair_bag_for_jazzy.py が**追記だけで**直す。このスクリプトが自動で呼ぶ。
#
# ## frame_id="map" は map ではない
#
# 純正 SLAM が名乗る "map" は実体としてはロボット基準（＝odom）。
# 信じて他の地図と重ねるとずれる。ここでは単独で見るだけなので問題にならない。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../Demo" && pwd)/lib.sh"

RUN_NAME="20260906T135940_UiS_room_v3"
RATE=1
OPEN_RVIZ=1
while [ $# -gt 0 ]; do
    case "$1" in
        --rate) RATE="$2"; shift 2 ;;
        --no-rviz) OPEN_RVIZ=0; shift ;;
        --*) _die "知らない引数です: $1" ;;
        *) RUN_NAME="$1"; shift ;;
    esac
done

BAG_DIR="${REPO_DIR}/Mapping/real/runs/${RUN_NAME}/raw/rosbag2"
RVIZ_CONFIG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/g1_lidar.rviz"

if [ ! -f "${BAG_DIR}/metadata.yaml" ]; then
    _die "記録がありません: ${BAG_DIR}
     Mac から持ってきてください: bash Mapping/real/ubuntu/fetch_bag.sh"
fi

use_ros
# 再生は自分のドメインで完結させる。ロボットの domain 0 に流し込まない。
export ROS_DOMAIN_ID="${DEMO_ROS_DOMAIN_ID}"

# Humble 由来の db3 は Jazzy の play が開けない。足りない分を追記して直す（冪等）。
REPAIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/repair_bag_for_jazzy.py"
if ! python3 "${REPAIR}" "${BAG_DIR}" > /dev/null 2>&1; then
    _info "この記録は Humble 時代の形式です。Jazzy で再生できるよう直します（追記のみ）"
    python3 "${REPAIR}" "${BAG_DIR}" --apply || _die "記録を直せませんでした"
fi

PIDS=()
cleanup() {
    printf '\n'
    _info "後片付けをします"
    stop_tracked_pids "${PIDS[@]}"
}
trap cleanup EXIT INT TERM

_head "LiDAR の記録を再生する（実演2 / 実機不要）"
_info "記録  : ${BAG_DIR}"
_info "速さ  : ${RATE} 倍"
grep -E "^  (version|storage_identifier|message_count):" "${BAG_DIR}/metadata.yaml" || true

# frame が map と livox_frame に割れているので橋を架ける
_info "静的 TF を出します: map -> livox_frame（恒等）"
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id map --child-frame-id livox_frame \
    > /dev/null 2>&1 &
PIDS+=($!)

if [ "${OPEN_RVIZ}" = "1" ]; then
    _info "RViz2 を開きます（設定: $(basename "${RVIZ_CONFIG}")）"
    ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" > /dev/null 2>&1 &
    PIDS+=($!)
    sleep 4
fi

cat <<'GUIDE'

--------------------------------------------------------------
 見どころ: 「SLAM が積んだ地図」が歩くにつれて増えていきます。
           赤い矢印が歩いた軌跡です。

 ⚠️ 「生の LiDAR」は原点に貼り付いて見えます。記録に TF が
    入っていないためで、故障ではありません。
--------------------------------------------------------------

GUIDE

_info "再生します（Ctrl-C で止まります）"
# --loop は付けない。巻き戻すと TF が全部死ぬ。
ros2 bag play "${BAG_DIR}" --rate "${RATE}" --clock
_ok "再生が終わりました"
