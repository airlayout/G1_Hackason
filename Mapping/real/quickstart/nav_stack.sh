#!/usr/bin/env bash
# TF・OctoMap・Nav2 を立ち上げる。データ源は「記録の再生」と「実機」から選ぶ。
#
#   offline  記録した bag を再生する。実機に触らない（既定）
#   live     実機の LiDAR と odom を使う
#
# ## なぜ実機なしで組めるのか
#
# 記録は正規の rosbag2（型も CDR も ROS 標準）なので、`ros2 bag play` で当時のトピックが
# そのまま蘇る。TF は odom から作れる。障害物も生 LiDAR から作れる。
# **実機が要るのは /cmd_vel を足に渡すところだけ**で、そこまでは全部これで詰められる。
#
# ## 前提
#
#   bash quickstart/start_rviz_mac.sh offline     # コンテナを実機なしで起動
#
# ## 使い方
#
#   bash quickstart/nav_stack.sh                    # 記録を再生して起動（実機に触らない）
#   bash quickstart/nav_stack.sh --rate 1           # 等速で再生
#   bash quickstart/nav_stack.sh live               # 実機のデータで起動
#   bash quickstart/nav_stack.sh stop               # 全部止める
#
# ⚠️ live でも**足は動かない**。Nav2 は /cmd_vel を出すだけで、それを足に渡すのは
# PC2 側の loco_driver.py（Navigation/real/）である。動かすにはあちらを --arm で起こす。
set -uo pipefail

NAME="${G1_RVIZ_NAME:-rviz}"
SESSION="${G1_SESSION:-20260906T135940_UiS_room_v3}"
BAG="/work/G1_Hackason/Mapping/real/runs/$SESSION/raw/rosbag2"
RATE="3"
NAV2_PARAMS="/work/G1_Hackason/Navigation/nav2/g1_nav2.yaml"
# 実測値（2026-09-05）。**度**で渡す。--livox-rpy はラジアンなので取り違えないこと
LIVOX_RPY_DEG="178.35 -8.41 -0.72"
LIVOX_XYZ="-0.004 0.016 -0.037"
# OctoMap の maxRange。2026-09-06 の掃引で 2m が誤除去最少だった
OCTO_MAX_RANGE="${G1_OCTO_MAX_RANGE:-2.0}"

MODE="offline"
[ "${1:-}" = "live" ] && { MODE="live"; shift; }

# offline はブリッジ NIC が無いので DDS をループバックに閉じる。
# live は G1 の L2 に載っている col0 に載せる
# （コンテナは --network host なので VM と同じ netns を見る）
if [ "$MODE" = "live" ]; then
    DDS='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="col0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
else
    DDS='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
fi

say() { echo "[stack] $*"; }

# コンテナ内でノードを 1 つ起こす。環境は毎回明示する（bash -lc は XAUTHORITY を落とすので使わない）
spawn() {
    local log="$1"; shift
    docker exec -d -u ubuntu \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 \
        "$NAME" bash -c "source /opt/ros/humble/setup.bash && $* > /home/ubuntu/$log 2>&1"
}

if [ "${1:-}" = "stop" ]; then
    # パターンに自分自身が入らないよう [] で括る（pkill -f は自分のコマンド行にも当たる）
    docker exec "$NAME" bash -c 'pkill -f "ros2 bag pla[y]"; pkill -f "odom_to_t[f]"; pkill -f "octomap_serve[r]"; pkill -f "nav2_[a-z]"; pkill -f "navigation_launc[h]"' >/dev/null 2>&1
    say "止めました（RViz2 は残す）"
    exit 0
fi
if [ "${1:-}" = "--rate" ]; then RATE="$2"; shift 2; fi

docker inspect "$NAME" >/dev/null 2>&1 || { echo "[stack] コンテナ $NAME が無い。先に start_rviz_mac.sh" >&2; exit 1; }

# 再生では /clock を使うので use_sim_time が要る。実機では実時刻なので使わない
SIM_TIME="true"
if [ "$MODE" = "live" ]; then
    SIM_TIME="false"
    say "1/4 実機のデータを使う（再生はしない）"
    docker exec "$NAME" bash -c "source /opt/ros/humble/setup.bash && RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI='"'"'$DDS'"'"' ROS_DOMAIN_ID=0 timeout 8 ros2 topic hz /utlidar/cloud_livox_mid360 2>&1 | tail -1"
else
    docker exec "$NAME" test -d "$BAG" || { echo "[stack] bag が無い: $BAG（/work のバインドを確認）" >&2; exit 1; }
    say "1/4 記録を再生する（$SESSION / ${RATE}倍速 / ループ）"
    spawn bagplay.log "ros2 bag play $BAG --rate $RATE --clock --loop"
    sleep 4
fi

say "2/4 TF を流す（Nav2 の map -> odom -> base_link 形式）"
spawn odomtf.log "python3 /work/G1_Hackason/Mapping/real/quickstart/odom_to_tf.py \
    --nav2-frames --livox-rpy-deg $LIVOX_RPY_DEG --livox-xyz $LIVOX_XYZ"
sleep 4

say "3/4 OctoMap を作る（生 LiDAR / maxRange ${OCTO_MAX_RANGE}m）"
# 地図点群 /unitree/slam_mapping/points は stamp=0 で TF を引けないので使わない
spawn octomap.log "ros2 run octomap_server octomap_server_node --ros-args \
    -p resolution:=0.10 -p frame_id:=map -p base_frame_id:=base_link \
    -p sensor_model.max_range:=$OCTO_MAX_RANGE -p use_sim_time:=$SIM_TIME \
    -r cloud_in:=/utlidar/cloud_livox_mid360"
sleep 6

say "4/4 Nav2 を起動する"
spawn nav2.log "ros2 launch nav2_bringup navigation_launch.py \
    params_file:=$NAV2_PARAMS use_sim_time:=$SIM_TIME"
sleep 20

say "起動したもの:"
docker exec "$NAME" bash -c 'pgrep -af "bag pla[y]|odom_to_t[f]|octomap_serve[r]|controller_serve[r]|planner_serve[r]|bt_navigato[r]" | sed "s/^/  /" | cut -c1-110'
echo
say "ログ: docker exec $NAME tail -f /home/ubuntu/{bagplay,odomtf,octomap,nav2}.log"
say "RViz2: http://localhost/ （VNC パスワード ubuntu）"
