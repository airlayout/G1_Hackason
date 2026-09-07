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
#   G1_USE_MOLA=1 bash quickstart/nav_stack.sh      # map->odom を MOLA-LO に出させる（段 3）
#
# ## G1_USE_MOLA=1 が何を変えるか
#
# 既定では odom_to_tf.py が map->odom を**固定変換**として出す。内蔵SLAM の odom は
# 歩行中に roll/pitch が中央値 10.5° 狂うので（2026-09-07 実測）、固定変換では
# 事前地図とライブ点群が重ならない。
# G1_USE_MOLA=1 にすると odom_to_tf.py から map->odom を外し（--no-map-odom）、
# MOLA-LO が測位のみモードで scan-to-map の結果として map->odom を毎スキャン出す。
# 地図は runs/<SESSION>/mola_aligned/map_full.mm を読む（無ければ止まる）。
# その .mm は **SLAM 地図座標系に合わせて作ってある**ので、map_server の
# nav_map.pgm とそのまま噛み合う（合わせ方は run_mola_lo.sh の MOLA_INITIAL_*）。
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
# 段 3: map->odom を MOLA-LO に出させるか
USE_MOLA="${G1_USE_MOLA:-0}"
MOLA_MAP="/work/G1_Hackason/Mapping/real/runs/$SESSION/mola_aligned/map_full.mm"
MOLA_PIPELINE="/work/G1_Hackason/Mapping/real/quickstart/mola/g1_lidar3d_icp_imu.yaml"
# Nav2 の global_costmap.static_layer が読む /map の出どころ
NAV_MAP="${G1_NAV_MAP:-/work/G1_Hackason/Mapping/real/runs/$SESSION/map/nav_map.yaml}"

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
    docker exec "$NAME" bash -c 'pkill -f "ros2 bag pla[y]"; pkill -f "odom_to_t[f]"; pkill -f "octomap_serve[r]"; pkill -f "nav2_[a-z]"; pkill -f "navigation_launc[h]"; pkill -f "mola_[a-z]"; pkill -f "ros2-lidar-odometr[y]"; pkill -f "map_serve[r]"' >/dev/null 2>&1
    say "止めました（RViz2 は残す）"
    exit 0
fi
if [ "${1:-}" = "--rate" ]; then RATE="$2"; shift 2; fi

docker inspect "$NAME" >/dev/null 2>&1 || { echo "[stack] コンテナ $NAME が無い。先に start_rviz_mac.sh" >&2; exit 1; }

# 再生では /clock を使うので use_sim_time が要る。実機では実時刻なので使わない
SIM_TIME="true"
if [ "$MODE" = "live" ]; then
    SIM_TIME="false"
    say "[1] 実機のデータを使う（再生はしない）"
    docker exec "$NAME" bash -c "source /opt/ros/humble/setup.bash && RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI='"'"'$DDS'"'"' ROS_DOMAIN_ID=0 timeout 8 ros2 topic hz /utlidar/cloud_livox_mid360 2>&1 | tail -1"
else
    docker exec "$NAME" test -d "$BAG" || { echo "[stack] bag が無い: $BAG（/work のバインドを確認）" >&2; exit 1; }
    say "[1] 記録を再生する（$SESSION / ${RATE}倍速 / ループ）"
    spawn bagplay.log "ros2 bag play $BAG --rate $RATE --clock --loop"
    sleep 4
fi

if [ "$USE_MOLA" = "1" ]; then
    say "[2] TF を流す（map -> odom は MOLA-LO に任せるので出さない）"
    spawn odomtf.log "python3 /work/G1_Hackason/Mapping/real/quickstart/odom_to_tf.py \
        --nav2-frames --no-map-odom --livox-rpy-deg $LIVOX_RPY_DEG --livox-xyz $LIVOX_XYZ"
else
    say "[2] TF を流す（map -> odom は固定変換で出す）"
    spawn odomtf.log "python3 /work/G1_Hackason/Mapping/real/quickstart/odom_to_tf.py \
        --nav2-frames --livox-rpy-deg $LIVOX_RPY_DEG --livox-xyz $LIVOX_XYZ"
fi
sleep 4

if [ "$USE_MOLA" = "1" ]; then
    docker exec "$NAME" test -f "$MOLA_MAP" || {
        echo "[stack] MOLA の地図が無い: $MOLA_MAP" >&2
        echo "[stack] 先に docker exec rviz bash /work/.../run_mola_lo.sh <SESSION> で作る" >&2
        exit 1; }
    say "[3] MOLA-LO を測位のみモードで起こす（map -> odom を出させる）"
    # start_mapping_enabled:=False が測位のみ。地図は更新しない
    # publish_localization_following_rep105:=True（既定）で map -> odom を出す
    # min_nearby_poses_occupied:=2 は Livox の非反復スキャン向け
    spawn mola.log "ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        mola_lo_pipeline:=$MOLA_PIPELINE \
        lidar_topic_name:=/utlidar/cloud_livox_mid360 \
        imu_topic_name:=/utlidar/imu_livox_mid360 \
        use_imu_for_lio:=True \
        min_nearby_poses_occupied:=2 \
        start_mapping_enabled:=False \
        mola_initial_map_mm_file:=$MOLA_MAP \
        publish_localization_following_rep105:=True \
        ignore_lidar_pose_from_tf:=false \
        use_mola_gui:=False use_rviz:=False \
        use_sim_time:=$SIM_TIME"
    sleep 10
fi

say "[4] OctoMap を作る（生 LiDAR / maxRange ${OCTO_MAX_RANGE}m）"
# 地図点群 /unitree/slam_mapping/points は stamp=0 で TF を引けないので使わない
spawn octomap.log "ros2 run octomap_server octomap_server_node --ros-args \
    -p resolution:=0.10 -p frame_id:=map -p base_frame_id:=base_link \
    -p sensor_model.max_range:=$OCTO_MAX_RANGE -p use_sim_time:=$SIM_TIME \
    -r cloud_in:=/utlidar/cloud_livox_mid360"
sleep 6

# Nav2 の global_costmap.static_layer は /map を読む（g1_nav2.yaml:140）。
# その /map を出すノードが 2026-09-06 の構成には無く、静的レイヤが空のままだった
# （2026-09-07 に気づいた）。map_server はライフサイクルノードなので
# configure/activate を lifecycle_bringup で明示的に叩く必要がある。
if docker exec "$NAME" test -f "$NAV_MAP"; then
    say "[5] map_server を起こす（$(basename "$NAV_MAP") を /map に流す）"
    spawn mapserver.log "ros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:=$NAV_MAP -p use_sim_time:=$SIM_TIME -p frame_id:=map"
    sleep 4
    spawn mapserver_bringup.log "ros2 run nav2_util lifecycle_bringup map_server"
    sleep 6
else
    say "[5] ⚠️ $NAV_MAP が無いので map_server を飛ばす（Nav2 の静的レイヤは空になる）"
fi

say "[6] Nav2 を起動する"
spawn nav2.log "ros2 launch nav2_bringup navigation_launch.py \
    params_file:=$NAV2_PARAMS use_sim_time:=$SIM_TIME"
sleep 20

say "起動したもの:"
docker exec "$NAME" bash -c 'pgrep -af "bag pla[y]|odom_to_t[f]|octomap_serve[r]|controller_serve[r]|planner_serve[r]|bt_navigato[r]|mola_lidar_odometr[y]|map_serve[r]" | sed "s/^/  /" | cut -c1-110'
echo
say "ログ: docker exec $NAME tail -f /home/ubuntu/{bagplay,odomtf,mola,mapserver,octomap,nav2}.log"
say "RViz2: http://localhost/ （VNC パスワード ubuntu）"
