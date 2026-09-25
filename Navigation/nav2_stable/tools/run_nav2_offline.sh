#!/usr/bin/env bash
# 記録済み rosbag で Nav2 の配線を検証する（実機不要）。
# 2026-09-09 の実機試験はフレームが上下逆のまま走らせたので、その修正後のやり直し。
#
# ⚠️ set -u は使わない（ROS の setup.bash が未定義変数を参照して落ちる）
set -o pipefail
source /opt/ros/humble/setup.bash
cd /work
P=/work/nav2_live_wiring.yaml

echo "[1/6] rosbag を再生（必要な3トピックのみ。bag 内の /tf は使わず自分で出す）"
ros2 bag play /bag --clock --loop --rate 1.0 \
  --topics /utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360 /unitree/slam_mapping/odom \
  > /work/o_bag.log 2>&1 &
sleep 6
echo "  /clock: $(timeout 6 ros2 topic hz /clock 2>&1 | grep -m1 'average rate' || echo '出ていない')"

echo "[2/6] TF配線ノード（内蔵SLAM odom -> TF。use_sim_time=true）"
python3 /work/g1_slam_odom_tf.py --ros-args -p use_sim_time:=true > /work/o_tf.log 2>&1 &
sleep 10
grep -E "自動校正した|静的TF" /work/o_tf.log | head -2

echo "[3/6] map_server（room_a_map）"
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/work/room_a_map.yaml -p use_sim_time:=true > /work/o_map.log 2>&1 &
sleep 4
ros2 run nav2_util lifecycle_bringup map_server > /dev/null 2>&1
echo "  map_server: $(ros2 lifecycle get /map_server 2>&1)"

echo "[4/6] planner / controller"
ros2 run nav2_planner planner_server --ros-args --params-file $P -p use_sim_time:=true > /work/o_planner.log 2>&1 &
ros2 run nav2_controller controller_server --ros-args --params-file $P -p use_sim_time:=true > /work/o_controller.log 2>&1 &
sleep 8
ros2 run nav2_util lifecycle_bringup planner_server > /dev/null 2>&1
ros2 run nav2_util lifecycle_bringup controller_server > /dev/null 2>&1
echo "  planner:    $(ros2 lifecycle get /planner_server 2>&1)"
echo "  controller: $(ros2 lifecycle get /controller_server 2>&1)"

echo "[5/6] フレーム検証（床が base_link の z≈0 に来るか）"
python3 /work/verify_frame.py 2>&1 | grep -E "並進|z 範囲|z=|判定" | head -8

echo "[6/6] local costmap（実機LiDAR由来）"
timeout 12 ros2 topic hz /local_costmap/costmap 2>&1 | grep -m1 "average rate" || echo "  出ていない"
echo "DONE"
