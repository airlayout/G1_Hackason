#!/usr/bin/env bash
# 実機データで Nav2 の配線を通す(足は繋がない)。コンテナ内で実行される。
set -o pipefail   # ROS の setup.bash は未定義変数を参照するため set -u は使えない
source /opt/ros/humble/setup.bash
cd /work
P=/work/nav2_live_wiring.yaml

spawn() { local log=$1; shift; "$@" > /work/$log 2>&1 & echo $!; }

echo "[1/6] TF配線ノード(内蔵SLAM odom -> TF)"
python3 /work/g1_slam_odom_tf.py > /work/n_tf.log 2>&1 &
sleep 8
grep -E "残差" /work/n_tf.log | tail -1

echo "[2/6] map_server (room_a_map)"
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/work/room_a_map.yaml > /work/n_map.log 2>&1 &
sleep 4
ros2 run nav2_util lifecycle_bringup map_server > /work/n_map_lc.log 2>&1
echo "  状態: $(ros2 lifecycle get /map_server 2>&1)"

echo "[3/6] planner_server / controller_server"
ros2 run nav2_planner planner_server --ros-args --params-file $P > /work/n_planner.log 2>&1 &
ros2 run nav2_controller controller_server --ros-args --params-file $P > /work/n_controller.log 2>&1 &
sleep 6
ros2 run nav2_util lifecycle_bringup planner_server > /work/n_planner_lc.log 2>&1
ros2 run nav2_util lifecycle_bringup controller_server > /work/n_controller_lc.log 2>&1
echo "  planner:    $(ros2 lifecycle get /planner_server 2>&1)"
echo "  controller: $(ros2 lifecycle get /controller_server 2>&1)"

echo "[4/6] behavior_server / velocity_smoother / bt_navigator"
ros2 run nav2_behaviors behavior_server --ros-args --params-file $P > /work/n_behavior.log 2>&1 &
ros2 run nav2_velocity_smoother velocity_smoother --ros-args --params-file $P > /work/n_smoother.log 2>&1 &
sleep 5
ros2 run nav2_util lifecycle_bringup behavior_server > /work/n_behavior_lc.log 2>&1
ros2 run nav2_util lifecycle_bringup velocity_smoother > /work/n_smoother_lc.log 2>&1
ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file $P > /work/n_bt.log 2>&1 &
sleep 6
ros2 run nav2_util lifecycle_bringup bt_navigator > /work/n_bt_lc.log 2>&1
echo "  behavior:   $(ros2 lifecycle get /behavior_server 2>&1)"
echo "  smoother:   $(ros2 lifecycle get /velocity_smoother 2>&1)"
echo "  bt_nav:     $(ros2 lifecycle get /bt_navigator 2>&1)"

echo "[5/6] local costmap に実機のLiDARが入っているか"
timeout 10 ros2 topic hz /local_costmap/costmap 2>&1 | grep -m1 "average rate" || echo "  local_costmap 出ていない"

echo "[6/6] 起動しているノード"
ros2 node list 2>/dev/null | sort | sed 's/^/  /'
