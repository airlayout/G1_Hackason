#!/usr/bin/env bash
# §5 の launch を PC2 の pixi(Humble) 環境で上げる。
#
# 第1引数: map_to_odom。"dx dy yaw[rad]" か `none`(連続localization を併用するとき)。
# 第2引数: lidar_yaw[度]。既定 180。
#   ⚠️ 2026-09-15: この値は**定数では決められない**ことが分かった。自動校正の
#   leveling は逆さ取付だと約180°の回転になり、その軸が傾きの方位で決まるため、
#   姿勢によって X が反転したりしなかったりする。RViz で base_link の赤軸(前方)が
#   実機の正面と一致しているかを必ず目視すること。
set -u
MAP2ODOM="${1:-0 0 0}"
LIDAR_YAW="${2:-180}"
MAP=/home/unitree/g1_nav2/g1_ws/install/g1_navigation/share/g1_navigation/maps/room_a_map.yaml
cd /home/unitree/g1_nav2/pc2_humble
rm -f /tmp/nav_launch.log
setsid nohup ~/.pixi/bin/pixi run bash -lc "
  source /home/unitree/g1_nav2/g1_ws/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  exec ros2 launch g1_navigation navigation.launch.py backend:=real map:=$MAP \
       map_to_odom:='$MAP2ODOM' lidar_yaw:=$LIDAR_YAW
" > /tmp/nav_launch.log 2>&1 < /dev/null &
echo "started (map_to_odom=$MAP2ODOM lidar_yaw=$LIDAR_YAW)"
