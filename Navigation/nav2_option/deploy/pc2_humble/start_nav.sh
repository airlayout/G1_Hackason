#!/usr/bin/env bash
# §5 の launch を PC2 の pixi(Humble) 環境で上げる。
# ⚠️ 上体が 19° 傾いた姿勢で起動しているので、自動校正の値は捨て値。配線の確認が目的。
#
# 第1引数: map_to_odom。"dx dy yaw[rad]" か `none`(連続localization を併用するとき)。
set -u
MAP2ODOM="${1:-0 0 0}"
MAP=/home/unitree/g1_nav2/g1_ws/install/g1_navigation/share/g1_navigation/maps/room_a_map.yaml
cd /home/unitree/g1_nav2/pc2_humble
rm -f /tmp/nav_launch.log
setsid nohup ~/.pixi/bin/pixi run bash -lc "
  source /home/unitree/g1_nav2/g1_ws/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  exec ros2 launch g1_navigation navigation.launch.py backend:=real map:=$MAP \
       map_to_odom:='$MAP2ODOM'
" > /tmp/nav_launch.log 2>&1 < /dev/null &
echo "started (map_to_odom=$MAP2ODOM)"
