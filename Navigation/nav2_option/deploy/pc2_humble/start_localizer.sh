#!/usr/bin/env bash
# 手順書 §7 の推奨経路: 連続 localization。launch 側は map_to_odom:=none で上げること。
set -u
cd /home/unitree/g1_nav2/pc2_humble
rm -f /tmp/localizer.log
setsid nohup ~/.pixi/bin/pixi run bash -lc "
  source /home/unitree/g1_nav2/g1_ws/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  exec python3 /home/unitree/g1_nav2/tools/map_localizer.py --initial -1.350 -3.850 2.7751
" > /tmp/localizer.log 2>&1 < /dev/null &
echo "started"
