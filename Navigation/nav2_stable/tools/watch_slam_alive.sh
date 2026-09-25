#!/usr/bin/env bash
# 内蔵SLAM の odom が生きているかを60秒ごとに記録する。
# 2026-09-15: 1801 を送ってから 12〜17 分で勝手に止まった。何分保つのかを測るため。
# 止めるときは: pkill -f watch_slam_aliv[e]
set -u
LOG=/tmp/slam_watch.log
cd /home/unitree/g1_nav2/pc2_humble
echo "$(date '+%F %T') 監視開始" >> "$LOG"
while true; do
  if ~/.pixi/bin/pixi run bash -lc '
        export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        timeout 8 ros2 topic echo /unitree/slam_mapping/odom --once' >/dev/null 2>&1; then
    echo "$(date '+%F %T') OK" >> "$LOG"
  else
    echo "$(date '+%F %T') ★止まった" >> "$LOG"
  fi
  sleep 60
done
