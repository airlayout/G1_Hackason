#!/usr/bin/env bash
# §4 の記録を PC2 の pixi(Humble) 環境で開始する。Nav2 より先に回すこと。
set -u
OUT="/home/unitree/g1_nav2/runs/$(date +%Y%m%d_%H%M%S)_nav2"
cd /home/unitree/g1_nav2/pc2_humble
rm -f /tmp/record.log
setsid nohup ~/.pixi/bin/pixi run bash -lc "
  source /home/unitree/g1_nav2/g1_ws/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  exec bash /home/unitree/g1_nav2/tools/record_nav2_run.sh --output $OUT
" > /tmp/record.log 2>&1 < /dev/null &
echo "started -> $OUT"
