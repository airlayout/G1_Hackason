#!/usr/bin/env bash
# heartbeat の安定性を測る。operator_heartbeat_age_s の最大値が
# operator_timeout_s(既定 1.0) に近ければ、その網では D-31 が誤作動する。
set -u
SEC="${1:-60}"
cd /home/unitree/g1_nav2/pc2_humble
~/.pixi/bin/pixi run bash -lc "
  source /home/unitree/g1_nav2/g1_ws/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///home/unitree/g1_nav2/cyclonedds_eth0.xml
  timeout $SEC ros2 topic echo /g1/bridge_status 2>/dev/null
" | grep -A1 operator_heartbeat_age_s | grep -o "'[0-9.]*'" | tr -d "'" > /tmp/hb_ages.txt
N=$(wc -l < /tmp/hb_ages.txt)
if [ "$N" -eq 0 ]; then echo "サンプルが取れなかった"; exit 1; fi
sort -n /tmp/hb_ages.txt > /tmp/hb_sorted.txt
echo "サンプル ${N} 件 (${SEC}秒)"
echo "  最小 $(head -1 /tmp/hb_sorted.txt) s"
echo "  中央 $(sed -n "$((N/2))p" /tmp/hb_sorted.txt) s"
echo "  最大 $(tail -1 /tmp/hb_sorted.txt) s   ← これが 1.0 に近いと operator_lost が出る"
echo "  0.5s 超え: $(awk '$1>0.5' /tmp/hb_ages.txt | wc -l) 件 / 1.0s 超え: $(awk '$1>1.0' /tmp/hb_ages.txt | wc -l) 件"
