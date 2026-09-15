#!/usr/bin/env bash
# §5 の launch を PC2 の pixi(Humble) 環境で上げる。
#
# 第1引数: map_to_odom。"dx dy yaw[rad]" か `none`(連続localization を併用するとき)。
# 第2引数: lidar_yaw[度]。既定 0。
#   ⚠️ 2026-09-15 に水平化を heading_preserving_leveling に替えたので、yaw は
#   構成上ずれなくなった。それでも RViz で base_link の赤軸(前方)が実機の正面と
#   一致しているかは毎回目視すること。
#
# ⚠️ **CycloneDDS を使う**(2026-09-15 に FastDDS から変更)。D-03 は FastDDS を
# 指定しているが、**PC2 の FastDDS は点群(441KB/フレーム)を受け取れない**。
# net.core.rmem_max が既定の 212992 のままだと落ちる。sudo で広げれば直るが、
# 権限を要らなくするため CycloneDDS を選んだ(同条件で受信できることを実測)。
# A-10e で問題になった「Twist/TwistStamped 同時購読でクラッシュ」は実行時型判定に
# 直してあるので、CycloneDDS でも成立する。
set -u
MAP2ODOM="${1:-0 0 0}"
LIDAR_YAW="${2:-0}"
# 第3引数: operator_timeout_s。heartbeat が何秒途絶したら止めるか(D-31)。
# ⚠️ 2026-09-15 実測: **有線なら 1.0 で足りるが、スマホのテザリングでは足りない**。
# WiFi 省電力を切った上でも heartbeat の age が最大 0.628 秒まで伸び(60秒間で
# 0.5 秒超えが 3 回)、既定の 1.0 では operator_lost の誤作動が起きた
# (実際に 1 回、機体が一歩も動かないまま FAULT になった)。
# 代償: 検知が遅れるぶん、通信断からの停止までの前進距離が伸びる
# (max_vx=0.30 なので +1 秒 ≒ +0.3m)。D-31 の受入目標 0.35m はこの値では満たせない。
OP_TIMEOUT="${3:-1.0}"
MAP=/home/unitree/g1_nav2/g1_ws/install/g1_navigation/share/g1_navigation/maps/room_a_map.yaml
CYCLONE_CFG=/home/unitree/g1_nav2/cyclonedds_eth0.xml
cd /home/unitree/g1_nav2/pc2_humble
rm -f /tmp/nav_launch.log
setsid nohup ~/.pixi/bin/pixi run bash -lc "
  source /home/unitree/g1_nav2/g1_ws/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$CYCLONE_CFG
  exec ros2 launch g1_navigation navigation.launch.py backend:=real map:=$MAP \
       map_to_odom:='$MAP2ODOM' lidar_yaw:=$LIDAR_YAW \
       operator_timeout_s:=$OP_TIMEOUT
" > /tmp/nav_launch.log 2>&1 < /dev/null &
echo "started (map_to_odom=$MAP2ODOM lidar_yaw=$LIDAR_YAW operator_timeout_s=$OP_TIMEOUT rmw=cyclonedds)"
