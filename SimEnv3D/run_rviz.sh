#!/usr/bin/env bash
# RViz を起動して 2D LiDAR (/scan) と 3D LiDAR (/points) を表示する。
#
# SimEnvTest と違い SimEnv3D は地図・Nav2 を持たないため、
# LiDAR の生データと odom を見るための最小構成にしてある。
#
# 使い方:
#   bash run_rviz.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

RVIZ_CONFIG="$SCRIPT_DIR/config/rviz_lidar.rviz"

echo "[INFO] RViz を起動します（設定: $RVIZ_CONFIG）"
echo "[INFO] Isaac Sim が /clock を配信しているため use_sim_time:=true で起動します"
exec ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
