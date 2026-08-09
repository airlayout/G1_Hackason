#!/usr/bin/env bash
# RViz を起動して地図・ロボット・経路を表示する。
#
# run_slam.sh（地図作成中の様子を見る）や run_nav2.sh（自律走行させる）と
# 併せて別ターミナルで実行する。
#
# 使い方:
#   bash run_rviz.sh
#
# Nav2 使用時の操作:
#   「2D Pose Estimate」 … G1 の現在位置を教える（最初に 1 回）
#   「2D Goal Pose」     … 目標地点を指定する（G1 が自律的に歩く）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

RVIZ_CONFIG="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"

if [[ ! -f "$RVIZ_CONFIG" ]]; then
    echo "[WARN] Nav2 の RViz 設定が見つかりません。既定の設定で起動します。"
    exec ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
fi

echo "[INFO] RViz を起動します（設定: $RVIZ_CONFIG）"
echo "[INFO] Isaac Sim が /clock を配信しているため use_sim_time:=true で起動します"
exec ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
