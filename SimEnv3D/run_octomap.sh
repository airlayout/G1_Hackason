#!/usr/bin/env bash
# octomap_server を起動して 3D voxel マップを作る。
#
# 前提: 別ターミナルで Isaac Sim を --lidar3d 付きで起動しておくこと。
#
#   source env.sh
#   "$ISAAC_SIM/python.sh" src/run_g1_twin.py --viz none \
#       --lidar3d --command-source patrol --max-steps 90000
#
# 配信されるトピック:
#   /octomap_full                  3D voxel マップ（octomap_msgs/Octomap）
#   /octomap_point_cloud_centers   voxel 中心の点群（RViz 表示用）
#   /projected_map                 2D 投影（nav_msgs/OccupancyGrid）
#   /occupied_cells_vis_array      voxel の可視化（RViz 表示用）
#
# 保存:
#   ros2 run octomap_server octomap_saver_node -f maps/warehouse_3d.bt
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

mkdir -p "$SCRIPT_DIR/logs" "$SCRIPT_DIR/maps"
LOG="$SCRIPT_DIR/logs/octomap.log"

# map -> odom を恒等変換で流す。
# Isaac Sim は真値の姿勢を持つため自己位置推定は不要で、AMCL の代わりに
# 恒等変換を流せば map と odom が一致する（2D で実績のある方法）。
echo "[INFO] map -> odom の TF を流します"
python3 src/publish_map_odom_tf.py > "$SCRIPT_DIR/logs/map_odom_tf.log" 2>&1 &
TF_PID=$!
# shellcheck disable=SC2064
trap "kill $TF_PID 2>/dev/null || true" EXIT

echo "[INFO] octomap_server を起動します (log: $LOG)"
echo "[INFO] 入力トピック /cloud_in を /points に remap します"

# 入力トピックは /cloud_in で固定なので remap する。
exec ros2 run octomap_server octomap_server_node \
    --ros-args \
    --params-file "$SCRIPT_DIR/config/octomap.yaml" \
    -r /cloud_in:=/points \
    2>&1 | stdbuf -oL -eL tee "$LOG"
