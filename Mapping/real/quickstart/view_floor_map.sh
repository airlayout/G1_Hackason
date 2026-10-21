#!/usr/bin/env bash
# Display a postprocess_floor_pcd.py output in native RViz (no robot or Docker).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${G1_FLOOR_VIEW_NATIVE:-0}" != 1 ]]; then
    exec env G1_FLOOR_VIEW_NATIVE=1 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-46}" \
        bash "$SCRIPT_DIR/../native/exec.sh" bash "$SCRIPT_DIR/view_floor_map.sh" "$@"
fi
[[ $# == 1 || $# == 2 ]] || { echo "Usage: $0 <postprocessed-directory> [ROS-topic]" >&2; exit 2; }
FLOOR_VIEW_DIR="$(realpath "$1")/height_lt_2m"
FLOOR_VIEW_TOPIC="${2:-/g1_mapping/floor_height_lt2}"
FLOOR_VIEW_TAG="$(basename "$(realpath "$1")" | tr -c '[:alnum:]_' '_')"
[[ -f "$FLOOR_VIEW_DIR/map_floor_height_lt_2m.pcd" && -f "$FLOOR_VIEW_DIR/height_view.rviz" ]] || {
    echo "[ERROR] floor-aligned crop/config not found: $FLOOR_VIEW_DIR" >&2; exit 1;
}
PCL_PREFIX="$(ros2 pkg prefix pcl_ros)"
"$PCL_PREFIX/lib/pcl_ros/pcd_to_pointcloud" --ros-args -r "__node:=floor_height_pcd_$FLOOR_VIEW_TAG" \
    -p "file_name:=$FLOOR_VIEW_DIR/map_floor_height_lt_2m.pcd" \
    -p tf_frame:=floor_map -p publishing_period_ms:=2000 \
    -r "/cloud_pcd:=$FLOOR_VIEW_TOPIC" &
FLOOR_PUBLISHER_PID=$!
trap 'kill -INT "$FLOOR_PUBLISHER_PID" 2>/dev/null || true; wait "$FLOOR_PUBLISHER_PID" 2>/dev/null || true' EXIT
rviz2 -d "$FLOOR_VIEW_DIR/height_view.rviz" -f floor_map --ros-args \
    -r "__node:=floor_height_rviz_$FLOOR_VIEW_TAG" \
    -r "/g1_mapping/floor_height_lt2:=$FLOOR_VIEW_TOPIC"
