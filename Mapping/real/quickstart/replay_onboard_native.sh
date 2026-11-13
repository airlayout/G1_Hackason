#!/usr/bin/env bash
# Reprocess a copied G1 sensor bag with the existing native FAST-LIO stack.
set -euo pipefail
REAL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SESSION="${1:?usage: replay_onboard_native.sh SESSION_ID}"
[[ "$SESSION" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Invalid session ID' >&2; exit 1; }
[[ -f "$REAL_DIR/runs/$SESSION/raw/rosbag2/metadata.yaml" ]] || { echo 'Recorded bag not found' >&2; exit 1; }
[[ ! -e "$REAL_DIR/runs/$SESSION/map/map_fastlio.pcd" ]] || { echo 'Output already exists' >&2; exit 1; }
export MAPPING_SESSION_ID="$SESSION"
export MAPPING_RUNS_DIR="$REAL_DIR/runs"
export REPLAY_RECORD="replay_$(date -u +%Y%m%dT%H%M%SZ)"
export MAP_OUTPUT_NAME=map_fastlio.pcd
export RAW_POINTS_TOPIC=/utlidar/cloud_livox_mid360
export RAW_IMU_TOPIC=/utlidar/imu_livox_mid360
export ENABLE_FOLLOWER_CROP=false
export ROS_DOMAIN_ID=47
exec "$REAL_DIR/native/exec.sh" bash "$REAL_DIR/docker/replay-occupancy.sh"
