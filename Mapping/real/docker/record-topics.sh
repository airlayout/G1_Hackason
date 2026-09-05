#!/usr/bin/env bash
set -euo pipefail

SESSION_ID="${MAPPING_SESSION_ID:?MAPPING_SESSION_ID is required}"
OUTPUT_DIR="/runs/$SESSION_ID/raw/rosbag2"

TOPICS=(
    "${RAW_POINTS_TOPIC:?RAW_POINTS_TOPIC is required}"
    "${RAW_IMU_TOPIC:?RAW_IMU_TOPIC is required}"
    "${ONBOARD_POINTS_TOPIC:?ONBOARD_POINTS_TOPIC is required}"
    "${ONBOARD_ODOM_TOPIC:?ONBOARD_ODOM_TOPIC is required}"
    /g1_mapping/livox
    /g1_mapping/imu
    /g1_mapping/odom
    /g1_mapping/cloud_registered
    /g1_camera/color/image/compressed
    /g1_camera/color/camera_info
    /g1_camera/frame_metadata
    /g1_sim/ground_truth/odom
    /g1_sim/ground_truth/camera_pose
    /slam_info
    /slam_key_info
    /tf
    /tf_static
    /clock
)

mkdir -p "$(dirname "$OUTPUT_DIR")"
exec ros2 bag record --storage sqlite3 --output "$OUTPUT_DIR" "${TOPICS[@]}"
