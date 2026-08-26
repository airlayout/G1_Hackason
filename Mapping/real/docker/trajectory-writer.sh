#!/usr/bin/env bash
set -euo pipefail

BACKEND="${1:?backend is required}"
SESSION_ID="${MAPPING_SESSION_ID:?MAPPING_SESSION_ID is required}"
OUTPUT_PATH="/runs/$SESSION_ID/trajectory/trajectory.tum"

if [[ "$BACKEND" == "raw" ]]; then
    ODOM_TOPIC=/g1_mapping/odom
else
    ODOM_TOPIC="${ONBOARD_ODOM_TOPIC:?ONBOARD_ODOM_TOPIC is required}"
fi

exec ros2 run g1_mapping_tools trajectory_writer --ros-args \
    -p "output_path:=$OUTPUT_PATH" \
    -p "odom_topic:=$ODOM_TOPIC"
