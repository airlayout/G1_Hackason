#!/usr/bin/env bash
set -euo pipefail

SESSION_ID="${MAPPING_SESSION_ID:?MAPPING_SESSION_ID is required}"
OUTPUT_PATH="/runs/$SESSION_ID/trajectory/trajectory.tum"

exec ros2 run g1_mapping_tools trajectory_writer --ros-args \
    -p "output_path:=$OUTPUT_PATH" \
    -p "odom_topic:=/g1_mapping/odom" \
    -p "use_sim_time:=${USE_SIM_TIME:-false}"
