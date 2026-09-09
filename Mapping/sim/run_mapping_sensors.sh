#!/usr/bin/env bash
# Isaac Simから実機と同じLiDAR・IMU・RGBトピックを配信する。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
SIM_DIR="$REPO_DIR/SimEnv3D"

# shellcheck disable=SC1091
source "$SIM_DIR/env.sh"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" /></Interfaces></General></Domain></CycloneDDS>}"

COMMAND_SOURCE="${SIM_COMMAND_SOURCE:-patrol}"
VIZ="${SIM_VIZ:-kit}"

exec "$ISAAC_SIM/python.sh" "$SIM_DIR/src/run_g1_twin.py" \
    --ros \
    --lidar3d \
    --enable_cameras \
    --command-source "$COMMAND_SOURCE" \
    --viz "$VIZ" \
    --kit_args="--/app/extensions/registryEnabled=false" \
    "$@"
