#!/usr/bin/env bash
# Run the native mapping overlay without inheriting conda/Isaac/another ROS distro.
set -eo pipefail
NATIVE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$NATIVE_DIR/../../.." && pwd)"
NATIVE_ROOT="${MAPPING_NATIVE_ROOT:-$REPO_DIR/_local/mapping}"
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH
export PATH="/usr/bin:/bin:$PATH"
source /opt/ros/jazzy/setup.bash
DEPS="$NATIVE_ROOT/deps"
export AMENT_PREFIX_PATH="$DEPS/opt/ros/jazzy:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$DEPS/opt/ros/jazzy:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$DEPS/opt/ros/jazzy/lib:$DEPS/opt/ros/jazzy/lib/x86_64-linux-gnu:$DEPS/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$DEPS/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
if [[ -f "$NATIVE_ROOT/install/local_setup.bash" ]]; then
    source "$NATIVE_ROOT/install/local_setup.bash"
fi
export MAPPING_NATIVE_ROOT="$NATIVE_ROOT"
export PYTHONUNBUFFERED=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" /></Interfaces></General></Domain></CycloneDDS>}"
exec "$@"
