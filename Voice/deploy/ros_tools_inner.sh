#!/usr/bin/env bash
# ROS setup.bash は未定義変数を使うため set -u は付けない。
set -eo pipefail
VOICE_REPO_ROOT="$1"
shift
VOICE_NAV_ROOT="${VOICE_NAV_ROOT:-/home/unitree/g1_nav2}"
source "$VOICE_NAV_ROOT/g1_ws/install/setup.bash"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$VOICE_NAV_ROOT/cyclonedds_eth0.xml"
exec "${VOICE_TOOLS_PYTHON:-python3}" "$VOICE_REPO_ROOT/Voice/tool_server.py" "$@"
