#!/usr/bin/env bash
# G1 の既存 pixi / ROS 環境でツールサーバーを起動する。sudo は使わない。
set -eo pipefail
VOICE_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VOICE_NAV_ROOT="${VOICE_NAV_ROOT:-/home/unitree/g1_nav2}"
cd "$VOICE_NAV_ROOT/pc2_humble"
exec "$HOME/.pixi/bin/pixi" run bash "$VOICE_REPO_ROOT/Voice/deploy/ros_tools_inner.sh" "$VOICE_REPO_ROOT" "$@"
