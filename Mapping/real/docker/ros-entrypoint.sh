#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
if [[ -f /opt/common_ws/install/setup.bash ]]; then
    source /opt/common_ws/install/setup.bash
fi
if [[ -f /opt/g1_ws/install/setup.bash ]]; then
    source /opt/g1_ws/install/setup.bash
fi

set -u
exec "$@"
