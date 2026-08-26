#!/usr/bin/env bash
set -euo pipefail

if [[ $# -eq 0 ]]; then
    echo "usage: onboard-entrypoint.sh <probe|start|stop|close> [options]" >&2
    exit 2
fi

exec /usr/local/bin/g1_onboard_lio "${G1_IFACE:?G1_IFACE is required}" "$@"
