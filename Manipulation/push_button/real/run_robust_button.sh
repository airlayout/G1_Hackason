#!/usr/bin/env bash
# check: no actuation; precontact: approach and return; push: measured press and lamp check.
# Defaults: 15mm stroke, 30s/50-candidate search, four conditions, first valid path.
# Conditions: nominal, 3mm farther, panel-horizontal left 3mm, panel-horizontal right 3mm.
# precontact/push share bottle monitoring and synchronous publication; CLI/JSON can override defaults.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec env PYTHONUNBUFFERED=1 python3 "$HERE/robust_session.py" "$@" --task button
