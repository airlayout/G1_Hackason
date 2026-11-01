#!/usr/bin/env bash
# check: no actuation; precontact: approach and return; push: measured press and lamp check.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec env PYTHONUNBUFFERED=1 python3 "$HERE/robust_session.py" "$@" --task button
