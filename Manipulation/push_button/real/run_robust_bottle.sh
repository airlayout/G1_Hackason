#!/usr/bin/env bash
# precontact/push: initialize -> one live three-scenario search -> immediate motion.
# Waist and left arm are held throughout enable, trajectory and release.
# check: separate non-actuating input/policy/output connectivity check.
# Startup and execution share collection state limits; calibration pose drift is logged.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec env PYTHONUNBUFFERED=1 python3 "$HERE/robust_session.py" --readiness-policy collection "$@"
