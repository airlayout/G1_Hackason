#!/usr/bin/env bash
# Experimental real-robot button press: static kinematic/collision checks only.
# This entry point always performs a full button press.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec env PYTHONUNBUFFERED=1 python3 "$HERE/robust_session.py" \
    --readiness-policy collection --button-feedback joint --button-policy collection \
    --button-press-mode straight_trial --planning-budget-s 300 push "$@" \
    --task button --button-validation kinematic
