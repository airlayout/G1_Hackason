#!/usr/bin/env bash
# check: weight zero; precontact: approach/return; push: press/retreat/lamp check.
# Align once before a straight press; bounded joint torque correction remains.
# CLI arguments can select --button-press-mode feedback for the earlier mode.
# G1 setup routes over SSH.
# Readiness uses the shared bottle collection limits; target movement is logged.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec env PYTHONUNBUFFERED=1 python3 "$HERE/robust_session.py" \
    --readiness-policy collection --button-feedback joint --button-policy collection \
    --button-press-mode straight_trial \
    --planning-budget-s 300 "$@" --task button
