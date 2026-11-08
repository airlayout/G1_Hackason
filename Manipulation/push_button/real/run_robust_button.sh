#!/usr/bin/env bash
# check: weight zero; precontact: approach/return; push: press/retreat/lamp check.
# Joint correction starts at the initial pose and continues throughout movement.
# CLI arguments after these defaults can select --button-feedback off and/or
# --button-policy standard (four conditions). G1 setup routes over SSH.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec env PYTHONUNBUFFERED=1 python3 "$HERE/robust_session.py" \
    --button-feedback joint --button-policy collection --planning-budget-s 120 "$@" --task button
