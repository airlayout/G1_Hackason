#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
export PYTHONPATH="$REAL_DIR/python${PYTHONPATH:+:$PYTHONPATH}"

python3 -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v

MASK_TEST_BINARY="$(mktemp "${TMPDIR:-/tmp}/g1-follower-mask.XXXXXX")"
trap 'rm -f "$MASK_TEST_BINARY"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror \
    -I"$REAL_DIR/ros2_ws/src/g1_sensor_adapter/src" \
    "$SCRIPT_DIR/test_follower_mask.cpp" -o "$MASK_TEST_BINARY"
"$MASK_TEST_BINARY"

bash -n "$REAL_DIR/mapctl" "$REAL_DIR"/docker/*.sh "$REAL_DIR"/scripts/*.sh
docker compose --env-file "$REAL_DIR/.env.example" \
    -f "$REAL_DIR/compose.yaml" --profile onboard --profile raw config --quiet
