#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
export PYTHONPATH="$REAL_DIR/python${PYTHONPATH:+:$PYTHONPATH}"
TEST_RUNTIME="$(python3 -c 'from g1_mapping.config import Settings; print(Settings.load().runtime)')"
if [[ "$TEST_RUNTIME" == native && "${G1_MAPPING_NATIVE_TESTS:-0}" != 1 ]]; then
    exec env G1_MAPPING_NATIVE_TESTS=1 bash "$REAL_DIR/native/exec.sh" bash "$SCRIPT_DIR/run_tests.sh"
fi

python3 -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v

MASK_TEST_BINARY="$(mktemp "${TMPDIR:-/tmp}/g1-follower-mask.XXXXXX")"
trap 'rm -f "$MASK_TEST_BINARY"' EXIT
for test in test_follower_mask test_sensor_time; do
    "${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror \
        -I"$REAL_DIR/ros2_ws/src/g1_sensor_adapter/src" \
        "$SCRIPT_DIR/$test.cpp" -o "$MASK_TEST_BINARY"
    "$MASK_TEST_BINARY"
done

if pkg-config --exists octomap; then
    OCTOMAP_TEST_BINARY="$(mktemp "${TMPDIR:-/tmp}/g1-occupancy.XXXXXX")"
    "${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror \
        -I"$REAL_DIR/ros2_ws/src/g1_mapping_octomap/include" \
        "$SCRIPT_DIR/test_occupancy_core.cpp" $(pkg-config --cflags --libs octomap) -o "$OCTOMAP_TEST_BINARY"
    "$OCTOMAP_TEST_BINARY"
    rm -f "$OCTOMAP_TEST_BINARY"
else
    echo '[SKIP] OctoMap C++ regression tests: liboctomap-dev is not installed'
fi

bash -n "$REAL_DIR/mapctl" "$REAL_DIR"/docker/*.sh "$REAL_DIR"/scripts/*.sh
if [[ "$TEST_RUNTIME" == docker ]]; then
    docker compose --env-file "$REAL_DIR/.env.example" \
        -f "$REAL_DIR/compose.yaml" --profile onboard --profile raw --profile sim config --quiet
fi
bash -n "$REAL_DIR"/native/*.sh
