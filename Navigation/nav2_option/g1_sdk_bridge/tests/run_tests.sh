#!/usr/bin/env bash
# g1_sdk_bridge の全テストを実行する。
# 各テストファイルをスクリプトとして直接実行する(colcon workspace化していないため、
# `python3 -m unittest discover` ではなくこの方式で `_pathfix` を効かせている)。
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

status=0
for f in test_*.py; do
    echo "=== ${f} ==="
    if ! python3 -W error::ResourceWarning "${f}" -v; then
        status=1
    fi
    echo
done

exit "${status}"
