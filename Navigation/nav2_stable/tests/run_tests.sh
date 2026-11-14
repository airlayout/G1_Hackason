#!/usr/bin/env bash
# nav2_stable のテスト。ROS・実機・PC2 のいずれも不要（ROS を import しない部分だけを見る）。
# scripts/ci/run_all_tests.sh が自動で見つけて実行する。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$SCRIPT_DIR/../tools${PYTHONPATH:+:$PYTHONPATH}"

python3 -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v
