#!/usr/bin/env bash
# Navigationのテスト。実機・PC2・DDS・numpyのいずれも不要。
# scripts/ci/run_all_tests.sh が自動で見つけて実行する。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVIGATION_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
export PYTHONPATH="$NAVIGATION_DIR${PYTHONPATH:+:$PYTHONPATH}"

python3 -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v
