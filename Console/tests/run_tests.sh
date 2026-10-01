#!/usr/bin/env bash
# Console のテスト。実機も ssh も要らない。
set -euo pipefail
cd "$(dirname "$0")"
python3 -m unittest -v test_console
