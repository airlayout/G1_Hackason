#!/usr/bin/env bash
# モデルのダウンロード・マイク・ロボット接続なしで境界と判定を確認する。
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON="${PYTHON:-python3}"
"$PYTHON" -m pip install -q 'numpy>=1.26,<3' 'scipy>=1.13,<2' 'soundfile>=0.13,<1' 'jsonschema>=4.18,<5'
cd "$TASK_ROOT"
"$PYTHON" -m unittest discover -s Voice/tests -p 'test_*.py' -v
