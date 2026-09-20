#!/usr/bin/env bash
# UI のテスト。scripts/ci/run_all_tests.sh が自動で見つけて呼ぶ。
# ローカルでも実行できる: bash G1_HuggingFace/UI/tests/run_tests.sh
#
# ⚠️ **YOLO/torch は要らない。** 航法ソースの契約と地図読み込みだけを見るので、
#    軽い依存（PyYAML / OpenCV / numpy）で足りる。映像側は Perception のテストが見る。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# venv があればそれを使う。無ければ system python に最低限だけ入れる
VENV="$SCRIPT_DIR/../../venv/bin/python"
if [[ -x "$VENV" ]]; then
  PY="$VENV"
else
  PY=python3
  pip install -q PyYAML "opencv-python-headless>=4.9" "numpy>=1.26"
fi

"$PY" -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v
