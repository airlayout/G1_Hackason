#!/usr/bin/env bash
# Perception のテストを実行する。scripts/ci/run_all_tests.sh が自動で見つけて呼ぶ。
# ローカルでも実行できる: bash Perception/tests/run_tests.sh
#
# 依存関係(ultralytics/opencv/pyzmq等)をこのスクリプト自身でインストールしてから
# テストを実行する(CI側に事前のpip installステップが無いため)。YOLOの重みは
# ultralyticsが初回実行時に自動ダウンロードする。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PERCEPTION_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

pip install -q -r "$PERCEPTION_DIR/requirements.txt"

export PYTHONPATH="$PERCEPTION_DIR${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v
