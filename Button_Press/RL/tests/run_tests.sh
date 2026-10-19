#!/usr/bin/env bash
# Button_Press/RL のテスト（シミュレーターに依存しない部分: 乱し幅・観測・報酬・評価用の Agent の形）。
#   bash Button_Press/RL/tests/run_tests.sh
#   PYTHON=~/miniconda3/envs/lerobot/bin/python bash Button_Press/RL/tests/run_tests.sh
#
# torch が無い Python では、torch を使うテストはスキップする。学習環境そのもの（Isaac Lab）の確認は
# README.md の「動作の確認」（scripts/train.sh --num_envs 16 --max_iterations 3）で行う。
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-python3}"

export PYTHONPATH="$FEATURE_DIR${PYTHONPATH:+:$PYTHONPATH}"
"$PYTHON" -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v
