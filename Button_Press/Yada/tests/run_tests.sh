#!/usr/bin/env bash
# Button_Press/Yada のテストを実行する。
#   bash Button_Press/Yada/tests/run_tests.sh
#   PYTHON=~/miniconda3/envs/lerobot/bin/python bash Button_Press/Yada/tests/run_tests.sh
#
# mujoco / pinocchio / 公式モデル（Button_Press/J1-gen/sim/fetch_models.sh で取得）が無いテストはスキップする。
# Isaac Sim のテストはここでは実行しない（起動に数分かかるため。README.md の「Isaac Sim」を参照）。
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-python3}"

export PYTHONPATH="$FEATURE_DIR${PYTHONPATH:+:$PYTHONPATH}"
"$PYTHON" -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v
