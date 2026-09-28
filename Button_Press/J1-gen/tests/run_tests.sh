#!/usr/bin/env bash
# Button_Press/J1-gen のテストを実行する。scripts/ci/run_all_tests.sh が自動で見つけて呼ぶ。
# ローカルでも実行できる: bash Button_Press/J1-gen/tests/run_tests.sh
#
# 依存関係をこのスクリプト自身でインストールしてから実行する（CI側に事前の pip install
# ステップが無いため。Perception/tests/run_tests.sh と同じ方式）。
# 公式モデルが無ければ sim/fetch_models.sh で取得する（ネットワークが必要）。
#
# 使う python は $PYTHON で変えられる（既定は python3）。ローカルでは venv の python を渡す:
#   PYTHON=G1_HuggingFace/venv/bin/python bash Button_Press/J1-gen/tests/run_tests.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$FEATURE_DIR/../.." && pwd)"
PYTHON="${PYTHON:-python3}"

"$PYTHON" -m pip install -q -r "$FEATURE_DIR/requirements.txt"

if [ ! -f "$REPO_ROOT/_local/button_press/models/g1_description/g1_29dof_rev_1_0.xml" ]; then
    bash "$FEATURE_DIR/sim/fetch_models.sh"
fi

export PYTHONPATH="$FEATURE_DIR${PYTHONPATH:+:$PYTHONPATH}"
"$PYTHON" -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v
