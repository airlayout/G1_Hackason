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
#
# GitHub の CI には画面（OpenGL）も unitree_sdk2py も無い。インストールやモデルの取得に失敗しても止めず、
# 足りないものを使うテストは「スキップ: <理由>」と表示して飛ばす（tests/_env.py）。最初に、何があって
# 何が無いかを一覧で表示する（スキップしたテストは確かめていない、という意味）。
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$FEATURE_DIR/../.." && pwd)"
PYTHON="${PYTHON:-python3}"

# pip は 1 つでも入れられないと全部を入れずに止まるので、失敗したら 1 つずつ入れ直す
if ! "$PYTHON" -m pip install -q -r "$FEATURE_DIR/requirements.txt"; then
    echo "[tests] ⚠️ requirements.txt をまとめて入れられなかった。1 つずつ入れ直す"
    grep -vE '^\s*(#|$)' "$FEATURE_DIR/requirements.txt" | while read -r pkg; do
        "$PYTHON" -m pip install -q "$pkg" || echo "[tests] ⚠️ $pkg を入れられなかった。これを使うテストはスキップする"
    done
fi

if [ ! -f "$REPO_ROOT/_local/button_press/models/g1_description/g1_29dof_rev_1_0.xml" ]; then
    if ! bash "$FEATURE_DIR/sim/fetch_models.sh"; then
        echo "[tests] ⚠️ 公式モデルを取得できなかった。モデルを使うテストはスキップする"
    fi
fi

export PYTHONPATH="$FEATURE_DIR:$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"
"$PYTHON" "$SCRIPT_DIR/_env.py" || exit 1
"$PYTHON" -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v
