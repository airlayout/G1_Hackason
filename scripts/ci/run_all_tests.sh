#!/usr/bin/env bash
# 各機能フォルダのテストを自動で見つけて実行する。
# ローカルでも実行できる: bash scripts/ci/run_all_tests.sh
#
# 規約: テストを追加するときは <フォルダ>/tests/run_tests.sh を作る。
# ここが自動で見つけて実行するので、CI設定(.github/workflows/ci.yml)を触る必要はない。
#
# テストが無いフォルダも一覧に出す。CIが緑でも「検証されていない」ことが
# 見えるようにするため。緑＝安全ではない。
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

# 棚上げ中のフォルダは対象外（IsaacSim_Envのtest_*.pyはIsaac Sim本体が要る）
FEATURE_DIRS=(Common SimpleWalk Perception Mapping Navigation Entame)

echo "=== テストの所在 ==="
declare -a RUNNERS=()
for dir in "${FEATURE_DIRS[@]}"; do
    runner=$(git ls-files "$dir/*/tests/run_tests.sh" "$dir/tests/run_tests.sh" | head -1)
    if [ -n "$runner" ]; then
        count=$(git ls-files "$dir/**/test_*.py" "$dir/test_*.py" | wc -l)
        printf '  %-12s %s（test_*.py %s ファイル）\n' "$dir" "$runner" "$count"
        RUNNERS+=("$runner")
    else
        printf '  %-12s テストなし（このフォルダは検証されていない）\n' "$dir"
    fi
done

if [ ${#RUNNERS[@]} -eq 0 ]; then
    echo
    echo "実行可能なテストが1つも無い。"
    exit 0
fi

for runner in "${RUNNERS[@]}"; do
    echo
    echo "=== 実行: $runner ==="
    bash "$runner"
done
