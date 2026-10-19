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
FEATURE_DIRS=(Common SimpleWalk Perception Mapping Navigation Entame Button_Press)

echo "=== テストの所在 ==="
declare -a RUNNERS=()
for dir in "${FEATURE_DIRS[@]}"; do
    # フォルダの中の run_tests.sh をすべて実行する（以前は最初の 1 つだけで、Button_Press/Yada などが
    # CI で動いていなかった。2026-10-19 に修正）
    mapfile -t found < <(git ls-files "$dir/tests/run_tests.sh" "$dir/*/tests/run_tests.sh")
    if [ ${#found[@]} -gt 0 ]; then
        for runner in "${found[@]}"; do
            sub="$(dirname "$(dirname "$runner")")"
            count=$(git ls-files "$sub/**/test_*.py" "$sub/test_*.py" | sort -u | wc -l)
            printf '  %-12s %s（test_*.py %s ファイル）\n' "$dir" "$runner" "$count"
            RUNNERS+=("$runner")
        done
    else
        printf '  %-12s テストなし（このフォルダは検証されていない）\n' "$dir"
    fi
done

if [ ${#RUNNERS[@]} -eq 0 ]; then
    echo
    echo "実行可能なテストが1つも無い。"
    exit 0
fi

# 1 つが失敗しても残りを実行し、最後に失敗したものをまとめて示す
declare -a FAILED=()
for runner in "${RUNNERS[@]}"; do
    echo
    echo "=== 実行: $runner ==="
    bash "$runner" || FAILED+=("$runner")
done

echo
if [ ${#FAILED[@]} -gt 0 ]; then
    echo "=== 失敗: ${#FAILED[@]} / ${#RUNNERS[@]} ==="
    printf '  %s\n' "${FAILED[@]}"
    exit 1
fi
echo "=== すべて成功: ${#RUNNERS[@]} ==="
