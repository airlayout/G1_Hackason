#!/usr/bin/env bash
# 各機能フォルダのテストを自動で見つけて実行する。
# ローカルでも実行できる: bash scripts/ci/run_all_tests.sh
#
# 規約: テストを追加するときは <フォルダ>/tests/run_tests.sh か <フォルダ>/<サブフォルダ>/tests/run_tests.sh を作る。
# ここが自動で見つけて実行するので、CI設定(.github/workflows/ci.yml)を触る必要はない。
# 1 つのフォルダに run_tests.sh が複数あれば、すべて実行する
# （2026-10-02 まで最初の 1 つしか実行しておらず、Navigation/tests/ や Button_Press/ の 2 つ目以降が実行されていなかった）。
#
# 1 つが失敗しても止まらず、最後まで実行してから結果を一覧で出す。1 つでも失敗していれば、終了コードは 1
# （2026-10-02 まで、途中で失敗すると残りのフォルダが実行されず、何が確かめられていないかが見えなかった）。
#
# テストが無いフォルダも一覧に出す。CIが緑でも「検証されていない」ことが
# 見えるようにするため。緑＝安全ではない。
set -uo pipefail

cd "$(git rev-parse --show-toplevel)"

# 棚上げ中のフォルダは対象外（IsaacSim_Envのtest_*.pyはIsaac Sim本体が要る）
FEATURE_DIRS=(Common SimpleWalk Perception Mapping Navigation Entame Button_Press)

echo "=== テストの所在 ==="
declare -a RUNNERS=()
for dir in "${FEATURE_DIRS[@]}"; do
    mapfile -t found < <(git ls-files "$dir/tests/run_tests.sh" "$dir/*/tests/run_tests.sh")
    if [ ${#found[@]} -gt 0 ]; then
        count=$(git ls-files "$dir/**/test_*.py" "$dir/test_*.py" | wc -l)
        printf '  %-12s test_*.py %s ファイル\n' "$dir" "$count"
        for runner in "${found[@]}"; do
            printf '  %-12s   └ %s\n' "" "$runner"
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

declare -a RESULTS=()
failed=0
for runner in "${RUNNERS[@]}"; do
    echo
    echo "=== 実行: $runner ==="
    start=$(date +%s)
    if bash "$runner"; then
        RESULTS+=("  OK   $runner（$(( $(date +%s) - start )) 秒）")
    else
        code=$?
        RESULTS+=("  NG   $runner（終了コード $code、$(( $(date +%s) - start )) 秒）")
        failed=1
    fi
done

echo
echo "=== 結果 ==="
printf '%s\n' "${RESULTS[@]}"
echo "（スキップしたテストは確かめていない。各テストの出力の「skipped」「スキップ」を見ること）"
exit $failed
