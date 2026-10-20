#!/usr/bin/env bash
# 全フォルダ共通の検査。テストが無いフォルダにも効く。
# ローカルでも実行できる: bash scripts/ci/run_checks.sh
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

echo "=== Python構文チェック ==="
# git管理下の .py だけを対象にする（CIがチェックアウトするものと一致させるため）
mapfile -t PY_FILES < <(git ls-files '*.py')
if [ ${#PY_FILES[@]} -eq 0 ]; then
    echo "[syntax] 対象の .py がない"
else
    python3 -m py_compile "${PY_FILES[@]}"
    echo "[syntax] OK: ${#PY_FILES[@]} ファイルすべて構文エラーなし"
fi

echo
echo "=== Markdownリンク検査 ==="
# PR では、変えた Markdown のリンクだけを検査する（CI_CHANGED_FILES。run_all_tests.sh と同じ考え。
# 無ければすべて）。ほかの班の文書のリンク切れで自分の PR が赤にならないように
if [ -n "${CI_CHANGED_FILES:-}" ]; then
    mapfile -t MD_CHANGED < <(printf '%s\n' "$CI_CHANGED_FILES" | grep -E '\.md$' || true)
    echo "[links] PR で変えた Markdown だけを検査する（${#MD_CHANGED[@]} ファイル）"
    python3 scripts/ci/check_markdown_links.py --only "${MD_CHANGED[@]+"${MD_CHANGED[@]}"}"
else
    python3 scripts/ci/check_markdown_links.py
fi
