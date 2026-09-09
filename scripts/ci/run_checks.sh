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
python3 scripts/ci/check_markdown_links.py
