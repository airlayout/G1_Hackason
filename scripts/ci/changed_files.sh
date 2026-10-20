#!/usr/bin/env bash
# PR で変わったファイルの一覧を、GitHub Actions の出力（files=...）の形で表示する。
#   bash scripts/ci/changed_files.sh <base の commit>
# base が空（PR 以外）なら空を出す（= すべてを検査する）。
# 複数行の値は、GitHub の決まりで「files<<区切り」の形にする。
set -euo pipefail

BASE="${1:-}"
if [ -z "$BASE" ]; then
    echo "files="
    exit 0
fi
# base と PR の分岐点から HEAD までの変更（merge-base。base に先に入った変更は含めない）
files="$(git diff --name-only "$(git merge-base "$BASE" HEAD)" HEAD)"
echo "[changed] PR で変わったファイル: $(printf '%s\n' "$files" | grep -c . || true) 件" >&2
{
    echo "files<<CI_CHANGED_FILES_EOF"
    echo "$files"
    echo "CI_CHANGED_FILES_EOF"
}
