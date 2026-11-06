#!/usr/bin/env bash
# **UpDown-SC（LiDAR の指紋）を Mac に用意する。** 1 回だけでよい。
#
#   bash quickstart/fingerprint/setup.sh
#
# - 置き場: quickstart/fingerprint/.updown-sc（git 管理外）
# - 依存: Homebrew の pcl / yaml-cpp / eigen / cmake（無ければ止まって入れ方を出す）
# - コミットを固定する。上流が変わっても台帳の指紋が変わらないように
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_URL="https://github.com/jiejie567/updown-sc.git"
COMMIT="28e5f0e131fb7f2afd4d8f95844c5856f6f3e141"   # 2026-09-26 に検証した版
DEST="$HERE/.updown-sc"

missing=""
for pkg in pcl yaml-cpp eigen cmake; do
    brew list --versions "$pkg" >/dev/null 2>&1 || missing="$missing $pkg"
done
if [ -n "$missing" ]; then
    echo "[fp] ⛔ 足りない:$missing" >&2
    echo "     → brew install$missing" >&2
    exit 1
fi

if [ ! -d "$DEST/.git" ]; then
    git clone -q "$REPO_URL" "$DEST"
fi
git -C "$DEST" fetch -q origin
git -C "$DEST" checkout -q "$COMMIT"

cmake -Wno-dev -S "$DEST/updown_sc" -B "$DEST/build/updown-sc" -DCMAKE_BUILD_TYPE=Release >/dev/null 2>&1
cmake --build "$DEST/build/updown-sc" -j8 >/dev/null
for tool in scan_context_rebuild scan_context_cross_sequence_evaluator; do
    [ -x "$DEST/build/updown-sc/$tool" ] || { echo "[fp] ⛔ $tool ができていない" >&2; exit 1; }
done
echo "[fp] OK: $DEST/build/updown-sc（commit ${COMMIT:0:8}）" >&2
