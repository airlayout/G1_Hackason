#!/usr/bin/env bash
# G1 の PC2 用(aarch64・Python 3.12)の配布物を、ラボPC(インターネットにつながるマシン)で集める。PC2 がインターネットに
# つながらないときに使う。集めたあと、中のライブラリが要求する glibc / GLIBCXX が PC2 の版(2.31 / 3.4.28)以下かを確かめる。
#
#   bash Perception/real/fetch_pc2_wheels.sh            # → _local/perception/pc2_wheels/
#
# PC2 へは、できたフォルダを写して `pip install --no-index --find-links <フォルダ> -r requirements_onboard.txt`。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PERCEPTION_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$PERCEPTION_DIR/.." && pwd)"
OUT="${1:-$REPO_ROOT/_local/perception/pc2_wheels}"
PYTHON="${PYTHON:-$REPO_ROOT/G1_HuggingFace/venv/bin/python}"
mkdir -p "$OUT"

"$PYTHON" -m pip download --only-binary=:all: --python-version 3.12 \
    --platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64 --platform manylinux_2_24_aarch64 \
    --platform manylinux_2_26_aarch64 --platform manylinux_2_27_aarch64 --platform manylinux_2_28_aarch64 \
    -r "$PERCEPTION_DIR/requirements_onboard.txt" -d "$OUT"

# 版の確認(展開して .so の要求を見るだけ。実行しない)。.so の無い配布物では grep が 1 を返すので || true
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
bad=0
for w in "$OUT"/*.whl; do
    n="$(basename "$w")"
    mkdir -p "$TMP/$n" && (cd "$TMP/$n" && python3 -m zipfile -e "$w" . >/dev/null)
    g="$(find "$TMP/$n" -name '*.so*' -type f -exec objdump -T {} \; 2>/dev/null | grep -o 'GLIBC_[0-9.]*' | sort -t. -k2,2n -u | tail -1 || true)"
    x="$(find "$TMP/$n" -name '*.so*' -type f -exec objdump -T {} \; 2>/dev/null | grep -o 'GLIBCXX_3\.4\.[0-9]*' | sort -t. -k3,3n -u | tail -1 || true)"
    mark="OK"
    if [ -n "$g" ] && [ "$(printf '%s\n%s\n' "${g#GLIBC_}" 2.31 | sort -V | tail -1)" != "2.31" ]; then mark="❌ glibc が新しすぎる"; bad=1; fi
    if [ -n "$x" ] && [ "${x##*.}" -gt 28 ]; then mark="❌ GLIBCXX が新しすぎる"; bad=1; fi
    echo "[fetch] $n: ${g:-（.so なし）} ${x} → $mark"
done
echo "[fetch] 保存先: $OUT（$(du -sh "$OUT" | cut -f1)）"
[ "$bad" -eq 0 ] && echo "[fetch] すべて PC2 の glibc 2.31 / GLIBCXX 3.4.28 で動く見込み" || { echo "[fetch] ❌ 動かないものがある（上の ❌）"; exit 1; }
