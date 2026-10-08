#!/usr/bin/env bash
# git-lfs を sudo なしで ~/.local/bin に導入する（Linux x86_64 用）。
# 取得物は専用の空ディレクトリで展開し、その中のコードは実行しない（git-lfs 本体だけ install する）。
set -euo pipefail
BIN_DIR="${BIN_DIR:-$HOME/.local/bin}"
mkdir -p "$BIN_DIR"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/lfs_dl.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT

TAG="$(curl -fsSL https://api.github.com/repos/git-lfs/git-lfs/releases/latest | python3 -c 'import sys,json; print(json.load(sys.stdin)["tag_name"])')"
echo "[lfs] tag=$TAG"
curl -fsSL -o "$WORK/lfs.tgz" "https://github.com/git-lfs/git-lfs/releases/download/${TAG}/git-lfs-linux-amd64-${TAG}.tar.gz"
tar -xzf "$WORK/lfs.tgz" -C "$WORK"
install -m 755 "$WORK/git-lfs-${TAG#v}/git-lfs" "$BIN_DIR/git-lfs"
"$BIN_DIR/git-lfs" version
echo "[lfs] INSTALL DONE ($BIN_DIR/git-lfs)"
