#!/usr/bin/env bash
# git-lfs を sudo なしで ~/.local/bin に導入する（Linux x86_64 用）。
# 版と SHA-256 を固定してあり、ダウンロードが一致しなければ入れずに止まる。
# 取得物は専用の空ディレクトリで展開し、その中のコードは実行しない（git-lfs 本体だけ install する）。
#
# 別の版を入れるとき: GIT_LFS_TAG=vX.Y.Z GIT_LFS_SHA256=<tar.gz の SHA-256> bash install_git_lfs.sh
#   SHA-256 はリリースの sha256sums.asc にある（git-lfs-linux-amd64-<tag>.tar.gz の行）。
set -euo pipefail

TAG="${GIT_LFS_TAG:-v3.7.0}"
# v3.7.0 の git-lfs-linux-amd64-v3.7.0.tar.gz（リリースの sha256sums.asc で確認した値）
SHA256="${GIT_LFS_SHA256:-e7ebba491af8a54e560be3a00666fa97e4cf2bbbb223178a0934b8ef74cf9bed}"
if [ "$TAG" != "v3.7.0" ] && [ -z "${GIT_LFS_SHA256:-}" ]; then
  echo "[error] GIT_LFS_TAG を変えるときは GIT_LFS_SHA256 も指定してください。" >&2
  exit 1
fi

if [ "$(uname -s)-$(uname -m)" != "Linux-x86_64" ]; then
  echo "[error] このスクリプトは Linux x86_64 用です（この PC: $(uname -s)-$(uname -m)）。" >&2
  exit 1
fi

BIN_DIR="${BIN_DIR:-$HOME/.local/bin}"
mkdir -p "$BIN_DIR"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/lfs_dl.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT

echo "[lfs] tag=$TAG"
curl -fsSL -o "$WORK/lfs.tgz" "https://github.com/git-lfs/git-lfs/releases/download/${TAG}/git-lfs-linux-amd64-${TAG}.tar.gz"
echo "$SHA256  $WORK/lfs.tgz" | sha256sum -c - || { echo "[error] SHA-256 が一致しません。入れずに止めます。" >&2; exit 1; }
tar -xzf "$WORK/lfs.tgz" -C "$WORK"
install -m 755 "$WORK/git-lfs-${TAG#v}/git-lfs" "$BIN_DIR/git-lfs"
"$BIN_DIR/git-lfs" version
echo "[lfs] INSTALL DONE ($BIN_DIR/git-lfs)"
