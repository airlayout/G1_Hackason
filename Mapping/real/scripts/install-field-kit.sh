#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="${1:-$PWD/G1_Hackason}"

cd "$SCRIPT_DIR"
sha256sum --check SHA256SUMS
docker load --input images.tar.gz
mkdir -p "$TARGET_DIR"
tar -C "$TARGET_DIR" -xzf source.tar.gz

echo "[OK] field kitを展開しました: $TARGET_DIR"
echo "[NEXT] cd $TARGET_DIR/Mapping/real && cp .env.example .env"
