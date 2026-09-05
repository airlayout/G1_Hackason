#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_DIR="$(cd "$REAL_DIR/../.." && pwd)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PATH="${1:-$REAL_DIR/dist/g1-mapping-field-kit_$TIMESTAMP.tar.gz}"
TEMP_DIR="$(mktemp -d)"
KIT_DIR="$TEMP_DIR/g1-mapping-field-kit"
IMAGES=(
    g1-mapping-common:local
    g1-mapping-onboard:local
    g1-mapping-raw:local
    g1-mapping-visualization:local
)

cleanup() {
    rm -rf -- "$TEMP_DIR"
}
trap cleanup EXIT

for image in "${IMAGES[@]}"; do
    if ! docker image inspect "$image" >/dev/null 2>&1; then
        echo "[ERROR] $image がありません。先に ./mapctl build を実行してください。" >&2
        exit 1
    fi
done

mkdir -p "$KIT_DIR"
docker save "${IMAGES[@]}" | gzip -1 > "$KIT_DIR/images.tar.gz"
tar \
    --exclude='Mapping/real/runs/*' \
    --exclude='Mapping/real/dist' \
    --exclude='Mapping/real/.env' \
    --exclude='__pycache__' \
    -C "$REPO_DIR" \
    -czf "$KIT_DIR/source.tar.gz" \
    Mapping Common/network README.md SETUP.md .gitignore
cp "$SCRIPT_DIR/install-field-kit.sh" "$KIT_DIR/install-field-kit.sh"
chmod +x "$KIT_DIR/install-field-kit.sh"
(
    cd "$KIT_DIR"
    sha256sum images.tar.gz source.tar.gz > SHA256SUMS
)

mkdir -p "$(dirname "$OUTPUT_PATH")"
tar -C "$TEMP_DIR" -czf "$OUTPUT_PATH" g1-mapping-field-kit
echo "[OK] $OUTPUT_PATH"
