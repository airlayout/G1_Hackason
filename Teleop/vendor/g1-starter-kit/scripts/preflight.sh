#!/usr/bin/env bash
# ロボットとの疎通と機体構成（23DoF / 29DoF）を確認する。ロボットは動かない。
#
#   ./scripts/preflight.sh
#   ./scripts/preflight.sh --skip-scan     # サブネットのスキャンを省く（速い）

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib.sh"

load_config
# 読み取り専用なので無線でも通す（不安定なら警告が出る）
require_robot_link
use_tv
exec python3 "$REPO_DIR/tools/preflight.py" --iface "$G1_WIRED_IFACE" "$@"
