#!/usr/bin/env bash
# Quest から この PC に届くかを、テレオペを起動せずに確認する。
#
#   ./scripts/quest_check.sh
#
# テレオペと同じポート 8012 を使うので、テレオペを止めてから実行すること。

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib.sh"

use_tv
exec python3 "$REPO_DIR/tools/quest_check.py" "$@"
