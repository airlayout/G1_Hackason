#!/usr/bin/env bash
# 実機側のSDKクライアントでも使う、Isaacに依存しないプロジェクト設定。
set -euo pipefail
E2E_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$E2E_ROOT/.env" ]]; then
    source "$E2E_ROOT/.env"
fi
E2E_PROJECT_PATH="$E2E_ROOT/src:$E2E_ROOT"
export CYCLONEDDS_HOME="$E2E_ROOT/assets/cache/dds/install"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
