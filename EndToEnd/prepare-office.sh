#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/runtime.sh"
# USDの依存関係の取得にはKitやGPUを起動しない。
usd_bundles=("$ISAAC_SIM_PATH"/extscache/omni.usd.libs-*/pxr)
if [[ ${#usd_bundles[@]} -ne 1 || ! -d "${usd_bundles[0]}" ]]; then
    echo '[NG] Isaac SimのUSD Python bundleを一意に選べません' >&2
    exit 1
fi
usd_bundle="${usd_bundles[0]%/pxr}"
export PYTHONPATH="$usd_bundle:$PYTHONPATH"
export LD_LIBRARY_PATH="$usd_bundle/bin:$LD_LIBRARY_PATH"
exec "$ISAAC_SIM_PATH/python.sh" "$E2E_ROOT/scripts/fetch_office.py" "$@"
