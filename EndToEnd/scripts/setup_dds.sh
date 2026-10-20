#!/usr/bin/env bash
# SDKのIDL/CRCとCyclone DDS本体をプロジェクト内へ固定して取得する。
set -euo pipefail
E2E_DDS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
E2E_DDS_CACHE="$E2E_DDS_ROOT/assets/cache/dds"
mkdir -p "$E2E_DDS_CACHE"
mapfile -t E2E_DDS_SOURCES < <("$E2E_DDS_ROOT/.venv/bin/python" - "$E2E_DDS_ROOT/configs/dds.lock.json" <<'LOCK'
import json, sys
config = json.load(open(sys.argv[1]))
for key in ("sdk_url", "sdk_commit", "cyclonedds_url", "cyclonedds_commit"):
    print(config[key])
LOCK
)
checkout_source() {
    local destination="$E2E_DDS_CACHE/$1" repository="$2" revision="$3"
    if [[ ! -d "$destination/.git" ]]; then
        git clone --filter=blob:none "$repository" "$destination"
    fi
    if [[ "$(git -C "$destination" rev-parse HEAD)" != "$revision" ]]; then
        git -C "$destination" fetch origin "$revision"
        git -C "$destination" checkout --detach "$revision"
    fi
}
checkout_source sdk "${E2E_DDS_SOURCES[0]}" "${E2E_DDS_SOURCES[1]}"
checkout_source cyclonedds "${E2E_DDS_SOURCES[2]}" "${E2E_DDS_SOURCES[3]}"
E2E_DDS_BUILT_REVISION=""
if [[ -f "$E2E_DDS_CACHE/install/.source-revision" ]]; then
    E2E_DDS_BUILT_REVISION="$(cat "$E2E_DDS_CACHE/install/.source-revision")"
fi
if [[ ! -x "$E2E_DDS_CACHE/install/bin/idlc" || "$E2E_DDS_BUILT_REVISION" != "${E2E_DDS_SOURCES[3]}" ]]; then
    cmake -S "$E2E_DDS_CACHE/cyclonedds" -B "$E2E_DDS_CACHE/build" \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$E2E_DDS_CACHE/install" \
        -DBUILD_DDSPERF=OFF -DBUILD_IDLC=ON -DENABLE_SSL=NO
    cmake --build "$E2E_DDS_CACHE/build" --parallel 4
    cmake --install "$E2E_DDS_CACHE/build"
    printf '%s\n' "${E2E_DDS_SOURCES[3]}" > "$E2E_DDS_CACHE/install/.source-revision"
fi
