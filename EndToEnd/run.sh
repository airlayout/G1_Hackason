#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/runtime.sh"
if [[ -v CUDA_VISIBLE_DEVICES ]]; then
    echo '[NG] CUDA_VISIBLE_DEVICESを解除し、.envのG1_E2E_GPU_INDEXでGPUを選択してください' >&2
    exit 1
fi
if [[ ! -d "$E2E_SITE/numpy" ]]; then
    echo '[NG] 先にbash EndToEnd/setup.sh --with-demoを実行してください' >&2
    exit 1
fi
# このPCではGPU 0が非RTX。RTXを選び、CUDAとKitの両方へ同じindexを渡す。
E2E_GPU_INDEX="${G1_E2E_GPU_INDEX:-}"
if [[ -z "$E2E_GPU_INDEX" ]]; then
    E2E_GPU_INDEX="$(nvidia-smi --query-gpu=index,name --format=csv,noheader | awk -F, '/RTX/ {gsub(/ /,"",$1); print $1; exit}')"
fi
if [[ -z "$E2E_GPU_INDEX" ]]; then
    echo '[NG] RTX GPUが見つかりません。.envのG1_E2E_GPU_INDEXを確認してください' >&2
    exit 1
fi
export G1_E2E_SELECTED_GPU="$E2E_GPU_INDEX"
exec "$ISAAC_SIM_PATH/python.sh" "$E2E_ROOT/sim/play.py" "$@"
