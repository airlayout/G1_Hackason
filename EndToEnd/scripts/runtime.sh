#!/usr/bin/env bash
# sourceしてIsaacのPythonとプロジェクト専用依存関係を組み合わせる。
set -euo pipefail
E2E_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$E2E_ROOT/.env" ]]; then
    source "$E2E_ROOT/.env"
fi
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-/opt/isaac-sim}"
ISAACLAB_PATH="${ISAACLAB_PATH:-$E2E_ROOT/../_local/IsaacLab}"
export ISAAC_SIM_PATH ISAACLAB_PATH
if [[ ! -x "$ISAAC_SIM_PATH/python.sh" || ! -d "$ISAACLAB_PATH/source/isaaclab" ]]; then
    echo '[NG] configs/runtime.env.exampleを参考に.envでISAAC_SIM_PATHとISAACLAB_PATHを設定してください' >&2
    return 1
fi
E2E_PYTHON_VERSION="$("$ISAAC_SIM_PATH/kit/python/bin/python3" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ "$E2E_PYTHON_VERSION" != 3.12 ]]; then
    echo "[NG] 検証対象はPython 3.12です: $E2E_PYTHON_VERSION" >&2
    return 1
fi
E2E_SITE="$E2E_ROOT/.venv/lib/python$E2E_PYTHON_VERSION/site-packages"
E2E_LAB_SOURCES=""
for source_dir in "$ISAACLAB_PATH"/source/*; do
    [[ -d "$source_dir" ]] && E2E_LAB_SOURCES+="$source_dir:"
done
E2E_TORCH_BUNDLE="$ISAAC_SIM_PATH/extsDeprecated/omni.isaac.ml_archive/pip_prebundle"
if [[ ! -d "$E2E_TORCH_BUNDLE/torch" ]]; then
    echo '[NG] Isaac Sim同梱のPyTorchが必要です。docs/development.mdとconfigs/runtime.lock.jsonを確認してください' >&2
    return 1
fi
E2E_NVIDIA_BUNDLE="$ISAAC_SIM_PATH/exts/isaacsim.pip.nv/pip_prebundle"
export PYTHONPATH="$E2E_ROOT/src:$E2E_ROOT:${E2E_LAB_SOURCES}$E2E_SITE:$E2E_NVIDIA_BUNDLE:$E2E_TORCH_BUNDLE${PYTHONPATH:+:$PYTHONPATH}"
# 6.2ではPyTorchとCUDAの共有ライブラリが別のpip archiveにある。
for library_dir in "$E2E_NVIDIA_BUNDLE"/nvidia/*/lib; do
    [[ -d "$library_dir" ]] && export LD_LIBRARY_PATH="$library_dir:${LD_LIBRARY_PATH:-}"
done
export PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
# CUDAの既定順位はnvidia-smi/Kitと異なることがある。PCI順へ揃える。
export CUDA_DEVICE_ORDER=PCI_BUS_ID
export TMPDIR="$E2E_ROOT/assets/cache/tmp"
mkdir -p "$TMPDIR"

e2e_launch() {
    local entrypoint="$1"
    shift
    if [[ -v CUDA_VISIBLE_DEVICES ]]; then
        echo '[NG] CUDA_VISIBLE_DEVICESを解除し、.envのG1_E2E_GPU_INDEXでGPUを選択してください' >&2
        return 1
    fi
    if [[ ! -d "$E2E_SITE/numpy" ]]; then
        echo '[NG] 先にbash EndToEnd/setup.sh --with-demoを実行してください' >&2
        return 1
    fi
    local selected_gpu="${G1_E2E_GPU_INDEX:-}"
    if [[ -z "$selected_gpu" ]]; then
        selected_gpu="$(nvidia-smi --query-gpu=index,name --format=csv,noheader | awk -F, '/RTX/ {gsub(/ /,"",$1); print $1; exit}')"
    fi
    if [[ -z "$selected_gpu" ]]; then
        echo '[NG] RTX GPUが見つかりません。.envのG1_E2E_GPU_INDEXを確認してください' >&2
        return 1
    fi
    export G1_E2E_SELECTED_GPU="$selected_gpu"
    exec "$ISAAC_SIM_PATH/python.sh" "$E2E_ROOT/$entrypoint" "$@"
}
