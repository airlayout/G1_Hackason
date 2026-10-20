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
    echo '[NG] この起動環境にはIsaac Sim同梱のPyTorchが必要です。READMEの対応環境を確認してください' >&2
    return 1
fi
export PYTHONPATH="$E2E_ROOT/src:$E2E_ROOT:${E2E_LAB_SOURCES}$E2E_SITE:$E2E_TORCH_BUNDLE${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
# CUDAの既定順位はnvidia-smi/Kitと異なることがある。PCI順へ揃える。
export CUDA_DEVICE_ORDER=PCI_BUS_ID
export TMPDIR="$E2E_ROOT/assets/cache/tmp"
mkdir -p "$TMPDIR"
