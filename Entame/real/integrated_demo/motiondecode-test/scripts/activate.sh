# Source this file from bash. Environment changes apply only to this shell.
_motiondecode_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_motiondecode_root/.venv/bin/activate"
export HF_HOME="$_motiondecode_root/.cache/huggingface"
export PIP_CACHE_DIR="$_motiondecode_root/.cache/pip"
export XDG_CACHE_HOME="$_motiondecode_root/.cache"
export MPLCONFIGDIR="$_motiondecode_root/.cache/matplotlib"
export CUDA_CACHE_PATH="$_motiondecode_root/.cache/cuda"
export TMPDIR="$_motiondecode_root/.cache/tmp"
export PYTHONPYCACHEPREFIX="$_motiondecode_root/.cache/pycache"
export CYCLONEDDS_HOME="$_motiondecode_root/external/cyclonedds"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset _motiondecode_root
