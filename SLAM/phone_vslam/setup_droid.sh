#!/usr/bin/env bash
# OMEN に DROID-SLAM（ステレオ用）の環境を作る（1 度だけ。やり直しても壊れない）
#
#   使い方: bash setup_droid.sh [clone|env|build|weights|all]
#   置き場所: ~/work_inoue/phone_slam/DROID-SLAM・conda の環境 droid
#
# MASt3R-SLAM の環境（mast3r）を壊さないよう別の環境にする。作りは setup_omen.sh と同じ:
# torch 2.7.0+cu128（RTX 5060 Ti の sm_120）・nvcc は conda-forge・CPATH を targets に通す。
# 表示は phone_vslam の Rerun（0.22.1）を使うので numpy 1.26 に揃える。open3d（上流の窓）は入れない。
set -euo pipefail

WORK="${WORK:-$HOME/work_inoue/phone_slam}"
ENV_NAME="${ENV_NAME:-droid}"
CONDA="$HOME/miniforge3/bin/mamba"
ENV_DIR="$HOME/miniforge3/envs/$ENV_NAME"
PY="$ENV_DIR/bin/python"
REPO="$WORK/DROID-SLAM"
UPSTREAM_COMMIT="${UPSTREAM_COMMIT:-}"   # 空なら main の先頭（取った後に記録する）
WEIGHTS_GDRIVE_ID="1PpqVt1H4maBa_GbPJp4NwxRsd9jk-elh"   # 上流の tools/download_model.sh と同じ

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

step_clone() {
  if [[ -d "$REPO/.git" ]]; then log "clone: ある（$(git -C "$REPO" rev-parse --short HEAD)）"; return; fi
  log "clone: princeton-vl/DROID-SLAM（submodule 込み）"
  git clone --recursive https://github.com/princeton-vl/DROID-SLAM.git "$REPO"
  if [[ -n "$UPSTREAM_COMMIT" ]]; then
    git -C "$REPO" checkout -q "$UPSTREAM_COMMIT"
    git -C "$REPO" submodule update --init --recursive
  fi
  log "clone: $(git -C "$REPO" rev-parse HEAD)"
}

step_env() {
  if [[ -x "$PY" ]] && "$PY" -c 'import torch; assert torch.cuda.is_available()' 2>/dev/null; then
    log "env: $ENV_NAME はある"; return
  fi
  log "env: conda で python 3.11 と CUDA 12.8 の nvcc・ヘッダ"
  "$CONDA" create -y -n "$ENV_NAME" -c conda-forge \
    python=3.11 cuda-version=12.8 \
    cuda-nvcc cuda-cudart-dev cuda-driver-dev cuda-cccl cuda-nvrtc-dev cuda-nvtx-dev cuda-profiler-api \
    libcublas-dev libcusparse-dev libcusolver-dev libcurand-dev ffmpeg
  log "env: torch 2.7.0+cu128 と上流の requirements（open3d を除く）"
  "$PY" -m pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
  # pycolmap 4.2.1 が numpy を 2.4.6 に上げる（rerun 0.22.1・opencv 4.11・DROID はそのまま動いた）
  "$PY" -m pip install "opencv-python==4.11.0.86" "pyarrow==19.0.1" "rerun-sdk==0.22.1" \
    tqdm evo scipy gdown tensorboard pyyaml plyfile "pycolmap==4.2.1" "setuptools<80" wheel ninja pytest
}

step_build() {
  export CUDA_HOME="$ENV_DIR" PATH="$ENV_DIR/bin:$PATH" TORCH_CUDA_ARCH_LIST="12.0" MAX_JOBS="${MAX_JOBS:-8}"
  local tgt="$ENV_DIR/targets/x86_64-linux"
  export CPATH="$tgt/include${CPATH:+:$CPATH}"
  export LIBRARY_PATH="$tgt/lib:$tgt/lib/stubs${LIBRARY_PATH:+:$LIBRARY_PATH}"
  cd "$REPO"
  log "build: lietorch（同梱）"
  "$PY" -m pip install --no-build-isolation ./thirdparty/lietorch
  log "build: pytorch_scatter（同梱。時間がかかる）"
  "$PY" -m pip install --no-build-isolation ./thirdparty/pytorch_scatter
  log "build: droid_backends"
  "$PY" -m pip install --no-build-isolation -e .
  "$PY" -m pip check || true
  "$PY" -c 'import torch, lietorch, torch_scatter, droid_backends; print("ok", torch.__version__, torch.cuda.get_device_name(0))'
}

step_weights() {
  if [[ -s "$REPO/droid.pth" ]]; then log "weights: ある"; return; fi
  log "weights: droid.pth（Google Drive）"
  (cd "$REPO" && "$ENV_DIR/bin/gdown" "$WEIGHTS_GDRIVE_ID" -O droid.pth)
  ls -la "$REPO/droid.pth"
}

case "${1:-all}" in
  clone) step_clone ;;
  env) step_env ;;
  build) step_build ;;
  weights) step_weights ;;
  all) step_clone; step_env; step_weights; step_build ;;
  *) echo "usage: $0 [clone|env|build|weights|all]" >&2; exit 2 ;;
esac
