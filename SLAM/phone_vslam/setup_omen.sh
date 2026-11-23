#!/usr/bin/env bash
# OMEN に MASt3R-SLAM の環境を作る（1 度だけ。やり直しても壊れない）
#
#   使い方: bash setup_omen.sh [env|clone|weights|tum|build|all]
#   置き場所: ~/work_inoue/phone_slam/
#
# RTX 5060 Ti（sm_120）は CUDA 12.8 以上でないとカーネルを作れない。
# 上流の README は torch 2.5.1+cu124 だが、ここでは torch 2.7.0+cu128 にする
# （OMEN の doorman 環境で sm_120 を認識した組み合わせ）。
# nvcc は sudo 無しで入れるため conda-forge から取る。
# RTX 50 向けの手当ては patches/mast3r_slam_rtx50.patch（sm_120 を足す・torch 2.7 で消えた API）。
# torch 2.6 から torch.load の既定が weights_only=True になり MASt3R の重みが読めないので、
# 走らせるときは TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1 を付ける（start_slam.sh が付ける）。
set -euo pipefail

WORK="${WORK:-$HOME/work_inoue/phone_slam}"
ENV_NAME="${ENV_NAME:-mast3r}"
CONDA="$HOME/miniforge3/bin/mamba"
ENV_DIR="$HOME/miniforge3/envs/$ENV_NAME"
PY="$ENV_DIR/bin/python"
REPO="$WORK/MASt3R-SLAM"
UPSTREAM_COMMIT="${UPSTREAM_COMMIT:-e6f4e3d474fad0e11f561482012be864ba8c3f17}"  # 2026-11-17 に試した上流
LIETORCH_REF="${LIETORCH_REF:-e7df86554156b36846008d8ddbcc4d8521a16554}"
RERUN_VERSION="${RERUN_VERSION:-0.22.1}"  # 0.23 から numpy 2 必須＝上流と並べられない
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

step_env() {
  if [[ -x "$PY" ]] && "$PY" -c 'import torch; assert torch.cuda.is_available()' 2>/dev/null; then
    log "env: $ENV_NAME はある（torch $("$PY" -c 'import torch; print(torch.__version__)')）"
    return
  fi
  log "env: conda で python 3.11 と CUDA 12.8 の nvcc・ヘッダを入れる"
  "$CONDA" create -y -n "$ENV_NAME" -c conda-forge \
    python=3.11 cuda-version=12.8 \
    cuda-nvcc cuda-cudart-dev cuda-driver-dev cuda-cccl cuda-nvrtc-dev cuda-nvtx-dev cuda-profiler-api \
    libcublas-dev libcusparse-dev libcusolver-dev libcurand-dev \
    ffmpeg
  log "env: torch 2.7.0+cu128"
  "$PY" -m pip install torch==2.7.0 torchvision==0.22.0 \
    --index-url https://download.pytorch.org/whl/cu128
  "$PY" -c 'import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_arch_list())'
}

step_clone() {
  if [[ -d "$REPO/.git" ]]; then
    log "clone: $REPO はある（$(git -C "$REPO" rev-parse --short HEAD)）"
    return
  fi
  log "clone: rmurai0610/MASt3R-SLAM（submodule 込み）"
  # in3d の中の pyimgui は git-lfs を使う。OMEN の git-lfs は ~/.local/bin にあり、ssh の PATH に無い
  export PATH="$HOME/.local/bin:$PATH"
  git clone https://github.com/rmurai0610/MASt3R-SLAM.git "$REPO"
  git -C "$REPO" checkout -q "$UPSTREAM_COMMIT"
  git -C "$REPO" submodule update --init --recursive
}

step_weights() {
  local d="$REPO/checkpoints" base="https://download.europe.naverlabs.com/ComputerVision/MASt3R"
  mkdir -p "$d"
  for f in MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth \
           MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_trainingfree.pth \
           MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_codebook.pkl; do
    if [[ -s "$d/$f" ]]; then log "weights: $f はある"; continue; fi
    log "weights: $f を取る"
    curl -fL --retry 3 -o "$d/$f.part" "$base/$f"
    mv "$d/$f.part" "$d/$f"
  done
  ls -la "$d"
}

step_tum() {
  # 電話なしで通しを試すための録画（TUM RGB-D fr1_desk・約 360 MB）
  local d="$WORK/datasets/tum" name="rgbd_dataset_freiburg1_desk"
  if [[ -d "$d/$name/rgb" ]]; then log "tum: $name はある"; return; fi
  mkdir -p "$d"
  log "tum: $name を取る"
  curl -fL --retry 3 -o "$d/$name.tgz" \
    "https://cvg.cit.tum.de/rgbd/dataset/freiburg1/$name.tgz"
  tar xzf "$d/$name.tgz" -C "$d"
  rm "$d/$name.tgz"
}

step_build() {
  local nvcc="$ENV_DIR/bin/nvcc"
  [[ -x "$nvcc" ]] || { log "build: nvcc が無い（先に env）"; exit 1; }
  # 上流への手当て（RTX 50 向け）を当てる。当て済みなら飛ばす
  export PATH="$HOME/.local/bin:$PATH"   # git-lfs（submodule の状態を見るのに要る）
  if [[ -f "$HERE/patches/mast3r_slam_rtx50.patch" ]] && \
     git -C "$REPO" apply --check "$HERE/patches/mast3r_slam_rtx50.patch" 2>/dev/null; then
    log "build: パッチを当てる"
    git -C "$REPO" apply "$HERE/patches/mast3r_slam_rtx50.patch"
  fi
  export CUDA_HOME="$ENV_DIR"
  # conda-forge の CUDA は頭書き・ライブラリを targets/x86_64-linux に置く。torch は $CUDA_HOME/include
  # しか足さないので、g++ が cuda.h を見つけられず落ちる（lietorch で起きた）→ ここで道を通す
  local tgt="$ENV_DIR/targets/x86_64-linux"
  export CPATH="$tgt/include${CPATH:+:$CPATH}"
  export LIBRARY_PATH="$tgt/lib:$tgt/lib/stubs${LIBRARY_PATH:+:$LIBRARY_PATH}"
  export PATH="$ENV_DIR/bin:$PATH"
  export TORCH_CUDA_ARCH_LIST="12.0"
  export MAX_JOBS="${MAX_JOBS:-8}"
  cd "$REPO"
  log "build: 組み立ての道具"
  "$PY" -m pip install "setuptools<80" wheel ninja cython
  log "build: lietorch（$LIETORCH_REF）"
  "$PY" -m pip install --no-build-isolation \
    "lietorch @ git+https://github.com/princeton-vl/lietorch.git@$LIETORCH_REF"
  log "build: mast3r（curope を含む）"
  "$PY" -m pip install --no-build-isolation -e thirdparty/mast3r
  log "build: in3d（上流の可視化。import のためだけに入れる）"
  "$PY" -m pip install -e thirdparty/in3d
  log "build: mast3r_slam（backends。sm_60〜sm_120 の 7 つを作るので数分かかる）"
  "$PY" -m pip install --no-build-isolation -e .
  log "build: ウェブ表示（rerun-sdk $RERUN_VERSION）"
  # numpy 1.26.4（上流の固定）に合う版で揃える。新しい opencv-python・pyarrow は numpy 2 を引き込む
  "$PY" -m pip install "rerun-sdk==$RERUN_VERSION" "numpy==1.26.4" \
    "opencv-python==4.11.0.86" "pyarrow==19.0.1"
  "$PY" -m pip check
  TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1 "$PY" - <<'PY'
import torch, lietorch, mast3r_slam_backends, curope  # noqa: F401
print("ok", torch.__version__, torch.cuda.get_device_name(0))
PY
}

case "${1:-all}" in
  env) step_env ;;
  clone) step_clone ;;
  weights) step_weights ;;
  tum) step_tum ;;
  build) step_build ;;
  all) step_clone; step_env; step_weights; step_tum; step_build ;;
  *) echo "usage: $0 [env|clone|weights|tum|build|all]" >&2; exit 2 ;;
esac
