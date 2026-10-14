#!/usr/bin/env bash
# DoorMan（NVlabs/GR00T-VisualSim2Real の doorman ブランチ）の環境を OMEN に作る。
# 既存の env_isaaclab（Isaac Sim 6.0.1）と conda 環境 tv には触らない。
# 置き場所: ~/work_inoue/doorman/{IsaacLab,VIRAL,VIRAL29,LAFAN-G1}、conda 環境: doorman51
#
# 使い方（OMEN で）: このディレクトリ（constraints.txt・patches/ と同じ場所）から
#   ./setup_doorman.sh > setup.log 2>&1      # 完了の目印は最後の SETUP_DONE
#
# 2026-10-06 の実績: README の手順どおりに流すと次の 3 つで壊れた。ここでは最初から防いでいる。
#   1. flatdict 4.0.1 のビルドが pkg_resources 無しで失敗（setuptools 84）→ setuptools<80
#   2. isaaclab_tasks・isaaclab_rl の依存で torch が 2.14.1+cu130 に黙って上がる → constraints.txt
#   3. DoorMan（trl 0.17）の依存で transformers 5.18・numpy 2.4 に上がる → constraints.txt
# ⚠️ 当日は「README どおり → fix → 小物の固定」と 3 回に分けて流した。この通しの形ではまだ流していない。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
BASE="$HOME/work_inoue/doorman"
ENV_NAME="doorman51"
CONDA="$HOME/miniforge3/bin/conda"
ENV_BIN="$HOME/miniforge3/envs/$ENV_NAME/bin"
ISAACLAB_TAG="v2.3.2"   # Isaac Sim 5.1 に対応する Isaac Lab
DOORMAN_COMMIT="016c70c1e4e76f521963c36691ee69a6ab3ac9cd"  # doorman ブランチ 2026-04-20 "Clean up"
CU128="https://download.pytorch.org/whl/cu128"

step() { echo "=== [$(date +%T)] $*"; }
torch_check() {
  "$ENV_BIN/python" -c "import torch, sys; v=torch.__version__; print('torch', v, 'cuda', torch.cuda.is_available()); sys.exit(0 if v.startswith('2.7.0+cu128') and torch.cuda.is_available() else 1)"
}

mkdir -p "$BASE"
cp "$HERE/constraints.txt" "$BASE/constraints.txt"
cd "$BASE"
C="-c $BASE/constraints.txt"

step "1. conda 環境 $ENV_NAME（Python 3.11 と git-lfs。sudo を使わない）"
ENVS="$("$CONDA" env list)"
if ! grep -q "^$ENV_NAME " <<< "$ENVS"; then
  "$CONDA" create -y -n "$ENV_NAME" -c conda-forge python=3.11 git-lfs
fi
export PATH="$ENV_BIN:$PATH"   # git-lfs もこの中にしか無い
PIP="$ENV_BIN/python -m pip"
git lfs install --skip-repo

step "2. PyTorch 2.7.0（cu128。RTX 5060 Ti = sm_120 を含む）"
$PIP install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url "$CU128"
torch_check

step "3. Isaac Sim 5.1"
$PIP install $C isaacsim==5.1.0.0 isaacsim-rl==5.1.0.0 --extra-index-url https://pypi.nvidia.com
torch_check

step "4. Isaac Lab $ISAACLAB_TAG"
[ -d IsaacLab ] || git clone --depth 1 -b "$ISAACLAB_TAG" https://github.com/isaac-sim/IsaacLab.git
$PIP install $C "setuptools<80" poetry-core
$PIP install $C --no-build-isolation flatdict==4.0.1
$PIP install $C --extra-index-url "$CU128" --no-build-isolation \
  -e IsaacLab/source/isaaclab -e IsaacLab/source/isaaclab_assets -e IsaacLab/source/isaaclab_tasks \
  -e "IsaacLab/source/isaaclab_rl[all]"
torch_check

step "5. DoorMan（commit 固定。LFS の重み・模型も取る）"
if [ ! -d VIRAL ]; then
  git clone -b doorman --single-branch https://github.com/NVlabs/GR00T-VisualSim2Real.git VIRAL
fi
git -C VIRAL checkout -q "$DOORMAN_COMMIT"
(cd VIRAL && git lfs pull)
$PIP install $C --extra-index-url "$CU128" -e VIRAL
torch_check

step "6. 指の無い 29 関節版の作業ツリー VIRAL29（patches/ を当てる）"
if [ ! -d VIRAL29 ]; then
  git -C VIRAL worktree add -q ../VIRAL29 "$DOORMAN_COMMIT"
  for p in "$HERE"/patches/*.patch; do
    git -C VIRAL29 -c user.name=g1 -c user.email=g1@local am -q "$p"
  done
fi
git -C VIRAL29 log --oneline -3

step "7. LAFAN-G1（リセット用の動作データ、CC-BY-4.0、約 475 MB）"
[ -d LAFAN-G1 ] || git clone https://huggingface.co/datasets/ember-lab-berkeley/LAFAN-G1
(cd LAFAN-G1 && git lfs pull)

step "8. 確認"
torch_check
$PIP check || true   # 既知: fastapi と starlette、wheel と packaging（Isaac Sim 5.1 と Isaac Lab v2.3.2 の組み合わせ自体の食い違い）
"$ENV_BIN/python" -c "import isaaclab; print('isaaclab', isaaclab.__file__)"
ls -la VIRAL/models/model_walk.pt VIRAL/models/model_stand.pt
du -sh "$BASE"/* 2>/dev/null
df -h "$HOME" | tail -1
echo "SETUP_DONE"
