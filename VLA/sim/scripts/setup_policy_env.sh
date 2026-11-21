#!/usr/bin/env bash
# 方策（GR00T N1.6）側の Python 環境を作り、GPU で計算できることを確認する。
#
# 経緯: RTX 5060 Ti（Blackwell, sm_120）は cu126 の torch では動かない。`uv sync` は cu126 を入れるため、
# sync の後に torch / torchvision を cu128 ビルドへ入れ替える。nvcc が無い環境では flash-attn が
# 入らないので外す（その代わり patch_no_flash_attn.py で SDPA に切り替える）。
# 以後この venv では `uv sync` を再実行しない（cu126 に戻る）。実行は venv の python を直接使う。
#
# 使い方: bash setup_policy_env.sh        （前提: uv が PATH にある。無ければ https://docs.astral.sh/uv/ ）
#   TORCH_BACKEND=cu128   torch の CUDA ビルド（GPU に合わせて変える。sync と入れ替えの両方に使う）
#   TORCH_INDEX=...       torch の取得先（既定は https://download.pytorch.org/whl/$TORCH_BACKEND）
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"
require_groot_dir

if ! command -v uv >/dev/null 2>&1; then
  echo "[error] uv が見つかりません。https://docs.astral.sh/uv/ の手順で入れてください（~/.local/bin に入る）。" >&2
  exit 1
fi

TORCH_BACKEND="${TORCH_BACKEND:-cu128}"
TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/$TORCH_BACKEND}"
TORCH_VER="${TORCH_VER:-2.7.1}"
TORCHVISION_VER="${TORCHVISION_VER:-0.22.1}"

cd "$GROOT_DIR"
HEAD_COMMIT="$(git rev-parse HEAD)"
git log -1 --format='[groot] %h %s (%cd)'
if [ "$HEAD_COMMIT" != "$GROOT_COMMIT" ]; then
  echo "[warn] 動作確認したコミット $GROOT_COMMIT と異なります（HEAD=$HEAD_COMMIT）。パッチが当たらないことがあります。" >&2
fi

UV_TORCH_BACKEND="$TORCH_BACKEND" uv sync --python 3.10 --extra gpu --no-install-package flash-attn
uv pip install --python .venv/bin/python --reinstall-package torch --reinstall-package torchvision \
  "torch==$TORCH_VER" "torchvision==$TORCHVISION_VER" --index-url "$TORCH_INDEX"

"$POLICY_PY" - <<'PY'
import torch
print('[groot] torch', torch.__version__, 'cuda', torch.version.cuda, 'arch', torch.cuda.get_arch_list())
a = torch.randn(2048, 2048, device='cuda', dtype=torch.bfloat16)
print('[groot] matmul ok', float((a @ a).float().abs().mean()))
PY

# deepspeed は推論に不要で、CUDA_HOME（nvcc）が無いと import で落ちる。入っていれば外す。
if "$POLICY_PY" -c "import deepspeed" 2>/dev/null; then
  echo "[groot] deepspeed を外す（nvcc なしでは import に失敗するため）"
  uv pip uninstall --python .venv/bin/python deepspeed
fi
echo "[groot] POLICY ENV DONE"
