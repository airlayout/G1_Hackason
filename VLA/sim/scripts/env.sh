#!/usr/bin/env bash
# 共通設定。他のスクリプトから `source` して使う（単体では何もしない）。
# どの値も、同名の環境変数を先に export すれば上書きできる。

# Isaac-GR00T の clone 先。VLA/sim/README.md の手順で作る。
GROOT_DIR="${GROOT_DIR:-$HOME/Isaac-GR00T}"
# 動作を確認した Isaac-GR00T のコミット（setup_policy_env.sh が一致を警告する）。
GROOT_COMMIT="${GROOT_COMMIT:-7d5a455add459e870c2e4e4569006acace432d49}"
# 方策サーバー（ZMQ）。認証が無いので、既定は同じ PC からしか届かない 127.0.0.1。
#   POLICY_BIND_HOST  サーバーが待ち受けるアドレス（run_server.sh）。別の PC から使うときだけ 0.0.0.0 などにする
#   POLICY_HOST       クライアントが接続するサーバーのアドレス（run_client.sh / run_eval.sh）
POLICY_BIND_HOST="${POLICY_BIND_HOST:-127.0.0.1}"
POLICY_HOST="${POLICY_HOST:-127.0.0.1}"
POLICY_PORT="${POLICY_PORT:-5555}"
# 既定の checkpoint。cloudwalk 版に切り替えるなら CKPT=cloudwalk-research/GR00T-N1.6-G1-PnPAppleToPlate。
CKPT="${CKPT:-nvidia/GR00T-N1.6-G1-PnPAppleToPlate}"
ENV_NAME="${ENV_NAME:-gr00tlocomanip_g1_sim/LMPnPAppleToPlateDC_G1_gear_wbc}"
# 方策の環境（torch 等）と、sim の環境（MuJoCo 等）は別の venv。
POLICY_PY="${POLICY_PY:-$GROOT_DIR/.venv/bin/python}"
SIM_PY="${SIM_PY:-$GROOT_DIR/gr00t/eval/sim/GR00T-WholeBodyControl/GR00T-WholeBodyControl_uv/.venv/bin/python}"
# 1 回の rollout の設定（action chunk 20 ステップごとに方策へ問い合わせる）。
N_ACTION_STEPS="${N_ACTION_STEPS:-20}"

# git-lfs を sudo なしで入れる先（install_git_lfs.sh）。uv もここに置く想定。
export PATH="$HOME/.local/bin:$PATH"

# 前提のディレクトリが無ければ止める。
require_groot_dir() {
  if [ ! -d "$GROOT_DIR/.git" ]; then
    echo "[error] GROOT_DIR=$GROOT_DIR に Isaac-GR00T の clone がありません。" >&2
    echo "        VLA/sim/README.md の「1. Isaac-GR00T を用意する」を実行するか、GROOT_DIR を指定してください。" >&2
    return 1
  fi
}
