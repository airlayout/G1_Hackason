#!/usr/bin/env bash
# Button_Press/RL の環境設定。評価環境（Yada）の env.sh（Isaac Sim と Isaac Lab の場所）を読み、
# このパッケージ（button_press_rl）を import できるようにする。
#
# 使い方: source Button_Press/RL/scripts/env.sh
# Isaac Sim・Isaac Lab の場所が違うマシンでは、ISAAC_SIM / ISAACLAB を環境変数で渡す（SETUP.md）。

_RL_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RL_DIR="$(cd "$_RL_SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$RL_DIR/../.." && pwd)"
# 既定の場所は各自のホームの下（setup.sh と同じ）。Yada の env.sh の既定はこのマシンのパスなので、先に決めておく
export ISAAC_SIM="${ISAAC_SIM:-$HOME/NVIDIA/env_isaaclab}"
export ISAACLAB="${ISAACLAB:-$HOME/NVIDIA/IsaacLab}"
export TMPDIR="${TMPDIR:-$HOME/NVIDIA/.isaac_asset_cache}"
source "$REPO_ROOT/Button_Press/Yada/sim/isaac/env.sh" || return 1 2>/dev/null || exit 1
export ISAAC_SIM ISAACLAB
export PYTHONPATH="$RL_DIR${PYTHONPATH:+:${PYTHONPATH}}"
# 学習の出力（ログ・checkpoint）の置き場所。git の対象外
export RL_OUT_DIR="${RL_OUT_DIR:-$REPO_ROOT/_local/button_press_rl}"
mkdir -p "$RL_OUT_DIR"
