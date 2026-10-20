#!/usr/bin/env bash
# Button_Press/RL の環境設定。評価環境（Yada）の env.sh（Isaac Sim と Isaac Lab の場所）を読み、
# このパッケージ（button_press_rl）を import できるようにする。
#
# 使い方: source Button_Press/RL/scripts/env.sh
# Isaac Sim・Isaac Lab の場所が違うマシンでは、ISAAC_SIM / ISAACLAB を環境変数で渡す（SETUP.md）。

_RL_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RL_DIR="$(cd "$_RL_SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$RL_DIR/../.." && pwd)"
# 場所の決め方: 環境変数 > setup.sh の記録（_local/button_press_rl/paths.env）> 既定（各自のホームの下）。
# Yada の env.sh の既定はこのマシンのパスなので、先に決めておく
_RL_PATHS="$REPO_ROOT/_local/button_press_rl/paths.env"
if [[ -f "$_RL_PATHS" ]]; then
    while IFS='=' read -r _k _v; do
        [[ "$_k" =~ ^(ISAAC_SIM|ISAACLAB|MUJOCO_PYTHON)$ ]] || continue
        [[ -z "${!_k:-}" ]] && export "$_k=$_v"
    done < "$_RL_PATHS"
fi
export ISAAC_SIM="${ISAAC_SIM:-$HOME/NVIDIA/env_isaaclab}"
export ISAACLAB="${ISAACLAB:-$HOME/NVIDIA/IsaacLab}"
# MuJoCo での採点に使う Python（setup.sh が探す。既定は Isaac Sim の環境。mujoco と pinocchio も入っている）
export MUJOCO_PYTHON="${MUJOCO_PYTHON:-$ISAAC_SIM/bin/python}"
export TMPDIR="${TMPDIR:-$HOME/NVIDIA/.isaac_asset_cache}"
source "$REPO_ROOT/Button_Press/Yada/sim/isaac/env.sh" || return 1 2>/dev/null || exit 1
export ISAAC_SIM ISAACLAB
export PYTHONPATH="$RL_DIR${PYTHONPATH:+:${PYTHONPATH}}"
# 学習の出力（ログ・checkpoint）の置き場所。git の対象外
export RL_OUT_DIR="${RL_OUT_DIR:-$REPO_ROOT/_local/button_press_rl}"
mkdir -p "$RL_OUT_DIR"
