#!/usr/bin/env bash
# Isaac Sim + IsaacLab + ROS 2 を同一 Python から使うための共通環境設定。
#
# この環境では isaacsim と isaaclab が別々の Python に入っているため、
# Isaac Sim の python.sh に PYTHONPATH を通して両方を使えるようにする。
# （./isaaclab.sh は使えない。詳細は G1/CLAUDE.md 参照）
#
# 使い方（各スクリプトから読み込む）:
#   source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
#   "$ISAAC_SIM/python.sh" some_script.py

# ============================================================
# 環境ごとに変更が必要なのはこの 3 行だけ。
# 他のスクリプトは env.sh を source するか、自分のファイル位置から
# 相対でパスを求めるので、ここ以外に書き換える箇所は無い。
# ============================================================
ISAAC_SIM=/home/spacedata/isaacSim6.0dev2/_build/linux-x86_64/release
ISAACLAB=/home/spacedata/IsaacLab
ENV_SH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# editable install の実ソースと、依存パッケージ（warp, rsl_rl 等）の両方を通す
LAB_SOURCES=$(ls -d "$ISAACLAB"/source/*/ | tr '\n' ':')
LAB_SITE_PACKAGES="$ISAACLAB/env_isaaclab/lib/python3.12/site-packages"

# ROS 2 Jazzy（rclpy / tf2_ros / 各メッセージ型）を通す。
# Isaac Sim と ROS 2 はどちらも Python 3.12 なので ABI が一致し、
# python.sh から rclpy を直接 import できる（疎通確認済み）。
ROS_SETUP=/opt/ros/jazzy/setup.bash
if [[ -f "$ROS_SETUP" ]]; then
    # set -u 下でも落ちないよう一時的に無効化する（ROS の setup は未定義変数を触る）
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    set -u
fi

export PYTHONPATH="${ENV_SH_DIR}/src:${LAB_SOURCES}${LAB_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"
export DISPLAY="${DISPLAY:-:1}"

# print() をバッファリングさせない（tee 越しでも進捗が即座に見えるように）
export PYTHONUNBUFFERED=1

# 起動時に blas_thread_shutdown / __libc_fork 内で segfault することがあるため
# BLAS のスレッド数を 1 に抑える（Isaac Sim の fork と OpenBLAS の相性問題）
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

export ISAAC_SIM ISAACLAB
