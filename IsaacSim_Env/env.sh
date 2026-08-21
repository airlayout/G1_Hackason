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
ISAAC_SIM=/home/ubuntu/NVIDIA/env_isaaclab
ISAACLAB=/home/ubuntu/NVIDIA/IsaacLab
ENV_SH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# パスが正しいか先に確かめる。
# 間違っていると「python.sh がありません」「ls: アクセスできません」といった
# 分かりにくいエラーになり、env.sh を直せばよいと気付きにくいため。
if [[ ! -x "$ISAAC_SIM/python.sh" ]]; then
    echo "[NG] Isaac Sim が見つかりません: $ISAAC_SIM" >&2
    echo "     env.sh の ISAAC_SIM を自分の環境に合わせてください。" >&2
    echo "     python.sh がある階層を指定します。例:" >&2
    echo "       ISAAC_SIM=/path/to/isaacsim/_build/linux-x86_64/release" >&2
    echo "     探す場合: find / -name python.sh -path '*isaac*' 2>/dev/null" >&2
    echo "     pip 版 Isaac Sim（venv に isaacsim をインストールした場合）には" >&2
    echo "     python.sh が存在しない。venv 直下に次の薄いラッパーを置けばよい:" >&2
    echo "       #!/bin/bash" >&2
    echo '       exec "$(dirname "${BASH_SOURCE[0]}")/bin/python" "$@"' >&2
    return 1 2>/dev/null || exit 1
fi

if [[ ! -d "$ISAACLAB/source" ]]; then
    echo "[NG] IsaacLab が見つかりません: $ISAACLAB" >&2
    echo "     env.sh の ISAACLAB を自分の環境に合わせてください。" >&2
    echo "     source/ ディレクトリがある階層を指定します。例:" >&2
    echo "       ISAACLAB=/path/to/IsaacLab" >&2
    return 1 2>/dev/null || exit 1
fi

# editable install の実ソースと、依存パッケージ（warp, rsl_rl 等）の両方を通す
LAB_SOURCES=$(ls -d "$ISAACLAB"/source/*/ | tr '\n' ':')
# pip 版 Isaac Sim（isaacsim と isaaclab が同じ venv = ISAAC_SIM に同居）では
# site-packages は ISAACLAB 配下ではなく ISAAC_SIM 配下にある。
LAB_SITE_PACKAGES="$ISAAC_SIM/lib/python3.12/site-packages"

# ROS 2 Jazzy（rclpy / tf2_ros / 各メッセージ型）を通す。
# Isaac Sim と ROS 2 はどちらも Python 3.12 なので ABI が一致し、
# python.sh から rclpy を直接 import できる（疎通確認済み）。
ROS_SETUP=/opt/ros/jazzy/setup.bash
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "[WARN] ROS 2 の setup.bash が見つかりません: $ROS_SETUP" >&2
    echo "       SLAM / Nav2 は使えません（キーボード操作のみ可）。" >&2
    echo "       別の版を使う場合は env.sh の ROS_SETUP を変えてください。" >&2
fi
if [[ -f "$ROS_SETUP" ]]; then
    # set -u 下でも落ちないよう一時的に無効化する（ROS の setup は未定義変数を触る）
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    set -u
fi

export PYTHONPATH="${ENV_SH_DIR}/src:${LAB_SOURCES}${LAB_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"
export DISPLAY="${DISPLAY:-:1}"

# IsaacLab はアセットのダウンロードキャッシュを $TMPDIR（既定 /tmp）配下の
# 固定パス（/tmp/Assets/...）に書き込む。共有マシンで他ユーザーが所有する
# /tmp/Assets が既に存在し書き込み権限が無い場合に備え、自分専用のキャッシュ
# ディレクトリを使う。
mkdir -p "${TMPDIR:-/home/ubuntu/NVIDIA/.isaac_asset_cache}"
export TMPDIR="${TMPDIR:-/home/ubuntu/NVIDIA/.isaac_asset_cache}"

# print() をバッファリングさせない（tee 越しでも進捗が即座に見えるように）
export PYTHONUNBUFFERED=1

# 起動時に blas_thread_shutdown / __libc_fork 内で segfault することがあるため
# BLAS のスレッド数を 1 に抑える（Isaac Sim の fork と OpenBLAS の相性問題）
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

export ISAAC_SIM ISAACLAB
