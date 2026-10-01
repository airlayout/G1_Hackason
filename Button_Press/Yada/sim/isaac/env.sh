#!/usr/bin/env bash
# Isaac Sim + IsaacLab を同じ Python から使うための環境設定（Button_Press/Yada/sim/isaac 用）。
#
# 考え方は IsaacSim_Env/env.sh と同じ（./isaaclab.sh は使えないので、PYTHONPATH で IsaacLab を通す。
# 理由はリポジトリ直下の CLAUDE.md）。ROS 2 は使わないので読み込まない。
#
# 使い方: source Button_Press/Yada/sim/isaac/env.sh

# ============================================================
# 環境ごとに変えるのはこの 2 行だけ（環境変数で上書きもできる）
#   ISAAC_SIM: python.sh がある階層（pip 版は venv 直下に python.sh のラッパーを置く。IsaacSim_Env/SETUP.md）
#   ISAACLAB : source/ がある階層
# ============================================================
ISAAC_SIM="${ISAAC_SIM:-/home/ubuntu/NVIDIA/env_isaaclab}"
ISAACLAB="${ISAACLAB:-/home/ubuntu/NVIDIA/IsaacLab}"

if [[ ! -x "$ISAAC_SIM/python.sh" ]]; then
    echo "[NG] Isaac Sim が見つかりません: $ISAAC_SIM（env.sh の ISAAC_SIM を直してください）" >&2
    return 1 2>/dev/null || exit 1
fi
if [[ ! -d "$ISAACLAB/source" ]]; then
    echo "[NG] IsaacLab が見つかりません: $ISAACLAB（env.sh の ISAACLAB を直してください）" >&2
    return 1 2>/dev/null || exit 1
fi

LAB_SOURCES=$(ls -d "$ISAACLAB"/source/*/ | tr '\n' ':')
# pip 版では isaaclab の依存（warp, rsl_rl など）は ISAAC_SIM の venv の site-packages にある
LAB_SITE_PACKAGES="$ISAAC_SIM/lib/python3.12/site-packages"
export PYTHONPATH="${LAB_SOURCES}${LAB_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"
export DISPLAY="${DISPLAY:-:1}"

# IsaacLab はアセットのキャッシュを $TMPDIR に書く。共有マシンで /tmp/Assets が他ユーザーのものだと書けないため
export TMPDIR="${TMPDIR:-/home/ubuntu/NVIDIA/.isaac_asset_cache}"
mkdir -p "$TMPDIR"

export PYTHONUNBUFFERED=1
# 起動時に blas_thread_shutdown / __libc_fork で segfault することがあるため（IsaacSim_Env/run.sh と同じ対策）
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
