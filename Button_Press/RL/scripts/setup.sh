#!/usr/bin/env bash
# Isaac Sim（pip 版）+ Isaac Lab + rsl_rl を、configs/versions.yaml の版で入れる（新しいマシン用）。
# すでにあるものは飛ばすので、何度実行してもよい。最後に check_setup.sh で確かめる。
#
#   bash Button_Press/RL/scripts/setup.sh                    # 既定: ~/NVIDIA/env_isaaclab と ~/NVIDIA/IsaacLab
#   ISAAC_SIM=/path/to/venv ISAACLAB=/path/to/IsaacLab bash Button_Press/RL/scripts/setup.sh
#
# 必要なもの: Ubuntu 22.04 / 24.04、python3.12（venv が作れること）、NVIDIA ドライバ 570 以上、ディスク 60 GB 程度、
# ネットワーク（pypi.nvidia.com と github.com）。Isaac Sim のダウンロードに 20〜40 分かかる。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$RL_DIR/../.." && pwd)"
ISAAC_SIM="${ISAAC_SIM:-$HOME/NVIDIA/env_isaaclab}"
ISAACLAB="${ISAACLAB:-$HOME/NVIDIA/IsaacLab}"

ver() { python3 -c "import yaml,sys; print(yaml.safe_load(open('$RL_DIR/configs/versions.yaml'))[sys.argv[1]])" "$1" 2>/dev/null \
        || grep "^$1:" "$RL_DIR/configs/versions.yaml" | sed 's/^[^:]*: *"\{0,1\}\([^"]*\)"\{0,1\}.*/\1/'; }
V_SIM="$(ver isaacsim)"; V_BRANCH="$(ver isaaclab_branch)"; V_COMMIT="$(ver isaaclab_commit)"
V_PY="$(ver python)"; MIN_DRIVER="$(ver min_driver)"

echo "[setup] Isaac Sim $V_SIM → $ISAAC_SIM"
echo "[setup] Isaac Lab $V_BRANCH（$V_COMMIT）→ $ISAACLAB"

# 1. GPU とドライバ
DRIVER="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || true)"
if [[ -z "$DRIVER" ]]; then
    echo "[NG] nvidia-smi が動かない（NVIDIA ドライバを入れる）" >&2; exit 1
fi
if (( ${DRIVER%%.*} < MIN_DRIVER )); then
    echo "[NG] NVIDIA ドライバ $DRIVER は古い（$MIN_DRIVER 以上が要る）" >&2; exit 1
fi
echo "[OK] GPU: $(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader | head -1)、ドライバ $DRIVER"

# 2. Python の venv と Isaac Sim
if [[ ! -x "$ISAAC_SIM/bin/python" ]]; then
    command -v "python$V_PY" >/dev/null || { echo "[NG] python$V_PY が無い（sudo apt install python$V_PY python$V_PY-venv）" >&2; exit 1; }
    "python$V_PY" -m venv "$ISAAC_SIM"
    "$ISAAC_SIM/bin/python" -m pip install --quiet --upgrade pip
fi
PY="$ISAAC_SIM/bin/python"
# 入っている版（pip show の出力をいったん変数に入れる。`pip show | grep -q` は pipefail の下で、grep が先に
# 終わると pip が失敗扱いになり「入っていない」と誤って判定する。2026-10-19 にこれで torch を入れ替えてしまった）
pkg_version() { local out; out="$("$PY" -m pip show "$1" 2>/dev/null || true)"; sed -n 's/^Version: //p' <<<"$out"; }
if [[ "$(pkg_version isaacsim)" != "$V_SIM" ]]; then
    echo "[setup] Isaac Sim を入れる（20〜40 分）"
    "$PY" -m pip install "isaacsim[all,extscache]==$V_SIM" --extra-index-url https://pypi.nvidia.com
fi
# pip 版には python.sh が無いので、薄いラッパーを置く（IsaacSim_Env/SETUP.md の「環境の例②」）
if [[ ! -x "$ISAAC_SIM/python.sh" ]]; then
    cat > "$ISAAC_SIM/python.sh" <<'WRAP'
#!/bin/bash
# pip 版 Isaac Sim の venv には python.sh が無いための薄いラッパー
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/bin/python" "$@"
WRAP
    chmod +x "$ISAAC_SIM/python.sh"
fi

# 3. Isaac Lab（版を固定）
if [[ ! -d "$ISAACLAB/.git" ]]; then
    git clone --branch "$V_BRANCH" https://github.com/isaac-sim/IsaacLab.git "$ISAACLAB"
fi
HEAD="$(git -C "$ISAACLAB" rev-parse HEAD)"
if [[ "$HEAD" != "$V_COMMIT" ]]; then
    if [[ -n "$(git -C "$ISAACLAB" status --porcelain --untracked-files=no)" ]]; then
        echo "[NG] $ISAACLAB に変更がある。版を合わせられない（変更を退避してからやり直す）" >&2; exit 1
    fi
    git -C "$ISAACLAB" fetch origin "$V_BRANCH"
    git -C "$ISAACLAB" checkout --quiet "$V_COMMIT"
fi
if [[ -z "$(pkg_version isaaclab_rl)" ]]; then
    echo "[setup] Isaac Lab を入れる（torch $(grep '^torch:' "$RL_DIR/configs/versions.yaml" | cut -d'"' -f2) と rsl_rl を含む）"
    # isaaclab.sh は、有効にした venv の Python に入れる。torch は CUDA 12.8 版（cu128）を入れる
    (cd "$ISAACLAB" && VIRTUAL_ENV="$ISAAC_SIM" PATH="$ISAAC_SIM/bin:$PATH" ./isaaclab.sh --install 'rl[rsl-rl]')
fi
# ⚠️ rsl-rl-lib や torch を pip install で単独に入れ直さないこと。依存で torch が PyPI の版（CUDA 13 版）に
#    入れ替わり、ドライバ 570 では GPU で動かなくなる。版が違えば check_setup.sh が知らせるので、手で直す

# 4. G1 の公式モデル（J1-gen と共通）
if [[ ! -f "$REPO_ROOT/_local/button_press/models/g1_description/g1_29dof_rev_1_0.urdf" ]]; then
    bash "$REPO_ROOT/Button_Press/J1-gen/sim/fetch_models.sh"
fi

echo "[setup] 入れ終わった。確かめる:"
ISAAC_SIM="$ISAAC_SIM" ISAACLAB="$ISAACLAB" bash "$SCRIPT_DIR/check_setup.sh"
