#!/usr/bin/env bash
# Isaac Sim（pip 版）+ Isaac Lab + rsl_rl と、MuJoCo の評価用の Python を用意する（configs/versions.yaml の版）。
#
#   bash Button_Press/RL/scripts/setup.sh              # 探して、あればそれを使う。無い・版が違えば聞く
#   bash Button_Press/RL/scripts/setup.sh --dry-run    # 探して結果を表示するだけ（何も入れない・記録しない）
#   bash Button_Press/RL/scripts/setup.sh --yes        # 聞かずに既定の答え（無い・版が違えば新しく入れる）
#
# 1. 探す: 環境変数（ISAAC_SIM / ISAACLAB / MUJOCO_PYTHON）→ 前回の記録（_local/button_press_rl/paths.env）→
#    既定の場所（~/NVIDIA/env_isaaclab、~/NVIDIA/IsaacLab）→ よくある場所（~/NVIDIA/*、~/*、conda の環境）の順
# 2. 決めた版のものが見つかれば、それをそのまま使う（何も入れ直さない）
# 3. 見つからない・版が違うときは聞く。新しく入れるときは、見つかった環境には手を入れず、別の場所に入れる
#    （ほかの人や Yada の評価が使っている環境を壊さないため。FAILURES.md の 2026-10-19）
# 4. 選んだ場所を _local/button_press_rl/paths.env に記録する。以後のスクリプトは env.sh を通してこれを読む
#
# 必要なもの: Ubuntu 22.04 / 24.04、python3.12（venv が作れること）、NVIDIA ドライバ 570 以上、ディスク 50 GB 以上、
# ネットワーク（pypi.nvidia.com、download.pytorch.org、github.com）。Isaac Sim のダウンロードに 20〜40 分かかる。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$RL_DIR/../.." && pwd)"
PATHS_FILE="$REPO_ROOT/_local/button_press_rl/paths.env"

DRY_RUN=0
ASSUME_YES=0
for a in "$@"; do
    case "$a" in
        --dry-run) DRY_RUN=1 ;;
        --yes|-y) ASSUME_YES=1 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "[NG] 知らない引数: $a（--dry-run / --yes）" >&2; exit 2 ;;
    esac
done

ver() { grep "^$1:" "$RL_DIR/configs/versions.yaml" | sed 's/^[^:]*: *"\{0,1\}\([^"]*\)"\{0,1\}.*/\1/'; }
V_SIM="$(ver isaacsim)"; V_BRANCH="$(ver isaaclab_branch)"; V_COMMIT="$(ver isaaclab_commit)"
V_PY="$(ver python)"; MIN_DRIVER="$(ver min_driver)"; V_MUJOCO_MIN="$(ver mujoco_min)"
DEFAULT_SIM="$HOME/NVIDIA/env_isaaclab"
DEFAULT_LAB="$HOME/NVIDIA/IsaacLab"
DEFAULT_MUJOCO_ENV="$HOME/NVIDIA/env_mujoco"

# 前回の記録（環境変数で指定されたものが優先）
if [[ -f "$PATHS_FILE" ]]; then
    while IFS='=' read -r k v; do
        [[ "$k" =~ ^(ISAAC_SIM|ISAACLAB|MUJOCO_PYTHON)$ ]] || continue
        [[ -z "${!k:-}" ]] && printf -v "$k" '%s' "$v"
    done < "$PATHS_FILE"
fi

# ---- 共通の部品 --------------------------------------------------------------

# 聞く。答えを標準出力に返す（質問は標準エラーに出す）。--yes なら既定の答え。対話できなければ止める
ask() {
    local question="$1" default="$2" ans
    if (( ASSUME_YES )); then echo "$default"; return; fi
    if (( DRY_RUN )); then echo "  （ここで聞く）$question" >&2; echo "dry"; return; fi
    if [[ ! -t 0 ]]; then
        echo "[NG] 確認が要るが、対話できない（端末から実行するか、--yes で既定の答え「$default」にする）: $question" >&2
        exit 2
    fi
    read -r -p "$question [$default] " ans
    echo "${ans:-$default}"
}

# Python の中のパッケージの版（無ければ空）。import はせず、配布の情報だけを読む（速い）
pkg_version() {
    "$1" -c "import importlib.metadata as m, sys
try: print(m.version(sys.argv[1]))
except Exception: pass" "$2" 2>/dev/null || true
}

# Isaac Lab の取得先（Isaac Sim の環境に editable で入っていれば、その場所）
lab_of_env() {
    local out loc
    out="$("$1" -m pip show isaaclab 2>/dev/null || true)"
    loc="$(sed -n 's/^Editable project location: //p' <<<"$out")"
    [[ -n "$loc" ]] && (cd "$loc/../.." 2>/dev/null && pwd) || true
}

# 新しく入れる先（既定の場所がふさがっていれば、版の名前を付けた場所）
fresh_path() {
    local base="$1" tag="$2"
    if [[ ! -e "$base" ]]; then echo "$base"; else echo "${base}_${tag}"; fi
}

record() {
    (( DRY_RUN )) && return
    mkdir -p "$(dirname "$PATHS_FILE")"
    {
        echo "# setup.sh が選んだ場所（$(date '+%Y-%m-%d %H:%M')）。scripts/env.sh が読む。消すと次の setup.sh で探し直す"
        echo "ISAAC_SIM=$ISAAC_SIM"
        echo "ISAACLAB=$ISAACLAB"
        echo "MUJOCO_PYTHON=$MUJOCO_PYTHON"
    } > "$PATHS_FILE"
    echo "[setup] 記録した: $PATHS_FILE"
}

# ---- 1. GPU とドライバ -------------------------------------------------------

DRIVER="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || true)"
if [[ -z "$DRIVER" ]]; then
    echo "[NG] nvidia-smi が動かない（NVIDIA ドライバを入れる）" >&2; exit 1
fi
if (( ${DRIVER%%.*} < MIN_DRIVER )); then
    echo "[NG] NVIDIA ドライバ $DRIVER は古い（$MIN_DRIVER 以上が要る）" >&2; exit 1
fi
echo "[OK] GPU: $(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader | head -1)、ドライバ $DRIVER"

# ---- 2. Isaac Sim を探す -----------------------------------------------------

echo "[setup] Isaac Sim $V_SIM を探す"
declare -a SIM_CANDIDATES=()
for d in "${ISAAC_SIM:-}" "$DEFAULT_SIM" "$HOME"/NVIDIA/*/ "$HOME"/*/ "$HOME"/miniconda3/envs/*/ "$HOME"/anaconda3/envs/*/ \
         "$HOME"/miniforge3/envs/*/; do
    d="${d%/}"
    [[ -n "$d" && -x "$d/bin/python" ]] || continue
    [[ " ${SIM_CANDIDATES[*]-} " == *" $d "* ]] && continue
    v="$(pkg_version "$d/bin/python" isaacsim)"
    [[ -n "$v" ]] || continue
    SIM_CANDIDATES+=("$d")
    echo "  見つかった: $d（Isaac Sim $v）"
done

SIM_CHOSEN=""
SIM_INSTALL=0
for d in "${SIM_CANDIDATES[@]+"${SIM_CANDIDATES[@]}"}"; do
    if [[ "$(pkg_version "$d/bin/python" isaacsim)" == "$V_SIM" ]]; then SIM_CHOSEN="$d"; break; fi
done
if [[ -n "$SIM_CHOSEN" ]]; then
    echo "[OK] Isaac Sim $V_SIM の環境を使う: $SIM_CHOSEN"
elif (( ${#SIM_CANDIDATES[@]} > 0 )); then
    first="${SIM_CANDIDATES[0]}"
    new="$(fresh_path "$DEFAULT_SIM" "$V_SIM")"
    echo "[!!] 版の違う Isaac Sim しか無い（${first}: $(pkg_version "$first/bin/python" isaacsim)、必要なのは $V_SIM）"
    ans="$(ask "  1) $new に新しく入れる  2) $first をそのまま使う（動かない恐れ）  3) やめる:" 1)"
    case "$ans" in
        1|dry) SIM_CHOSEN="$new"; SIM_INSTALL=1 ;;
        2) SIM_CHOSEN="$first"; echo "[!!] 版の違う Isaac Sim を使う。動かなければ、もう一度 setup.sh で 1 を選ぶ" ;;
        *) echo "[setup] やめた"; exit 1 ;;
    esac
else
    new="$(fresh_path "$DEFAULT_SIM" "$V_SIM")"
    echo "[!!] Isaac Sim が見つからない"
    ans="$(ask "  $new に Isaac Sim $V_SIM を新しく入れますか（20〜40 分、約 33 GB）? [y/n]" y)"
    [[ "$ans" =~ ^(y|Y|yes|dry)$ ]] || { echo "[setup] やめた（ISAAC_SIM で場所を指定して、もう一度実行できる）"; exit 1; }
    SIM_CHOSEN="$new"; SIM_INSTALL=1
fi
ISAAC_SIM="$SIM_CHOSEN"

# ---- 3. Isaac Lab を探す -----------------------------------------------------

echo "[setup] Isaac Lab $V_BRANCH（${V_COMMIT:0:9}）を探す"
LAB_FROM_ENV=""
(( SIM_INSTALL )) || LAB_FROM_ENV="$(lab_of_env "$ISAAC_SIM/bin/python")"
declare -a LAB_CANDIDATES=()
for d in "$LAB_FROM_ENV" "${ISAACLAB:-}" "$DEFAULT_LAB" "$HOME"/NVIDIA/IsaacLab*/ "$HOME"/IsaacLab*/ "$HOME"/*/IsaacLab*/; do
    d="${d%/}"
    [[ -n "$d" && -d "$d/.git" && -d "$d/source/isaaclab" ]] || continue
    [[ " ${LAB_CANDIDATES[*]-} " == *" $d "* ]] && continue
    LAB_CANDIDATES+=("$d")
    echo "  見つかった: $d（$(git -C "$d" rev-parse --short=9 HEAD)）$([[ "$d" == "$LAB_FROM_ENV" ]] && echo "  ← Isaac Sim の環境に入っているもの")"
done

LAB_CHOSEN=""
LAB_CLONE=0
for d in "${LAB_CANDIDATES[@]+"${LAB_CANDIDATES[@]}"}"; do
    if [[ "$(git -C "$d" rev-parse HEAD)" == "$V_COMMIT" ]]; then LAB_CHOSEN="$d"; break; fi
done
if [[ -n "$LAB_CHOSEN" && -n "$LAB_FROM_ENV" && "$LAB_CHOSEN" != "$LAB_FROM_ENV" ]]; then
    # 版の合う Isaac Lab はあるが、Isaac Sim の環境には別の Isaac Lab が入っている
    echo "[!!] Isaac Sim の環境（$ISAAC_SIM）には、別の Isaac Lab（$LAB_FROM_ENV）が入っている"
    ans="$(ask "  1) 環境に入っているほうを使う（版が違う）  2) やめる（ISAAC_SIM で別の環境を指定する）:" 1)"
    [[ "$ans" == 1 || "$ans" == dry ]] || { echo "[setup] やめた"; exit 1; }
    LAB_CHOSEN="$LAB_FROM_ENV"
fi
if [[ -n "$LAB_CHOSEN" ]]; then
    echo "[OK] Isaac Lab を使う: $LAB_CHOSEN"
else
    new="$(fresh_path "$DEFAULT_LAB" "3.0.0b2")"
    if (( ${#LAB_CANDIDATES[@]} > 0 )); then
        first="${LAB_CANDIDATES[0]}"
        echo "[!!] 版の違う Isaac Lab しか無い（${first}: $(git -C "$first" rev-parse --short=9 HEAD)、必要なのは ${V_COMMIT:0:9}）"
        ans="$(ask "  1) $new に新しく取得する  2) $first をそのまま使う（動かない恐れ）  3) やめる:" 1)"
        case "$ans" in
            1|dry) LAB_CHOSEN="$new"; LAB_CLONE=1 ;;
            2) LAB_CHOSEN="$first" ;;
            *) echo "[setup] やめた"; exit 1 ;;
        esac
    else
        echo "[!!] Isaac Lab が見つからない"
        ans="$(ask "  $new に Isaac Lab（$V_BRANCH）を取得しますか? [y/n]" y)"
        [[ "$ans" =~ ^(y|Y|yes|dry)$ ]] || { echo "[setup] やめた"; exit 1; }
        LAB_CHOSEN="$new"; LAB_CLONE=1
    fi
fi
ISAACLAB="$LAB_CHOSEN"

# Isaac Sim の環境に Isaac Lab（rsl_rl を含む）が入っているか
LAB_INSTALL=0
if (( SIM_INSTALL || LAB_CLONE )) || [[ -z "$(pkg_version "$ISAAC_SIM/bin/python" isaaclab_rl)" ]]; then
    LAB_INSTALL=1
fi
if (( LAB_INSTALL && ! SIM_INSTALL )); then
    echo "[!!] Isaac Sim の環境（$ISAAC_SIM）に、Isaac Lab（$ISAACLAB）を入れる必要がある"
    echo "     この環境をほかの人や Yada の評価が使っていると、影響が出る（torch などの版が変わることがある）"
    ans="$(ask "  入れますか? [y/n]" y)"
    [[ "$ans" =~ ^(y|Y|yes|dry)$ ]] || { echo "[setup] やめた"; exit 1; }
fi

# ---- 4. MuJoCo の評価用の Python を探す -------------------------------------

echo "[setup] MuJoCo（$V_MUJOCO_MIN 以上）と pinocchio と torch が入った Python を探す（MuJoCo での採点に使う）"
mujoco_ok() {
    "$1" - "$V_MUJOCO_MIN" <<'PY' 2>/dev/null
import sys
import importlib.metadata as m
try:
    import mujoco, pinocchio, torch, yaml  # noqa: F401
except Exception:
    sys.exit(1)
need = tuple(int(x) for x in sys.argv[1].split("."))
have = tuple(int(x) for x in mujoco.__version__.split(".")[:len(need)])
print(mujoco.__version__)
sys.exit(0 if have >= need else 1)
PY
}
MUJOCO_CHOSEN=""
MUJOCO_CREATE=0
declare -a MJ_CANDIDATES=()
(( SIM_INSTALL )) || MJ_CANDIDATES+=("$ISAAC_SIM/bin/python")
MJ_CANDIDATES+=("${MUJOCO_PYTHON:-}" "$DEFAULT_MUJOCO_ENV/bin/python" "$REPO_ROOT/G1_HuggingFace/venv/bin/python")
for d in "$HOME"/miniconda3/envs/*/ "$HOME"/anaconda3/envs/*/ "$HOME"/miniforge3/envs/*/; do MJ_CANDIDATES+=("${d%/}/bin/python"); done
for p in "${MJ_CANDIDATES[@]}"; do
    [[ -n "$p" && -x "$p" ]] || continue
    if v="$(mujoco_ok "$p")"; then
        MUJOCO_CHOSEN="$p"
        echo "[OK] MuJoCo $v の Python を使う: $p"
        break
    fi
done
if (( SIM_INSTALL )) && [[ -z "$MUJOCO_CHOSEN" ]]; then
    # 新しく入れる Isaac Sim の環境には mujoco と pinocchio も入る（Isaac Lab の依存）。入れたあとで確かめる
    MUJOCO_CHOSEN="$ISAAC_SIM/bin/python"
    echo "[setup] MuJoCo は、新しく入れる Isaac Sim の環境のものを使う（入れたあとで確かめる）"
elif [[ -z "$MUJOCO_CHOSEN" ]]; then
    echo "[!!] MuJoCo の評価に使える Python が見つからない"
    ans="$(ask "  $DEFAULT_MUJOCO_ENV に MuJoCo 用の環境を新しく作りますか（数分、約 3 GB）? [y/n]" y)"
    [[ "$ans" =~ ^(y|Y|yes|dry)$ ]] || { echo "[setup] MuJoCo の環境は作らない（MuJoCo での採点はできない）"; }
    [[ "$ans" =~ ^(y|Y|yes|dry)$ ]] && { MUJOCO_CHOSEN="$DEFAULT_MUJOCO_ENV/bin/python"; MUJOCO_CREATE=1; }
fi
MUJOCO_PYTHON="$MUJOCO_CHOSEN"

# ---- まとめ ------------------------------------------------------------------

echo
echo "[setup] 使うもの:"
echo "  Isaac Sim : $ISAAC_SIM$( (( SIM_INSTALL )) && echo "（新しく入れる）")"
echo "  Isaac Lab : $ISAACLAB$( (( LAB_CLONE )) && echo "（新しく取得する）")$( (( LAB_INSTALL )) && echo "（Isaac Sim の環境に入れる）")"
echo "  MuJoCo    : ${MUJOCO_PYTHON:-（使わない）}$( (( MUJOCO_CREATE )) && echo "（新しく作る）")"
if (( DRY_RUN )); then
    echo "[setup] --dry-run なので、ここで終わる（何も入れていない・記録していない）"
    exit 0
fi

# ---- 5. 入れる ---------------------------------------------------------------

if (( SIM_INSTALL )); then
    command -v "python$V_PY" >/dev/null || { echo "[NG] python$V_PY が無い（sudo apt install python$V_PY python$V_PY-venv）" >&2; exit 1; }
    echo "[setup] Isaac Sim を入れる（20〜40 分）: $ISAAC_SIM"
    "python$V_PY" -m venv "$ISAAC_SIM"
    "$ISAAC_SIM/bin/python" -m pip install --quiet --upgrade pip
    "$ISAAC_SIM/bin/python" -m pip install "isaacsim[all,extscache]==$V_SIM" --extra-index-url https://pypi.nvidia.com
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

if (( LAB_CLONE )); then
    echo "[setup] Isaac Lab を取得する: $ISAACLAB"
    git clone --branch "$V_BRANCH" https://github.com/isaac-sim/IsaacLab.git "$ISAACLAB"
    git -C "$ISAACLAB" checkout --quiet "$V_COMMIT"
fi
if (( LAB_INSTALL )); then
    echo "[setup] Isaac Lab を入れる（torch $(ver torch) と rsl_rl を含む）"
    # isaaclab.sh は、有効にした venv の Python に入れる。torch は CUDA 12.8 版（cu128）を入れる
    (cd "$ISAACLAB" && VIRTUAL_ENV="$ISAAC_SIM" PATH="$ISAAC_SIM/bin:$PATH" ./isaaclab.sh --install 'rl[rsl-rl]')
fi
# ⚠️ rsl-rl-lib や torch を pip install で単独に入れ直さないこと。依存で torch が PyPI の版（CUDA 13 版）に
#    入れ替わり、ドライバ 570 では GPU で動かなくなる。版が違えば check_setup.sh が知らせるので、手で直す

if (( MUJOCO_CREATE )); then
    echo "[setup] MuJoCo 用の環境を作る: $DEFAULT_MUJOCO_ENV"
    command -v "python$V_PY" >/dev/null || { echo "[NG] python$V_PY が無い" >&2; exit 1; }
    "python$V_PY" -m venv "$DEFAULT_MUJOCO_ENV"
    "$DEFAULT_MUJOCO_ENV/bin/python" -m pip install --quiet --upgrade pip
    # torch は CPU 版（評価用の Agent の推論だけに使う）。ほかは J1-gen の requirements と同じ
    "$DEFAULT_MUJOCO_ENV/bin/python" -m pip install torch --index-url https://download.pytorch.org/whl/cpu
    "$DEFAULT_MUJOCO_ENV/bin/python" -m pip install -r "$REPO_ROOT/Button_Press/J1-gen/requirements.txt" scipy pillow
fi
if [[ -n "$MUJOCO_PYTHON" ]] && ! mujoco_ok "$MUJOCO_PYTHON" >/dev/null; then
    echo "[!!] $MUJOCO_PYTHON では MuJoCo の採点に要るもの（mujoco $V_MUJOCO_MIN 以上・pinocchio・torch）が import できない" >&2
fi

# ---- 6. G1 の公式モデル（J1-gen と共通） ---------------------------------------

if [[ ! -f "$REPO_ROOT/_local/button_press/models/g1_description/g1_29dof_rev_1_0.urdf" ]]; then
    bash "$REPO_ROOT/Button_Press/J1-gen/sim/fetch_models.sh"
fi

record
echo "[setup] 入れ終わった。確かめる:"
ISAAC_SIM="$ISAAC_SIM" ISAACLAB="$ISAACLAB" MUJOCO_PYTHON="$MUJOCO_PYTHON" bash "$SCRIPT_DIR/check_setup.sh"
