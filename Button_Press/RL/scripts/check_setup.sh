#!/usr/bin/env bash
# 学習の環境が configs/versions.yaml の版でそろっているかを確かめる（数秒。Isaac Sim は起動しない）。
#
#   bash Button_Press/RL/scripts/check_setup.sh
#   ISAAC_SIM=/path/to/venv ISAACLAB=/path/to/IsaacLab bash Button_Press/RL/scripts/check_setup.sh
#
# すべて [OK] なら、次は動作の確認: bash Button_Press/RL/scripts/train.sh --num_envs 16 --max_iterations 3
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$RL_DIR/../.." && pwd)"
source "$SCRIPT_DIR/env.sh" >/dev/null 2>&1 || true
ISAAC_SIM="${ISAAC_SIM:-$HOME/NVIDIA/env_isaaclab}"
ISAACLAB="${ISAACLAB:-$HOME/NVIDIA/IsaacLab}"
PY="$ISAAC_SIM/bin/python"
NG=0
ok() { echo "[OK] $*"; }
ng() { echo "[NG] $*"; NG=$((NG + 1)); }
want() { grep "^$1:" "$RL_DIR/configs/versions.yaml" | sed 's/^[^:]*: *"\{0,1\}\([^"]*\)"\{0,1\}.*/\1/'; }

DRIVER="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)"
GPU="$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null | head -1)"
if [[ -n "$DRIVER" ]] && (( ${DRIVER%%.*} >= $(want min_driver) )); then ok "GPU: $GPU、ドライバ $DRIVER"; else ng "NVIDIA ドライバ: ${DRIVER:-無い}（$(want min_driver) 以上が要る）"; fi

[[ -x "$ISAAC_SIM/python.sh" ]] && ok "Isaac Sim の python.sh: $ISAAC_SIM/python.sh" || ng "Isaac Sim の python.sh が無い: $ISAAC_SIM（ISAAC_SIM を指定するか、setup.sh を実行する）"
for pkg in isaacsim:isaacsim rsl-rl-lib:rsl_rl torch:torch; do
    name="${pkg%%:*}"; key="${pkg##*:}"
    out="$("$PY" -m pip show "$name" 2>/dev/null || true)"
    have="$(sed -n 's/^Version: //p' <<<"$out")"
    [[ "$have" == "$(want "$key")" ]] && ok "$name $have" || ng "$name ${have:-無い}（$(want "$key") が要る）"
done
head="$(git -C "$ISAACLAB" rev-parse HEAD 2>/dev/null)"
[[ "$head" == "$(want isaaclab_commit)" ]] && ok "Isaac Lab ${head:0:9}（$(want isaaclab_branch)）" || ng "Isaac Lab の版: ${head:-無い}（$(want isaaclab_commit) が要る）"
out="$("$PY" -m pip show isaaclab_rl 2>/dev/null || true)"; [[ -n "$out" ]] && ok "isaaclab_rl が入っている" || ng "isaaclab_rl が無い（setup.sh を実行する）"
[[ -f "$REPO_ROOT/_local/button_press/models/g1_description/g1_29dof_rev_1_0.urdf" ]] && ok "G1 の公式モデル" || ng "G1 の公式モデルが無い（bash Button_Press/J1-gen/sim/fetch_models.sh）"

# シミュレーターに依存しない部分のテスト（乱し幅・観測・報酬）
if PYTHONPATH="$RL_DIR" "$PY" -m unittest discover -s "$RL_DIR/tests" -p 'test_*.py' >/tmp/check_setup_$$.log 2>&1; then
    ok "Button_Press/RL のテスト"
else
    ng "Button_Press/RL のテストが通らない（/tmp/check_setup_$$.log）"
fi

if (( NG > 0 )); then
    echo "[check] $NG 個の問題がある"; exit 1
fi
echo "[check] すべて OK。次は動作の確認: bash Button_Press/RL/scripts/train.sh --num_envs 16 --max_iterations 3"
