#!/usr/bin/env bash
# MuJoCo で採点する（Yada の contest/evaluate.py を、setup.sh が選んだ MuJoCo の Python で動かす）。
#   bash Button_Press/RL/scripts/evaluate_mujoco.sh --agent Button_Press/RL/agent --target down --trials 20
#   bash Button_Press/RL/scripts/evaluate_mujoco.sh --agent Button_Press/RL/agent --target down --trials 40 --set realistic --report
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
cd "$REPO_ROOT"
# Isaac Lab のソースを PYTHONPATH に入れたままだと、MuJoCo の評価では要らないものを読みに行くので外す
env -u PYTHONPATH "$MUJOCO_PYTHON" Button_Press/Yada/contest/evaluate.py "$@"
