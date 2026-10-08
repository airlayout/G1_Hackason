#!/usr/bin/env bash
# G1 の全身制御（WBC）sim の専用 venv を作る。公式の setup_GR00T_WholeBodyControl.sh を呼ぶだけ。
# 公式スクリプトは git submodule と robosuite の clone を行うため、git-lfs が要る（install_git_lfs.sh）。
#
# 使い方: bash setup_wbc_sim.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"
require_groot_dir

cd "$GROOT_DIR"
git lfs version
bash gr00t/eval/sim/GR00T-WholeBodyControl/setup_GR00T_WholeBodyControl.sh
test -x "$SIM_PY" || { echo "[error] sim 用 python が見つかりません: $SIM_PY" >&2; exit 1; }
echo "[wbc] SETUP DONE ($SIM_PY)"
