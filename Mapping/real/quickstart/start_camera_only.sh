#!/usr/bin/env bash
# G1 PC2上でカメラだけを配信する。歩行modeやslam_operateには触れない。
set -euo pipefail

# condaの配置はPC2の個体差がある。miniconda / miniforge / mambaforge を順に探す。
CONDA_SH=""
for candidate in \
    "${CONDA_ROOT:-}/etc/profile.d/conda.sh" \
    "${HOME}/miniconda3/etc/profile.d/conda.sh" \
    "${HOME}/miniforge3/etc/profile.d/conda.sh" \
    "${HOME}/mambaforge/etc/profile.d/conda.sh" \
    "${HOME}/anaconda3/etc/profile.d/conda.sh" \
    "/opt/miniconda3/etc/profile.d/conda.sh"; do
    if [[ -f "$candidate" ]]; then
        CONDA_SH="$candidate"
        break
    fi
done

if [[ -z "$CONDA_SH" ]]; then
    echo "[ERROR] condaが見つかりません。CONDA_ROOTで場所を指定してください" >&2
    exit 1
fi
echo "[CAMERA] conda: $CONDA_SH"

# shellcheck disable=SC1090
set +u
source "$CONDA_SH"
conda activate lerobot
set -u

echo "[CAMERA] LeRobot ImageServerを起動します: head_camera, 640x480@30, port 5555"
echo "[CAMERA] このプロセスはカメラ以外のDDS・歩行・SLAM状態を変更しません"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python "$SCRIPT_DIR/camera_only_server.py" "$@"
