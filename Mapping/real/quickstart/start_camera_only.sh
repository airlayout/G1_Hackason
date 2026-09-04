#!/usr/bin/env bash
# G1 PC2上でカメラだけを配信する。歩行modeやslam_operateには触れない。
set -euo pipefail

CONDA_SH="${HOME}/miniconda3/etc/profile.d/conda.sh"
if [[ ! -f "$CONDA_SH" ]]; then
    echo "[ERROR] Minicondaが見つかりません: $CONDA_SH" >&2
    exit 1
fi

# shellcheck disable=SC1090
set +u
source "$CONDA_SH"
conda activate lerobot
set -u

echo "[CAMERA] LeRobot ImageServerを起動します: head_camera, 640x480@30, port 5555"
echo "[CAMERA] このプロセスはカメラ以外のDDS・歩行・SLAM状態を変更しません"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python "$SCRIPT_DIR/camera_only_server.py" "$@"
