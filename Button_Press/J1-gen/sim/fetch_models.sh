#!/usr/bin/env bash
# Unitree 公式の G1 モデル（URDF と MJCF）を _local/button_press/models/ に取り出す。
#
#   bash Button_Press/J1-gen/sim/fetch_models.sh                  # GitHub から取得する
#   bash Button_Press/J1-gen/sim/fetch_models.sh --from <path>    # 既にある unitree_ros の clone から取り出す（オフライン）
#
# なぜ取得スクリプトなのか: メッシュを含めると数十MBあり、リポジトリに入れる大きさではない。
# Navigation/sim/fetch_assets.sh と同じく「.gitignore（_local/）+ 取得スクリプト」で用意する。
#
# IK 用の URDF とシミュレーション用の XML は、**同じリポジトリ・同じコミット・同じ機体構成**
# （g1_29dof_rev_1_0 = mode_machine 5）からそろえて取る。版がずれると、IK で計算した
# 手先位置とシミュレーション上の手先位置が食い違うため。
#
# ライセンス: unitree_ros は BSD-3-Clause。取り出したものは配布せず各自が取得する。

set -euo pipefail

# 取得元。**コミットで固定する。**（2026-09-28 時点の master）
# 変えるときは configs/robot.yaml の model.commit も同じ値にそろえること。
REPO_URL="https://github.com/unitreerobotics/unitree_ros.git"
REPO_COMMIT="ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee"
MODEL_NAME="g1_29dof_rev_1_0"
SUBDIR="robots/g1_description"

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SIM_DIR}/../../.." && pwd)"
DEST_DIR="${REPO_ROOT}/_local/button_press/models/g1_description"

SOURCE_DIR=""
if [[ "${1:-}" == "--from" ]]; then
    SOURCE_DIR="${2:?--from にはローカルの unitree_ros のパスが要る}"
    if [[ ! -f "${SOURCE_DIR}/${SUBDIR}/${MODEL_NAME}.urdf" ]]; then
        echo "error: ${SOURCE_DIR} は unitree_ros の clone ではない（${MODEL_NAME}.urdf が無い）" >&2
        exit 1
    fi
fi

cleanup() {
    if [[ -n "${TMP_DIR:-}" && -d "${TMP_DIR}" ]]; then rm -rf "${TMP_DIR}"; fi
}
trap cleanup EXIT

if [[ -z "${SOURCE_DIR}" ]]; then
    TMP_DIR="$(mktemp -d)"
    echo "[fetch_models] unitree_ros を取得する（${REPO_COMMIT:0:7}、g1_description のみ）..."
    # 履歴を持たずに 1 コミットだけ、必要なフォルダだけを取る（sparse checkout）。
    git -C "${TMP_DIR}" init --quiet
    git -C "${TMP_DIR}" remote add origin "${REPO_URL}"
    git -C "${TMP_DIR}" sparse-checkout set "${SUBDIR}"
    git -C "${TMP_DIR}" fetch --quiet --depth 1 --filter=blob:none origin "${REPO_COMMIT}"
    git -C "${TMP_DIR}" checkout --quiet FETCH_HEAD
    SOURCE_DIR="${TMP_DIR}"
fi

SRC="${SOURCE_DIR}/${SUBDIR}"
echo "[fetch_models] 取り出し先: ${DEST_DIR}"
rm -rf "${DEST_DIR}"
mkdir -p "${DEST_DIR}/meshes"

cp "${SRC}/${MODEL_NAME}.urdf" "${SRC}/${MODEL_NAME}.xml" "${SRC}/README.md" "${DEST_DIR}/"

# URDF と XML が参照するメッシュだけをコピーする（両方の和集合）。
mapfile -t MESHES < <(
    {
        grep -o 'filename="[^"]*"' "${SRC}/${MODEL_NAME}.urdf" | sed 's/filename="//; s/"$//; s|^.*/||'
        grep -o 'file="[^"]*"' "${SRC}/${MODEL_NAME}.xml" | sed 's/file="//; s/"$//; s|^.*/||'
    } | sort -u
)
for mesh in "${MESHES[@]}"; do
    cp "${SRC}/meshes/${mesh}" "${DEST_DIR}/meshes/${mesh}"
done

cat > "${DEST_DIR}/SOURCE.txt" <<EOF
${REPO_URL}
commit ${REPO_COMMIT}
model ${MODEL_NAME}
BSD-3-Clause

Button_Press/J1-gen/sim/fetch_models.sh が取り出したもの。手で編集しない。
頭カメラは XML に無いため、読み込み時に configs/robot.yaml の値で追加する（common/robot_model.py）。
EOF

echo "[fetch_models] ${MODEL_NAME}.urdf / .xml、メッシュ ${#MESHES[@]} 個（$(du -sh "${DEST_DIR}" | cut -f1)）"
echo "[fetch_models] 完了"
