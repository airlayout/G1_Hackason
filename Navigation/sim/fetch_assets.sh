#!/usr/bin/env bash
# unitree_rl_gym から、sim の歩行に要るものだけを Navigation/sim/assets/ に取り出す。
#
#   bash Navigation/sim/fetch_assets.sh                  # GitHub から浅く clone して取り出す
#   bash Navigation/sim/fetch_assets.sh --from <path>    # 既にある clone から取り出す（オフライン）
#
# なぜスクリプトなのか: 取り出すものは 24MB（メッシュ）+ 146KB（ポリシー）で、
# リポジトリに入れる大きさではない。`sim/maps/` と同じく **.gitignore + 取得スクリプト**で
# 用意する方針に揃える。clone 全体は 195MB あるが、12DoF に要らない
# 23dof/29dof/dual_arm のメッシュは持ってこない。
#
# ライセンス: unitree_rl_gym は BSD-3-Clause。取り出したものは配布せず各自が取得する。

set -euo pipefail

# 取得元。**コミットで固定する。** ポリシーと MJCF と config の 3 つが噛み合っている
# 必要があり、上流が動くと歩かなくなるため。
REPO_URL="https://github.com/unitreerobotics/unitree_rl_gym.git"
REPO_COMMIT="276801e46c5d433564f24658bac64f254b7d2d4b"

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASSET_DIR="${SIM_DIR}/assets"

SOURCE_DIR=""
if [[ "${1:-}" == "--from" ]]; then
    SOURCE_DIR="${2:?--from にはローカルの unitree_rl_gym のパスが要る}"
    if [[ ! -f "${SOURCE_DIR}/deploy/pre_train/g1/motion.pt" ]]; then
        echo "error: ${SOURCE_DIR} は unitree_rl_gym の clone ではない（motion.pt が無い）" >&2
        exit 1
    fi
fi

# 12DoF のポリシーが参照するメッシュだけ（`grep file= g1_12dof.xml` で確かめた 27 個）。
# 上半身は動かないが、質量と衝突形状として要るので落とせない。
MESHES=(
    pelvis.STL pelvis_contour_link.STL logo_link.STL head_link.STL
    torso_link_23dof_rev_1_0.STL
    left_hip_pitch_link.STL left_hip_roll_link.STL left_hip_yaw_link.STL
    left_knee_link.STL left_ankle_pitch_link.STL left_ankle_roll_link.STL
    right_hip_pitch_link.STL right_hip_roll_link.STL right_hip_yaw_link.STL
    right_knee_link.STL right_ankle_pitch_link.STL right_ankle_roll_link.STL
    left_shoulder_pitch_link.STL left_shoulder_roll_link.STL left_shoulder_yaw_link.STL
    left_elbow_link.STL left_wrist_roll_rubber_hand.STL
    right_shoulder_pitch_link.STL right_shoulder_roll_link.STL right_shoulder_yaw_link.STL
    right_elbow_link.STL right_wrist_roll_rubber_hand.STL
)

cleanup() {
    if [[ -n "${TMP_DIR:-}" && -d "${TMP_DIR}" ]]; then rm -rf "${TMP_DIR}"; fi
}
trap cleanup EXIT

if [[ -z "${SOURCE_DIR}" ]]; then
    TMP_DIR="$(mktemp -d)"
    echo "unitree_rl_gym を取得する（${REPO_COMMIT:0:7}）..."
    # 履歴を持たずに 1 コミットだけ取る。`clone --depth 1` は SHA を直接指定できないので
    # init + fetch にする。
    git -C "${TMP_DIR}" init --quiet
    git -C "${TMP_DIR}" remote add origin "${REPO_URL}"
    git -C "${TMP_DIR}" fetch --quiet --depth 1 origin "${REPO_COMMIT}"
    git -C "${TMP_DIR}" checkout --quiet FETCH_HEAD
    SOURCE_DIR="${TMP_DIR}"
fi

echo "取り出し先: ${ASSET_DIR}"
mkdir -p "${ASSET_DIR}/g1_description/meshes"

cp "${SOURCE_DIR}/deploy/pre_train/g1/motion.pt"          "${ASSET_DIR}/motion.pt"
cp "${SOURCE_DIR}/deploy/deploy_mujoco/configs/g1.yaml"   "${ASSET_DIR}/g1.yaml"
cp "${SOURCE_DIR}/resources/robots/g1_description/g1_12dof.xml" "${ASSET_DIR}/g1_description/"
cp "${SOURCE_DIR}/resources/robots/g1_description/README.md"    "${ASSET_DIR}/g1_description/"
for mesh in "${MESHES[@]}"; do
    cp "${SOURCE_DIR}/resources/robots/g1_description/meshes/${mesh}" \
       "${ASSET_DIR}/g1_description/meshes/${mesh}"
done

cat > "${ASSET_DIR}/SOURCE.txt" <<EOF
${REPO_URL}
commit ${REPO_COMMIT}
BSD-3-Clause

Navigation/sim/fetch_assets.sh が取り出したもの。手で編集しない。
scene.xml は持ってこない（床と光源は sim/rooms.py が部屋ごとに組むため）。
EOF

echo
echo "  motion.pt              $(du -h "${ASSET_DIR}/motion.pt" | cut -f1)"
echo "  g1.yaml                $(du -h "${ASSET_DIR}/g1.yaml" | cut -f1)"
echo "  g1_description/        $(du -sh "${ASSET_DIR}/g1_description" | cut -f1)（メッシュ ${#MESHES[@]} 個）"
echo
echo "完了。動作確認:"
echo "  cd Navigation && uv run --group mujoco --group walk python -m sim.g1_walker --selftest"
