"""段 1（2026-10-26）: 公式の G1 のモデル（g1_29dof_rev_1_0.urdf）から、脚 12 関節の運動学と質量を npz に書き出す（Mac・mujoco）。

PC2 には MuJoCo が無い（Python 3.8・numpy 1.24 だけ）。door_foot_load.py はこの npz と numpy だけで、HOLD の間に
両足が床から受けている力を静力学で逆算する。

作り方:
  1. URDF から見た目の形（<visual>）とメッシュの当たりの形を外す（質量・慣性・関節の位置と軸は URDF の文字に全部ある）
     残る当たりの形は足の裏の球 8 個と腕の円柱 4 個だけ（作り物の立ち姿のシムで足の裏を使う）
  2. MuJoCo で読み、pelvis に浮いた基底（freejoint）を足してコンパイルする（URDF の既定どおり fusestatic:
     固定関節の部品 ＝ 頭・ゴムの手・IMU・ライダーなどは親に合わさる。脚には固定関節が無い）
  3. 脚 12 関節（SDK の順: 左 0〜5・右 6〜11）について、剛体の親からの位置・向き、関節の位置・軸（剛体の座標）、
     剛体の質量・重心・慣性の向き、全体の質量と重力を npz に書く。出典と作り方は meta（JSON の文字列）に入れる

モデルは g1_29dof_rev_1_0（機体は mode_machine=5）。10-25 に使った unitree_mujoco の g1_29dof.xml（35.11 kg）とは、
脚の部品は同じで、全体の質量だけが違う（rev_1_0 は 33.34 kg）。

使い方（Mac）:
  cd G1_Hackason && Navigation/.venv/bin/python Door/real/export_leg_model.py
  （--urdf・--out で場所を変える。既定の出力は Door/real/leg_model_g1_29dof_rev_1_0.npz）
"""
from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from typing import Dict, Tuple

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROBOT = "g1_29dof_rev_1_0"
URDF_DEFAULT = HERE.parent / "doorman6" / "usd_fix" / "urdf_src" / f"{ROBOT}.urdf"
OUT_DEFAULT = HERE / f"leg_model_{ROBOT}.npz"
# 取得元（usd_fix/build_fixed_usd.py の URDF_COMMIT と同じ写し。sha256 は書き出す時に計算して入れる）
SOURCE_REPO = "https://github.com/unitreerobotics/unitree_ros"
SOURCE_PATH = f"robots/g1_description/{ROBOT}.urdf"
SOURCE_COMMIT = "7c40519e02d7dd16c06b25fe3fa3b67fdeb7cd74"

# SDK の関節の順（rt/lowstate の motor_state 0〜11）
LEG_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
)
FOOT_BODIES = ("left_ankle_roll_link", "right_ankle_roll_link")
ROOT_BODY = "pelvis"


def strip_urdf_meshes(urdf_text: str) -> str:
    """見た目の形とメッシュの当たりの形・<mujoco> の指定を外した URDF の文字（メッシュのファイルが無くても読める）"""
    root = ET.fromstring(urdf_text)
    for mj in root.findall("mujoco"):
        root.remove(mj)
    for link in root.findall("link"):
        for el in link.findall("visual"):
            link.remove(el)
        for el in link.findall("collision"):
            if el.find("geometry/mesh") is not None:
                link.remove(el)
    return ET.tostring(root, encoding="unicode")


def build_spec(urdf_path: Path) -> mujoco.MjSpec:
    """公式の URDF → pelvis に浮いた基底を足した MjSpec（コンパイル前。シムで床を足すときに使う）"""
    spec = mujoco.MjSpec.from_string(strip_urdf_meshes(Path(urdf_path).read_text(encoding="utf-8")))
    root = spec.worldbody.first_body()
    if root.name != ROOT_BODY:
        raise ValueError(f"根の剛体が {root.name}（{ROOT_BODY} のはず）")
    root.add_freejoint()
    return spec


def build_model(urdf_path: Path) -> mujoco.MjModel:
    m = build_spec(urdf_path).compile()
    check_joint_order(m)
    return m


def check_joint_order(m: mujoco.MjModel) -> None:
    """qpos の 7 番目から SDK の順（脚 12 → 腰 3 → 腕 14）に並んでいるか。違えば止める"""
    if m.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError("先頭の関節が浮いた基底でない")
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(1, m.njnt)]
    if len(names) != 29 or tuple(names[:12]) != LEG_JOINTS:
        raise ValueError(f"関節の順が SDK と違う: {names[:12]}")
    for j in range(1, m.njnt):
        if m.jnt_qposadr[j] != 6 + j or m.jnt_dofadr[j] != 5 + j:
            raise ValueError(f"関節 {names[j - 1]} の qpos・dof の位置がずれている")


def mujoco_foot_wrenches(m: mujoco.MjModel, d: mujoco.MjData, q: np.ndarray, tau: np.ndarray,
                         quat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """MuJoCo での逆算（10-25 の foot_load.py と同じ式。試験で numpy の版と突き合わせる）

    脚 6 関節の釣り合い qfrc_bias[脚] = tau[脚] + J_足ᵀ W を解く。W は ankle_roll_link の原点に働く
    [Fx, Fy, Fz, Mx, My, Mz]（世界の向き）。quat は pelvis の IMU（w, x, y, z）
    """
    d.qpos[:] = 0.0
    d.qpos[3:7] = quat
    d.qpos[7:] = q
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)
    out = []
    for side, body in enumerate(FOOT_BODIES):
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        mujoco.mj_jacBody(m, d, jacp, jacr, bid)
        cols = slice(6 + 6 * side, 12 + 6 * side)
        jac = np.vstack([jacp[:, cols], jacr[:, cols]])
        rhs = d.qfrc_bias[cols] - np.asarray(tau, dtype=float)[6 * side:6 * side + 6]
        out.append(np.linalg.solve(jac.T, rhs))
    return out[0], out[1]


def leg_arrays(m: mujoco.MjModel) -> Dict[str, np.ndarray]:
    """脚 12 関節（SDK の順）の剛体と関節の値。脚が pelvis から足まで枝なしの 1 本の鎖であることも確かめる"""
    check_joint_order(m)
    root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, ROOT_BODY)
    bodies = []
    for k, jname in enumerate(LEG_JOINTS):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if m.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError(f"{jname} が回転関節でない")
        bid = int(m.jnt_bodyid[jid])
        if m.body_jntnum[bid] != 1:
            raise ValueError(f"{jname} の剛体に関節が {m.body_jntnum[bid]} 個")
        parent = int(m.body_parentid[bid])
        want_parent = root if k % 6 == 0 else bodies[-1]
        if parent != want_parent:
            raise ValueError(f"{jname} の剛体の親が鎖の前の剛体でない")
        bodies.append(bid)
    for side, foot in enumerate(FOOT_BODIES):
        chain = bodies[6 * side:6 * side + 6]
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, foot) != chain[-1]:
            raise ValueError(f"鎖の先が {foot} でない")
        # 枝なし: 股 pitch の剛体から先の質量の合計 = 鎖の 6 個の質量の合計
        if not np.isclose(m.body_subtreemass[chain[0]], m.body_mass[chain].sum()):
            raise ValueError(f"{foot} の脚に枝がある（質量の合計が合わない）")
    jids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in LEG_JOINTS]
    return dict(
        joint_names=np.array(LEG_JOINTS),
        body_names=np.array([mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in bodies]),
        body_pos=m.body_pos[bodies].copy(),  # [12, 3] 親の剛体の座標（先頭の親は pelvis）
        body_quat=m.body_quat[bodies].copy(),  # [12, 4] w x y z
        jnt_pos=m.jnt_pos[jids].copy(),  # [12, 3] 剛体の座標
        jnt_axis=m.jnt_axis[jids].copy(),  # [12, 3] 剛体の座標
        body_mass=m.body_mass[bodies].copy(),  # [12]
        body_ipos=m.body_ipos[bodies].copy(),  # [12, 3] 剛体の座標での重心
        body_iquat=m.body_iquat[bodies].copy(),  # [12, 4] 慣性の主軸の向き（自重の計算には使わない）
        body_inertia=m.body_inertia[bodies].copy(),  # [12, 3] 主軸まわりの慣性（同上）
        total_mass=np.array(float(m.body_subtreemass[root])),
        gravity=m.opt.gravity.copy(),
    )


def _repo_relative(path: Path) -> str:
    """G1_Hackason からの相対の場所（外なら絶対の場所）"""
    p = Path(path).resolve()
    try:
        return str(p.relative_to(HERE.parents[1]))
    except ValueError:
        return str(p)


def make_meta(urdf_path: Path) -> dict:
    raw = Path(urdf_path).read_bytes()
    return dict(
        robot=ROBOT,
        source_repo=SOURCE_REPO,
        source_path=SOURCE_PATH,
        source_commit=SOURCE_COMMIT,
        urdf_file=_repo_relative(urdf_path),
        urdf_sha256=hashlib.sha256(raw).hexdigest(),
        made_by="Door/real/export_leg_model.py",
        made_at=datetime.now().isoformat(timespec="seconds"),
        mujoco=mujoco.__version__,
        how=("URDF の <visual> とメッシュの <collision> を外し、pelvis に freejoint を足して MuJoCo でコンパイル"
             "（fusestatic: 固定関節の部品は親に合わさる）。脚 12 関節の剛体・関節の値をそのまま写した"),
        joint_order="SDK の順。0〜5 左の股 pitch・roll・yaw・膝・足首 pitch・roll、6〜11 右",
        foot_point="ankle_roll_link の原点（足の 6 次元の力はこの点のまわり・世界の向き。鉛直の力は点に依らない）",
        imu="rt/lowstate の imu_state（pelvis の IMU。imu_in_pelvis は回転なし）。四元数 w x y z を pelvis の向きに使う",
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", type=Path, default=URDF_DEFAULT)
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    a = ap.parse_args()
    m = build_model(a.urdf)
    arrays = leg_arrays(m)
    meta = make_meta(a.urdf)
    np.savez(a.out, meta=np.array(json.dumps(meta, ensure_ascii=False)), **arrays)
    legs = arrays["body_mass"].reshape(2, 6).sum(1)
    print(f"[export_leg_model] {a.out}: 全体 {float(arrays['total_mass']):.3f} kg・脚 左 {legs[0]:.3f} 右 {legs[1]:.3f} kg"
          f"・URDF {meta['urdf_sha256'][:12]}（{SOURCE_COMMIT[:7]}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
