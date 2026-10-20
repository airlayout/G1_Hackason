"""G1（29DoF rev_1_0）の関節の並びと、MuJoCo モデルの読み込み。

用語:
- MJCF: MuJoCo 用のモデル記述（XML）。URDF と同じく、リンクと関節のつながりを書いたもの。
- MjSpec: MJCF を読み込んだあと、コンパイル前にプログラムから編集できる MuJoCo の仕組み。
  公式 XML ファイルを書き換えずに、頭カメラの追加や胴体の固定ができる。
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from .config import resolve_repo_path

# 関節名。並びは lowcmd / lowstate の motor_cmd[i] / motor_state[i] の番号（0〜28）と同じ。
# 公式 XML の関節の並び（floating_base_joint を除く）とも一致する（tests で確認）。
JOINT_NAMES: tuple[str, ...] = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
NUM_MOTORS: int = 29
WAIST_IDX: tuple[int, ...] = (12, 13, 14)
LEFT_ARM_IDX: tuple[int, ...] = tuple(range(15, 22))
RIGHT_ARM_IDX: tuple[int, ...] = tuple(range(22, 29))
# arm_sdk で weight を入れる motor_cmd の番号
ARM_SDK_WEIGHT_IDX: int = 29

FLOATING_BASE_JOINT: str = "floating_base_joint"


def rpy_to_quat_wxyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF の RPY（x→y→z の固定軸回転）を MuJoCo の四元数 (w, x, y, z) に変換する。"""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def build_spec(robot_cfg: dict[str, Any], fixed_base: bool = True) -> Any:
    """公式 XML を読み込み、頭カメラを追加した MjSpec を返す。

    fixed_base=True のときは floating_base_joint を消して、pelvis をワールドに固定する
    （腕の IK と軌道の確認用。歩行コントローラはシミュレーションに無いため）。
    """
    import mujoco

    mjcf_path = resolve_repo_path(robot_cfg["model"]["mjcf_path"])
    if not mjcf_path.exists():
        raise FileNotFoundError(
            f"公式モデルが無い: {mjcf_path}\n"
            "先に `bash Button_Press/J1-gen/sim/fetch_models.sh` を実行すること。"
        )
    spec = mujoco.MjSpec.from_file(str(mjcf_path))

    if fixed_base:
        spec.delete(spec.joint(FLOATING_BASE_JOINT))

    cam_cfg = robot_cfg["head_camera"]
    parent = spec.body(cam_cfg["parent_body"])
    if parent is None:
        raise ValueError(f"頭カメラの親リンクが XML に無い: {cam_cfg['parent_body']}")
    # URDF の d435_link に相当する body を作り、その中にカメラを置く。
    mount = parent.add_body(name="d435_link")
    mount.pos = list(cam_cfg["xyz"])
    mount.quat = list(rpy_to_quat_wxyz(*cam_cfg["rpy"]))
    # MuJoCo のカメラは -z 方向を見て、+y が画像の上、+x が画像の右。
    # d435_link（x 前方・y 左・z 上）で前方を見るよう、カメラの x 軸 = link の -y、
    # カメラの y 軸 = link の +z にする。
    cam = mount.add_camera(name=cam_cfg["name"])
    cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    cam.alt.xyaxes = [0.0, -1.0, 0.0, 0.0, 0.0, 1.0]
    cam.fovy = float(cam_cfg["fovy_deg"])
    return spec


def load_model(robot_cfg: dict[str, Any], fixed_base: bool = True) -> Any:
    """頭カメラ付きの MjModel を返す。"""
    return build_spec(robot_cfg, fixed_base=fixed_base).compile()
