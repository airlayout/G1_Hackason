"""実機日用のツール（タスク7）で共有する部品。どれも lowstate を読むだけで、何も送信しない。

- 指先の位置（FK）: lowstate の関節角 → 左右の指先の位置（pelvis 座標）
- 画像への投影: pelvis 座標の点 → 頭カメラの画素（FK とカメラの取り付け位置を、画像の上で確かめるため）
- 教えた姿勢（configs/taught_poses.yaml）の読み書き
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .camera_geometry import HeadCameraTransform
from .config import CONFIG_DIR
from .kinematics import ArmKinematics
from .rgbd_protocol import Intrinsics
from .robot_model import LEFT_ARM_IDX, RIGHT_ARM_IDX, WAIST_IDX

TAUGHT_POSES = CONFIG_DIR / "taught_poses.yaml"

SUPPORT_WARNING = (
    "腕を手で動かすには、リモコンでダンピング（L2+B）状態にする。ダンピング中は全身が脱力するので、\n"
    "必ず座った状態か、吊り下げた状態で行うこと。緊急時もリモコンを持った人がそばにいること。"
)


def confirm_support(input_fn: Any = input) -> bool:
    """腕を手で動かす前の確認。Enter で True、q で False。"""
    print(f"[support] ⚠️ {SUPPORT_WARNING}")
    ans = input_fn("[support] ロボットは座った状態、または吊り下げた状態か？ → Enter で続行 / q で中止: ")
    return ans.strip().lower() != "q"


class FingertipFK:
    """左右の指先の位置（pelvis 座標）を関節角から求める。"""

    def __init__(self, robot_cfg: dict[str, Any], ik_cfg: dict[str, Any]) -> None:
        self.kin = {s: ArmKinematics(robot_cfg, s, ik_cfg) for s in ("left", "right")}

    def positions(self, q: np.ndarray) -> dict[str, np.ndarray]:
        return {s: k.fk_pos(q) for s, k in self.kin.items()}


def project_to_image(p_pelvis: np.ndarray, q_waist: np.ndarray, transform: HeadCameraTransform,
                     intr: Intrinsics) -> tuple[float, float] | None:
    """pelvis 座標の点を頭カメラの画素 (u, v) に投影する。カメラの後ろなら None。

    較正の補正（transform.offset）は、pelvis 座標の点から引いてから投影する（to_pelvis の逆）。
    """
    r, p = transform.pelvis_from_optical(q_waist)
    x = r.T @ (np.asarray(p_pelvis, dtype=float) - transform.offset - p)
    if x[2] <= 1e-6:
        return None
    return intr.fx * x[0] / x[2] + intr.cx, intr.fy * x[1] / x[2] + intr.cy


def arm_q(q: np.ndarray, side: str) -> np.ndarray:
    return np.asarray(q, dtype=float)[list(RIGHT_ARM_IDX if side == "right" else LEFT_ARM_IDX)].copy()


def load_taught_poses(path: Path = TAUGHT_POSES) -> dict[str, dict[str, Any]]:
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return dict(data.get("poses", {}))


def save_taught_pose(name: str, side: str, q: np.ndarray, fingertip: np.ndarray, note: str = "",
                     path: Path = TAUGHT_POSES) -> None:
    """教えた姿勢を追加する（同じ名前があれば上書き）。ファイルの先頭の説明は残す。"""
    header = (
        "# Button_Press / ティーチングで記録した姿勢（real/teach.py が書く）。\n"
        "# 押し込み姿勢は IK の初期値（q_seed）に使う。fingertip_pelvis_m は記録したときの FK の指先の位置。\n"
        "# 実機日のデータなので、終わったらコミットして残す。\n"
    )
    poses = load_taught_poses(path)
    poses[name] = {
        "side": side,
        "arm_q_rad": [round(float(v), 6) for v in arm_q(q, side)],
        "waist_q_rad": [round(float(v), 6) for v in np.asarray(q)[list(WAIST_IDX)]],
        "fingertip_pelvis_m": [round(float(v), 5) for v in fingertip],
        "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "note": note,
    }
    body = yaml.safe_dump({"poses": poses}, allow_unicode=True, sort_keys=False)
    path.write_text(header + body, encoding="utf-8")


def taught_q_seed(name: str, q_now: np.ndarray, path: Path = TAUGHT_POSES) -> tuple[str, np.ndarray]:
    """教えた姿勢を IK の初期値（29 関節）にする。腕以外は q_now のまま。(腕の左右, q) を返す。"""
    poses = load_taught_poses(path)
    if name not in poses:
        raise KeyError(f"教えた姿勢 {name!r} が {path} に無い（あるのは {list(poses)}）")
    pose = poses[name]
    side = pose["side"]
    q = np.asarray(q_now, dtype=float).copy()
    q[list(RIGHT_ARM_IDX if side == "right" else LEFT_ARM_IDX)] = pose["arm_q_rad"]
    return side, q
