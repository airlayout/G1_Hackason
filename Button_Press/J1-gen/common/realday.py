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


OBSTACLES = CONFIG_DIR / "obstacles.yaml"


EDGE_TILT_WARN_DEG = 10.0


def box_from_touch_points(
    points: list[np.ndarray], margin_m: float, depth_m: float, below_m: float, width_m: float = 2.0,
) -> dict[str, Any]:
    """中指の先で触った机の手前の縁の点（pelvis 座標）から、障害物の箱を作る。

    ロボットは +x を向いている前提:
    - y（左右）: 触った点の真ん中から左右に width_m / 2 ずつ（触った点がもっと外にあればそこまで）。
      机が大きくて角を触れないとき、縁の上の 2 点（20〜30 cm 離れた点）だけでも机の幅を覆えるように
    - x（前後）: 手前の縁から奥へ depth_m。手前の縁は、
        1 点: その点の x（縁がロボットの正面に平行だと仮定）
        2 点以上: 触った点を通る直線（x = a y + b）を左右の端まで延ばし、一番手前になる x
                 （縁が斜めでも、延ばした先で箱が机より奥にならないように。安全側）
    - z（上下）: 一番高い点を天板の上面とし、下へ below_m（机の脚の分）
    まわりに margin_m の余裕を足す。縁の傾きが EDGE_TILT_WARN_DEG を超えたら "warning" に書く。
    """
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    if p.shape[0] < 1:
        raise ValueError("点が 1 つ以上いる（机の手前の縁の上の点）")
    y_mid = (p[:, 1].min() + p[:, 1].max()) / 2
    y_lo = min(p[:, 1].min(), y_mid - width_m / 2)
    y_hi = max(p[:, 1].max(), y_mid + width_m / 2)
    warning = ""
    tilt_deg = 0.0
    if p.shape[0] == 1 or np.ptp(p[:, 1]) < 1e-3:
        # 1 点（または左右に離れていない点）: 縁は正面に平行と仮定する
        x_front = p[:, 0].min()
        if p.shape[0] > 1:
            warning = "触った点が左右に離れていないので、縁は正面に平行だと仮定した"
    else:
        a, b = np.polyfit(p[:, 1], p[:, 0], 1)  # x = a y + b
        tilt_deg = float(np.degrees(np.arctan(a)))
        x_front = min(p[:, 0].min(), a * y_lo + b, a * y_hi + b)
        if abs(tilt_deg) > EDGE_TILT_WARN_DEG:
            warning = (f"縁がロボットの正面に対して {tilt_deg:+.1f}° 傾いている。左右に延ばした先で手前に"
                       f"せり出すので、箱の手前の縁を {x_front:+.3f} m にした（ロボットを机に正対させるとよい）")
    lo = np.array([x_front - margin_m, y_lo - margin_m, p[:, 2].max() - below_m])
    hi = np.array([max(p[:, 0].max(), x_front + depth_m) + margin_m, y_hi + margin_m, p[:, 2].max() + margin_m])
    return {"center": [round(float(v), 4) for v in (lo + hi) / 2],
            "half_size": [round(float(v), 4) for v in (hi - lo) / 2],
            "edge_tilt_deg": round(tilt_deg, 1), "warning": warning}


def load_obstacles(path: Path | str = OBSTACLES) -> list[dict[str, Any]]:
    path = Path(path)
    if not path.exists():
        return []
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return list(data.get("obstacles") or [])


def save_obstacle(name: str, box: dict[str, Any], points: list[np.ndarray], note: str = "",
                  path: Path = OBSTACLES) -> None:
    """障害物の箱を追加する（同じ名前があれば置き換える）。"""
    header = (
        "# Button_Press / 障害物の箱（real/teach.py --obstacle が書く）。pelvis 座標 [m]。\n"
        "# configs/press.yaml の obstacles に加えて、全体をつなぐスクリプトの衝突の確認に使う。\n"
        "# touch_points_m は中指の先で触った点。実機日のデータなので、終わったらコミットして残す。\n"
    )
    obs = [o for o in load_obstacles(path) if o.get("name") != name]
    obs.append({"name": name, "center": box["center"], "half_size": box["half_size"],
                "edge_tilt_deg": box.get("edge_tilt_deg", 0.0),
                "touch_points_m": [[round(float(v), 4) for v in pt] for pt in points],
                "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"), "note": note})
    path.write_text(header + yaml.safe_dump({"obstacles": obs}, allow_unicode=True, sort_keys=False), encoding="utf-8")


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
