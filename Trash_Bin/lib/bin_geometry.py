"""ゴミ箱の形と置き方から、押す点・計画用の障害物を求める。sim / real 共通（MuJoCo には依存しない）。

ゴミ箱の置き方（BinPose）は、上から見た中心 center_xy（pelvis 座標 [m]）と向き yaw [rad] で表す。
ゴミ箱の座標（以下「箱の座標」）: 原点は上から見た中心、x は正面（ロボット側）から背面へ向かう向き、y は左、
z は pelvis 座標の z と同じ（高さは設定では床から書き、pelvis_height_m を引いて pelvis 座標に直す）。
yaw = 0 なら、箱の座標の x が pelvis の x（前方）と同じ向き（ロボットがゴミ箱の正面に向いて立っている）。

置き方は、シミュレーションでは設定（configs/bin_scene.yaml の bin.front_x / center_y / yaw_deg）から、
実機では頭カメラの深度から（lib/locate_bin.py）求める。押す点と障害物は、どちらでも同じ関数で求める。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class BinPose:
    center_xy: np.ndarray  # 上から見た中心（pelvis 座標 [m]）
    yaw: float             # 箱の座標の x の向き（pelvis の x から左回り [rad]）

    def rot(self) -> np.ndarray:
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def to_pelvis(self, p_local: np.ndarray) -> np.ndarray:
        """箱の座標の点 → pelvis 座標の点。"""
        return self.rot() @ np.asarray(p_local, dtype=float) + np.array([self.center_xy[0], self.center_xy[1], 0.0])

    def to_local(self, p_pelvis: np.ndarray) -> np.ndarray:
        """pelvis 座標の点（N×3 でもよい）→ 箱の座標の点。"""
        p = np.asarray(p_pelvis, dtype=float) - np.array([self.center_xy[0], self.center_xy[1], 0.0])
        return p @ self.rot()


def pose_from_config(cfg: dict[str, Any]) -> BinPose:
    """設定（bin.front_x / center_y / yaw_deg）の置き方。front_x は yaw = 0 のときの正面の x。"""
    b = cfg["bin"]
    return BinPose(center_xy=np.array([float(b["front_x"]) + float(b["depth"]) / 2, float(b["center_y"])]),
                   yaw=float(np.radians(float(b.get("yaw_deg", 0.0)))))


def local_geometry(cfg: dict[str, Any]) -> dict[str, float]:
    """箱の座標での寸法（x0/x1: 正面/背面、ya/yb: 右/左の側面、o*: 開口の縁、z*: 高さ）。"""
    b = cfg["bin"]
    ph = float(cfg["pelvis_height_m"])
    d, w = float(b["depth"]), float(b["width"])
    od, ow = float(b["opening_depth"]), float(b["opening_width"])
    return {
        "x0": -d / 2, "x1": d / 2, "ya": -w / 2, "yb": w / 2,
        "ox0": -od / 2, "ox1": od / 2, "oy0": -ow / 2, "oy1": ow / 2,
        "z_floor": -ph, "z_top": float(b["height"]) - ph, "z_open": float(b["opening_z"]) - ph,
    }


def push_point_local(cfg: dict[str, Any], side: str = "right") -> np.ndarray:
    """押す点（閉じたフラップの上面の点）の箱の座標。

    フラップのどちらの半分を押すかは push.half（right / left）。書いていなければ腕の側（右腕なら右 = −y）。
    """
    g = local_geometry(cfg)
    p = cfg["push"]
    half = p.get("half") or side
    sgn = -1.0 if half == "right" else 1.0
    return np.array([g["ox0"] + float(p["inset_m"]), sgn * float(p["from_axis_m"]), g["z_open"]])


def push_point(cfg: dict[str, Any], side: str = "right", pose: BinPose | None = None) -> np.ndarray:
    """押す点（pelvis 座標）。pose を省略すると設定の置き方を使う。"""
    pose = pose or pose_from_config(cfg)
    return pose.to_pelvis(push_point_local(cfg, side))


def bin_obstacles(cfg: dict[str, Any], steps: int = 4, margin_m: float = 0.0,
                  pose: BinPose | None = None) -> list[dict[str, Any]]:
    """計画（衝突の確認）に使う障害物の箱（Button_Press/J1-gen/configs/press.yaml の obstacles の形）。

    計画の障害物は箱でしか書けないので、すり鉢の斜面は steps 段の階段で近似する。各段の上面は、その段の
    外側の端（高い方）の斜面の高さにするので、斜面より少し高い（安全側）。開口の真下は空けておく（押し込みで手が入るため）。
    2 段では内側の段が斜面より最大 1.25 cm 高く、開口の縁の近くへ下ろす手を「ぶつかる」と断った（2026-11-05）。
    ゴミ箱が斜め（yaw ≠ 0）なら、J1-gen の x_axis で箱を回して置く。
    """
    pose = pose or pose_from_config(cfg)
    g = local_geometry(cfg)
    x0, x1, ya, yb = g["x0"], g["x1"], g["ya"], g["yb"]
    ox0, ox1, oy0, oy1 = g["ox0"], g["ox1"], g["oy0"], g["oy1"]
    zf, zt, zo = g["z_floor"], g["z_top"], g["z_open"]
    boxes: list[tuple[str, list[float], list[float]]] = []
    # 正面側・背面側の帯（幅いっぱい）と、左右の帯（開口の奥行の範囲）。edge は開口の縁の座標、outer は外周の座標
    for name, edge, outer, axis in (("front", ox0, x0, "x"), ("back", ox1, x1, "x"),
                                    ("right", oy0, ya, "y"), ("left", oy1, yb, "y")):
        for k in range(steps):
            # 開口の縁から外周へ k 段目の帯。上面は外側の端の高さ
            b_in = edge + (outer - edge) * k / steps
            b_out = edge + (outer - edge) * (k + 1) / steps
            ztop = zo + (zt - zo) * (k + 1) / steps
            b0, b1 = min(b_in, b_out), max(b_in, b_out)
            if axis == "x":
                lo, hi = [b0, ya, zf], [b1, yb, ztop]
            else:
                lo, hi = [ox0, b0, zf], [ox1, b1, ztop]
            boxes.append((f"bin_{name}_{k}", lo, hi))
    out = []
    x_axis = [float(np.cos(pose.yaw)), float(np.sin(pose.yaw)), 0.0]
    for name, lo, hi in boxes:
        lo_, hi_ = np.asarray(lo) - margin_m, np.asarray(hi) + margin_m
        ob = {"name": name, "center": pose.to_pelvis((lo_ + hi_) / 2).round(4).tolist(),
              "half_size": ((hi_ - lo_) / 2).round(4).tolist()}
        if abs(pose.yaw) > 1e-6:
            ob["x_axis"] = x_axis
        out.append(ob)
    return out
