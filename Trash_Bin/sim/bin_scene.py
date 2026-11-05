"""MuJoCo のシーン（ゴミ箱）を、G1 のモデル（MjSpec）に足す。設定は configs/bin_scene.yaml。

Button_Press/J1-gen/common/sim_scene.py（机とボトル）と同じ書き方にしてある。
- 本体は中空の箱（4 枚の壁と底）。上面は浅いすり鉢で、4 枚の傾いた板で作る
- フラップは開口の中の板で、前後方向（x）の軸のヒンジで回る
- 中身は見た目だけ（当たり判定なし）。頭カメラに写った画素を数えるのに使う

座標: ゴミ箱全体を 1 つの body にし、設定の置き方（bin.front_x / center_y / yaw_deg）で置く。形は箱の座標
（lib/bin_geometry.py）で書く。高さは設定では床から書き、pelvis_height_m を引いて pelvis 座標に直す
（シミュレーションは pelvis を固定している）。
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

# 押す点・障害物の計算は sim / real 共通（lib/bin_geometry.py）。ここからも使えるように読み込んでおく
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from bin_geometry import (  # noqa: E402,F401
    BinPose, bin_obstacles, local_geometry, pose_from_config, push_point, push_point_local,
)

CONFIG_DIR = Path(__file__).resolve().parents[1] / "configs"

BIN = "bin"                  # 本体と上面の geom の名前の頭
FLAP = "bin_flap"            # フラップの body と geom
FLAP_JOINT = "bin_flap_hinge"
CONTENT = "bin_content_"     # 中身の geom の名前の頭


def load_scene_config(path: str | Path | None = None) -> dict[str, Any]:
    """シーンの設定を読む。相対パスは configs/ 基準で探す。"""
    p = Path(path) if path else CONFIG_DIR / "bin_scene.yaml"
    if not p.is_absolute() and not p.exists():
        p = CONFIG_DIR / p
    with open(p, encoding="utf-8") as f:
        return yaml.safe_load(f)


def add_bin_scene(spec: Any, cfg: dict[str, Any]) -> None:
    """ゴミ箱をワールドに置く。ゴミ箱全体を 1 つの body（bin_body）にし、設定の置き方（中心と yaw）で置く。"""
    import mujoco

    pose = pose_from_config(cfg)
    g = local_geometry(cfg)
    b, fl = cfg["bin"], cfg["flap"]
    pelvis = np.asarray(spec.body("pelvis").pos, dtype=float)
    yaw = pose.yaw
    body = spec.worldbody.add_body(name=f"{BIN}_body", pos=list(pelvis + [pose.center_xy[0], pose.center_xy[1], 0.0]),
                                   quat=[float(np.cos(yaw / 2)), 0.0, 0.0, float(np.sin(yaw / 2))])
    rgba = list(b["rgba"])
    t = float(b["wall_thickness"])

    def box(name: str, lo: list[float], hi: list[float], color: list[float], collide: bool = True) -> None:
        lo_, hi_ = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
        geom = body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=list((lo_ + hi_) / 2),
                             size=list((hi_ - lo_) / 2))
        geom.rgba = color
        if not collide:
            geom.contype, geom.conaffinity = 0, 0

    def slope(name: str, a: np.ndarray, c: np.ndarray, half_w: float, along: str) -> None:
        """a から c へ傾いた板（上面が a-c を結ぶ線になる）。along は板の幅の向き（x か y）。"""
        mid = (a + c) / 2
        d = c - a
        length = float(np.linalg.norm(d))
        if along == "y":  # 板は x 方向に傾く（正面・背面）。y 軸まわりに回す
            ang = -np.arctan2(d[2], d[0])
            quat = [np.cos(ang / 2), 0.0, np.sin(ang / 2), 0.0]
            size = [length / 2, half_w, t / 2]
        else:             # 板は y 方向に傾く（左右の側面）。x 軸まわりに回す
            ang = np.arctan2(d[2], d[1])
            quat = [np.cos(ang / 2), np.sin(ang / 2), 0.0, 0.0]
            size = [half_w, length / 2, t / 2]
        geom = body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(mid - [0, 0, t / 2]),
                             quat=quat, size=size)
        geom.rgba = rgba

    x0, x1, ya, yb = g["x0"], g["x1"], g["ya"], g["yb"]
    zf, zt, zo = g["z_floor"], g["z_top"], g["z_open"]
    # 本体（中空）: 壁は上端まで、底は薄い板
    box(f"{BIN}_front", [x0, ya, zf], [x0 + t, yb, zt], rgba)
    box(f"{BIN}_back", [x1 - t, ya, zf], [x1, yb, zt], rgba)
    box(f"{BIN}_right", [x0, ya, zf], [x1, ya + t, zt], rgba)
    box(f"{BIN}_left", [x0, yb - t, zf], [x1, yb, zt], rgba)
    box(f"{BIN}_bottom", [x0, ya, zf], [x1, yb, zf + 0.01], rgba)
    # 上面のすり鉢: 外周（zt）から開口の縁（zo）へ下がる 4 枚の板
    hw_x, hw_y = (yb - ya) / 2, (x1 - x0) / 2
    slope(f"{BIN}_funnel_front", np.array([x0, 0.0, zt]), np.array([g["ox0"], 0.0, zo]), hw_x, "y")
    slope(f"{BIN}_funnel_back", np.array([g["ox1"], 0.0, zo]), np.array([x1, 0.0, zt]), hw_x, "y")
    slope(f"{BIN}_funnel_right", np.array([0.0, ya, zt]), np.array([0.0, g["oy0"], zo]), hw_y, "x")
    slope(f"{BIN}_funnel_left", np.array([0.0, g["oy1"], zo]), np.array([0.0, yb, zt]), hw_y, "x")

    # フラップ: 上面が開口の縁の高さに来るように置き、x 軸まわりのヒンジで回す
    th = float(fl["thickness"])
    flap = body.add_body(name=FLAP, pos=[0.0, 0.0, zo - th / 2])
    rng = np.radians(float(fl["range_deg"]))
    flap.add_joint(name=FLAP_JOINT, type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0], range=[-rng, rng],
                   stiffness=float(fl["stiffness_nm_per_rad"]), springref=0.0, damping=float(fl["damping"]))
    fg = flap.add_geom(name=FLAP, type=mujoco.mjtGeom.mjGEOM_BOX,
                       size=[float(fl["depth"]) / 2, float(fl["width"]) / 2, th / 2])
    fg.mass = float(fl["mass_kg"])
    fg.rgba = list(fl["rgba"])
    sc, ss = fl["sticker_center"], fl["sticker_size"]
    sg = flap.add_geom(name=f"{FLAP}_sticker", type=mujoco.mjtGeom.mjGEOM_BOX,
                       pos=[float(sc[0]), float(sc[1]), th / 2 + 0.0001], size=[ss[0] / 2, ss[1] / 2, 0.0001])
    sg.rgba = list(fl["sticker_rgba"])
    sg.contype, sg.conaffinity, sg.mass = 0, 0, 0.0

    wc = cfg.get("wrist_camera")
    if wc:
        add_wrist_camera(spec, wc)

    # 押す点の目印（見た目だけの小さな球。押す点が分かるように）
    mk = body.add_geom(name=f"{BIN}_push_marker", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                       pos=list(push_point_local(cfg)), size=[0.006, 0, 0])
    mk.rgba = [1.0, 0.2, 0.2, 0.8]
    mk.contype, mk.conaffinity = 0, 0

    # 中身（見た目だけ）。位置はゴミ箱の上面の中心から
    ph = float(cfg["pelvis_height_m"])
    for c in cfg.get("contents") or []:
        cx, cy = float(c["center"][0]), float(c["center"][1])
        sx, sy = float(c["size"][0]) / 2, float(c["size"][1]) / 2
        z0, z1 = float(c["z"][0]) - ph, float(c["z"][1]) - ph
        box(f"{CONTENT}{c['name']}", [cx - sx, cy - sy, z0], [cx + sx, cy + sy, z1], list(c["rgba"]), collide=False)


def add_wrist_camera(spec: Any, wc: dict[str, Any]) -> None:
    """手首カメラを parent の body に付ける。向きは光学座標（x 右、y 下、z 前）の rpy で与える。

    MuJoCo のカメラは −z を見て、x 右、y 上。光学座標の y・z を反転したものが MuJoCo のカメラの座標になる。
    rpy は URDF と同じ（R = Rz(yaw)·Ry(pitch)·Rx(roll)）。
    """
    import mujoco

    r, p, y = np.radians(np.asarray(wc["rpy_deg"], dtype=float))
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    rot = rz @ ry @ rx @ np.diag([1.0, -1.0, -1.0])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rot.flatten())
    body = spec.body(wc["parent"])
    body.add_camera(name=wc["name"], pos=list(map(float, wc["xyz"])), quat=quat.tolist(), fovy=float(wc["fovy_deg"]))


def start_pose(cfg: dict[str, Any], q: np.ndarray) -> np.ndarray:
    """開始時の姿勢（29 関節）: 検出の姿勢（detection_pose_deg）に、右腕だけ start_right_arm_deg を重ねる。"""
    out = np.asarray(q, dtype=float).copy()
    for idx, deg in (cfg.get("detection_pose_deg") or {}).items():
        out[int(idx)] = np.radians(float(deg))
    arm = cfg.get("start_right_arm_deg")
    if arm is not None:
        out[22:29] = np.radians(np.asarray(arm, dtype=float))  # 右腕は motor 22〜28
    return out


def flap_angle_deg(model: Any, data: Any) -> float:
    """フラップの回転角 [deg]（閉じていれば 0）。"""
    return float(np.degrees(data.qpos[model.joint(FLAP_JOINT).qposadr[0]]))
