"""MuJoCo のシーン（机とボトル）を、G1 のモデル（MjSpec）に足す。タスク4・6 のシミュレーション用。"""

from __future__ import annotations

from typing import Any

import numpy as np

TABLE = "scene_table"
BOTTLE = "scene_bottle"


def add_scene(spec: Any, scene_cfg: dict[str, Any]) -> None:
    """机とボトルをワールドに置く（どちらも固定。ボトルは押しても動かない）。"""
    import mujoco

    pelvis = np.asarray(spec.body("pelvis").pos, dtype=float)
    world = spec.worldbody

    t = scene_cfg["table"]
    top = np.asarray(t["top_center"], dtype=float)
    half = np.asarray(t["half_size"], dtype=float)
    table = world.add_body(name=TABLE, pos=list(pelvis + top - [0, 0, half[2]]))
    g = table.add_geom(name=TABLE, type=mujoco.mjtGeom.mjGEOM_BOX, size=list(half))
    g.rgba = list(t["rgba"])

    b = scene_cfg["bottle"]
    base = pelvis + np.array([b["xy"][0], b["xy"][1], top[2]])
    bottle = world.add_body(name=BOTTLE, pos=list(base))
    parts = [
        ("body", b["radius"], 0.0, b["body_height"], b["body_rgba"]),
        ("neck", b["neck_radius"], b["body_height"], b["neck_height"], b["neck_rgba"]),
        ("cap", b["neck_radius"] * 1.2, b["body_height"] + b["neck_height"], b["cap_height"], b["cap_rgba"]),
    ]
    for name, r, z0, hgt, rgba in parts:
        g = bottle.add_geom(name=f"{BOTTLE}_{name}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                            size=[r, hgt / 2, 0], pos=[0, 0, z0 + hgt / 2])
        g.rgba = list(rgba)


def bottle_front_point(scene_cfg: dict[str, Any], height_fraction: float = 0.5) -> np.ndarray:
    """正解の値: ボトルの胴の、ロボット側（−x 側）の表面の点（pelvis 座標）。胴の高さの height_fraction の位置。"""
    b = scene_cfg["bottle"]
    z = scene_cfg["table"]["top_center"][2] + b["body_height"] * height_fraction
    return np.array([b["xy"][0] - b["radius"], b["xy"][1], z])


def table_obstacle(scene_cfg: dict[str, Any], below_m: float = 0.4) -> dict[str, Any]:
    """シーンの机を、衝突の確認の障害物（configs/press.yaml の obstacles の形）にする。脚の分として下に below_m 伸ばす。"""
    t = scene_cfg["table"]
    top = np.asarray(t["top_center"], dtype=float)
    half = np.asarray(t["half_size"], dtype=float)
    hz = (2 * half[2] + below_m) / 2
    return {"name": "table", "center": [top[0], top[1], top[2] - hz], "half_size": [half[0], half[1], hz]}


def detection_pose(scene_cfg: dict[str, Any], q: np.ndarray) -> np.ndarray:
    """q（29）のうち、detection_pose_deg に書いた関節だけを検出用の姿勢にしたもの。"""
    out = np.asarray(q, dtype=float).copy()
    for idx, deg in scene_cfg.get("detection_pose_deg", {}).items():
        out[int(idx)] = np.radians(float(deg))
    return out


def build_scene_model(robot_cfg: dict[str, Any], scene_cfg: dict[str, Any], fixed_base: bool = True) -> Any:
    """G1 + 机 + ボトルの MjModel。"""
    from .robot_model import build_spec

    spec = build_spec(robot_cfg, fixed_base=fixed_base)
    add_scene(spec, scene_cfg)
    return spec.compile()
