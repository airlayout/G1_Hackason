"""学習の条件を、評価環境（Yada）の設定から決める（シミュレーターに依存しない部分）。

- 乗り場は本番に似た形（configs/elevator_prod.yaml の値のまま）を 1 通りだけ作る
- 評価で試行ごとに変わる条件（configs/contest.yaml の randomize と realism.stance）のうち、
  押すボタンの位置に効くもの（壁までの距離、柱とボタンの出っ張り、柱の左右の位置、ボタンの高さ、立ち位置のずれ）は、
  **ロボットの立つ位置をずらして**作る。乗り場を環境ごとに作り分けるより軽いため。
  ボタンの間隔・直径などの形の違いは、まだ入れていない（README の「まだできていないこと」）
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .paths import use_yada, yada_config

TARGET_BUTTON = "down"  # 押すボタン（一般用の ▼）。評価は evaluate.py --target down


@dataclass(frozen=True)
class OffsetRanges:
    """ロボットの立つ位置のずらし幅（乗り場に対して。pelvis 座標の向き）[m]・[rad]。"""

    x: tuple[float, float]
    y: tuple[float, float]
    z: tuple[float, float]
    yaw: tuple[float, float]


def _lo_hi(v: Any) -> tuple[float, float]:
    if isinstance(v, (list, tuple)):
        return float(v[0]), float(v[1])
    return -float(v), float(v)


def target_face_nominal(scene_cfg: dict[str, Any], target: str = TARGET_BUTTON) -> tuple[float, float, float]:
    """乗り場の設定のままのときの、目標のボタンの面の中心（pelvis 座標）[m]。"""
    use_yada()
    from common.scene_spec import build_hall_scene

    f = build_hall_scene(scene_cfg).button(target).face_center
    return float(f[0]), float(f[1]), float(f[2])


def offset_ranges(contest_cfg: dict[str, Any], scene_cfg: dict[str, Any], target: str = TARGET_BUTTON) -> OffsetRanges:
    """評価の条件の範囲を覆う、ロボットの立つ位置のずらし幅。

    ボタンがロボットに対して +d ずれる条件は、ロボットを −d ずらして作る。
    """
    r = contest_cfg["randomize"]
    st = contest_cfg["realism"]["stance"]
    fx0 = float(scene_cfg["wall"]["front_x"])
    cp0 = float(scene_cfg["column"]["protrusion"])
    bp0 = float(scene_cfg["buttons"]["common"]["protrusion"])
    cy0 = float(scene_cfg["column"]["center_y"])
    items = {b["name"]: b for b in scene_cfg["buttons"]["items"]}

    # ボタンの面の x = 壁の表面 − 柱の出っ張り − ボタンの出っ張り
    fx_lo, fx_hi = _lo_hi(r["wall.front_x"])
    cp_lo, cp_hi = _lo_hi(r["column.protrusion"])
    bp_lo, bp_hi = _lo_hi(r["buttons.common.protrusion"])
    face_x0 = fx0 - cp0 - bp0
    face_x = (fx_lo - cp_hi - bp_hi, fx_hi - cp_lo - bp_lo)
    # 柱の左右の位置
    cy = _lo_hi(r["column.center_y"])
    # ボタンの高さ（▼ = ▲ の高さ − 間隔）
    up = _lo_hi(r["button_up_height"])
    sp = _lo_hi(r["button_spacing"])
    if target == "down":
        h0 = float(items["down"]["height"])
        h = (up[0] - sp[1], up[1] - sp[0])
    elif target == "up":
        h0 = float(items["up"]["height"])
        h = up
    else:
        raise ValueError(f"target は up か down: {target}")
    sxy = _lo_hi(st["xy_m"])
    syaw = tuple(math.radians(v) for v in _lo_hi(st["yaw_deg"]))
    return OffsetRanges(
        x=(face_x0 - face_x[1] + sxy[0], face_x0 - face_x[0] + sxy[1]),
        y=(cy0 - cy[1] + sxy[0], cy0 - cy[0] + sxy[1]),
        z=(h0 - h[1], h0 - h[0]),
        yaw=(float(syaw[0]), float(syaw[1])),
    )


@dataclass(frozen=True)
class TaskSpec:
    """学習環境が評価環境からそろえる値。"""

    contest_cfg: dict[str, Any]
    scene_cfg: dict[str, Any]
    target: str
    control_hz: float
    limit_margin: float
    max_joint_speed: float
    press_depth: float
    travel: float
    spring_stiffness: float
    initial_pose: dict[str, float]  # 関節の名前 → 初期の角度 [rad]
    fingertip_link: str
    fingertip_offset: tuple[float, float, float]
    fingertip_radius: float
    offsets: OffsetRanges
    press_ok: tuple[float, float]
    press_max: float
    post_press_s: float
    release_n: float


def load_task_spec(target: str = TARGET_BUTTON) -> TaskSpec:
    use_yada()
    from common.j1gen_bridge import j1gen_config
    from contest.interface import JOINT_NAMES

    contest = yada_config("contest.yaml")
    scene = yada_config(contest.get("scene_file", "elevator_prod.yaml"))
    robot = j1gen_config("robot.yaml")
    bc = scene["buttons"]["common"]
    pf = contest["press_force"]
    ee = robot["end_effector"]["right"]
    pose = {n: 0.0 for n in JOINT_NAMES}
    for idx, deg in scene.get("initial_pose_deg", {}).items():
        pose[JOINT_NAMES[int(idx)]] = math.radians(float(deg))
    return TaskSpec(
        contest_cfg=contest,
        scene_cfg=scene,
        target=target,
        control_hz=float(contest["control_hz"]),
        limit_margin=float(contest["safety"]["limit_margin_rad"]),
        max_joint_speed=float(contest["safety"]["max_joint_speed_rad_s"]),
        press_depth=float(bc["press_depth"]),
        travel=float(bc["travel"]),
        spring_stiffness=float(bc["stiffness"]),
        initial_pose=pose,
        fingertip_link=str(ee["link"]),
        fingertip_offset=tuple(float(v) for v in ee["offset"]),
        fingertip_radius=float(scene.get("fingertip", {}).get("radius", 0.008)),
        offsets=offset_ranges(contest, scene, target),
        press_ok=(float(pf["ok_range_n"][0]), float(pf["ok_range_n"][1])),
        press_max=float(pf["max_n"]),
        post_press_s=float(pf["post_press_s"]),
        release_n=float(pf["release_n"]),
    )
