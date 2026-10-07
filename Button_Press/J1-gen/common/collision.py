"""腕と体（胴体・脚・反対の腕など）がぶつからないかを MuJoCo で確かめる。タスク2。

公式 XML（IK 用 URDF と同じ版）を胴体固定で読み、関節角を入れて接触を調べるだけ（物理は進めない）。
- 肘から先とハンドは、clearance_m 未満に近づいた組も「ぶつかる」とみなす（geom の margin で検出する）。
  肩のリンク（shoulder_roll / shoulder_yaw）は設計上ゼロ姿勢でも胴体から約 9 mm しか離れていないので、
  実際にめり込んだときだけを「ぶつかる」とする
- 同じ腕のリンクどうしは調べない（関節の構造で隣り合っていて常に近い。可動範囲は関節リミットで守る）
- 公式モデルのハンド（rubber_hand）は見た目用のメッシュしか無く、衝突判定の形状が無い。
  そこで、ハンドのメッシュの頂点を包む箱を衝突判定用に足す（このチェック用のモデルだけ。公式ファイルは変えない）
- 床（world）は除く。押す対象（ボトルなど）はモデルに入れない（触れるのが正しいため）
- 机などの障害物は、設定（configs/press.yaml の obstacles）に箱で書けば調べる
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .robot_model import JOINT_NAMES, build_spec

ARM_LINKS = ("shoulder_pitch_link", "shoulder_roll_link", "shoulder_yaw_link", "elbow_link",
             "wrist_roll_link", "wrist_pitch_link", "wrist_yaw_link")
# clearance を付けるリンク（肘から先）。肩のリンクはめり込みだけを調べる
DISTAL_LINKS = ("elbow_link", "wrist_roll_link", "wrist_pitch_link", "wrist_yaw_link")


@dataclass
class Contact:
    body1: str
    body2: str
    dist: float  # [m]（負ならめり込み）

    def __str__(self) -> str:
        return f"{self.body1} ↔ {self.body2}（距離 {self.dist * 1000:+.1f} mm）"


def _hand_box(model: Any, data: Any, side: str) -> tuple[np.ndarray, np.ndarray]:
    """{side}_wrist_yaw_link に付いたハンドのメッシュの頂点を包む箱（中心、半分の大きさ）を body 座標で返す。"""
    import mujoco

    b = model.body(f"{side}_wrist_yaw_link").id
    rb = data.xmat[b].reshape(3, 3)
    pb = data.xpos[b]
    pts = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != b or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = model.geom_dataid[g]
        if "hand" not in model.mesh(mid).name:
            continue
        v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
        rg = data.geom_xmat[g].reshape(3, 3)
        world = v @ rg.T + data.geom_xpos[g]
        pts.append((world - pb) @ rb)
    if not pts:
        raise ValueError(f"{side} のハンドのメッシュが見つからない")
    allp = np.vstack(pts)
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    return (lo + hi) / 2, (hi - lo) / 2


def hand_boxes(robot_cfg: dict[str, Any]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """左右のハンドのメッシュを包む箱（中心、半分の大きさ）。{side}_wrist_yaw_link 基準。"""
    import mujoco

    base = build_spec(robot_cfg, fixed_base=True).compile()
    bd = mujoco.MjData(base)
    mujoco.mj_kinematics(base, bd)
    mujoco.mj_camlight(base, bd)
    return {s: _hand_box(base, bd, s) for s in ("left", "right")}


def add_hand_boxes(spec: Any, boxes: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
    """公式モデルのハンドには衝突判定の形状が無いので、メッシュを包む箱を足す（見た目には出ない）。"""
    import mujoco

    for s, (center, half) in boxes.items():
        g = spec.body(f"{s}_wrist_yaw_link").add_geom()
        g.name = f"{s}_hand_collision_box"
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.pos = list(center)
        g.size = list(half)
        g.contype = 1
        g.conaffinity = 1
        g.group = 3
        g.rgba = [1, 0, 0, 0.3]


def add_obstacles(spec: Any, obstacles: list[dict[str, Any]]) -> None:
    """障害物の箱（机など。pelvis 座標の中心 center と半分の大きさ half_size）をワールドに置く。

    x_axis（任意）: 箱の x 軸の向き（pelvis 座標）。与えると、箱をその向きに回して置く（y 軸は水平に取る）。
    柱の面のように、少し斜めの面を表すときに使う。無ければ pelvis の軸にそろえる。
    """
    import mujoco

    pelvis = np.asarray(spec.body("pelvis").pos, dtype=float)
    for i, ob in enumerate(obstacles):
        quat = [1.0, 0.0, 0.0, 0.0]
        if ob.get("x_axis") is not None:
            x = np.asarray(ob["x_axis"], dtype=float)
            x = x / np.linalg.norm(x)
            y = np.cross([0.0, 0.0, 1.0], x)
            y = y / np.linalg.norm(y)
            rot = np.column_stack([x, y, np.cross(x, y)])
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, rot.reshape(-1))
            quat = q.tolist()
        b = spec.worldbody.add_body(name=f"obstacle_{i}_{ob.get('name', '')}",
                                    pos=list(pelvis + np.asarray(ob["center"], dtype=float)), quat=quat)
        g = b.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=list(ob["half_size"]))
        g.contype = 1
        g.conaffinity = 1


class CollisionChecker:
    def __init__(self, robot_cfg: dict[str, Any], side: str, clearance_m: float,
                 obstacles: list[dict[str, Any]] | None = None) -> None:
        """obstacles: 障害物の箱（机など）。腕（ハンドを含む）が近づく・入る姿勢を「ぶつかる」とする。"""
        import mujoco

        self._mj = mujoco
        self.side = side
        self.clearance = float(clearance_m)

        boxes = hand_boxes(robot_cfg)
        spec = build_spec(robot_cfg, fixed_base=True)
        add_hand_boxes(spec, boxes)
        add_obstacles(spec, obstacles or [])
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        m = self.model
        self._qadr = np.array([m.joint(n).qposadr[0] for n in JOINT_NAMES])
        self.moving = {m.body(f"{side}_{n}").id for n in ARM_LINKS}
        self.hand_box = boxes[side]
        # 肘から先の衝突形状に margin を付け、clearance 未満に近づいた組も接触として出す
        distal = {m.body(f"{side}_{n}").id for n in DISTAL_LINKS}
        for g in range(m.ngeom):
            if m.geom_bodyid[g] in distal and (m.geom_contype[g] or m.geom_conaffinity[g]):
                m.geom_margin[g] = self.clearance

    def contacts(self, q: np.ndarray) -> list[Contact]:
        """関節角 q（29、motor 番号順）で、動かす腕が体にぶつかっている（近すぎる）組の一覧。"""
        mj, m, d = self._mj, self.model, self.data
        d.qpos[self._qadr] = q
        mj.mj_forward(m, d)
        out = []
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
            if b1 == 0 or b2 == 0:  # 床
                continue
            if (b1 in self.moving) == (b2 in self.moving):  # 腕と無関係、または同じ腕どうし
                continue
            if c.dist >= self.clearance:
                continue
            out.append(Contact(m.body(b1).name, m.body(b2).name, float(c.dist)))
        return out
