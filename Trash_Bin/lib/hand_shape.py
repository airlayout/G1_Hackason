"""計画とシミュレーションで使う手の衝突形状を、実際の手（ゴム製ハンドのメッシュ）に近づける。

Button_Press/J1-gen は、手をメッシュ全体を包む箱 1 つ（約 13 × 6.6 × 10.6 cm）で表す（common/collision.py の
add_hand_boxes）。箱の隅は実際の手より大きく張り出すので、ゴミ箱の開口（22 × 14 cm、周りがすり鉢）に手を入れると、
実際には当たらないのに「すき間 1 cm を割る」として計画が通らなかった（2026-11-05）。

ここでは、メッシュの頂点を手の座標で格子（指の向き × 指の並ぶ向き）に区切り、区画ごとに頂点を包む小さな箱を作る。
箱の和は頂点をすべて含むので、判定は安全側のまま、隅の張り出しだけが減る。すき間の決まり（clearance_m）は変えない。

install() を呼ぶと、J1-gen の add_hand_boxes をこの形に差し替える（J1-gen のファイルは変えない）。
計画（CollisionChecker）とシミュレーション（SimBackend）は、どちらも呼び出したときにこの関数を参照するので両方に効く。
"""

from __future__ import annotations

from typing import Any

import numpy as np

# 格子の分け方: 指の向き（手の x）と、指の並ぶ向き（手の z）の分割数。厚みの向き（手の y）は分けない
DEFAULT_GRID = (5, 3)


def hand_part_boxes(robot_cfg: dict[str, Any], grid: tuple[int, int] = DEFAULT_GRID
                    ) -> dict[str, list[tuple[np.ndarray, np.ndarray]]]:
    """左右のハンドを包む小さな箱の一覧（中心、半分の大きさ）。{side}_wrist_yaw_link 基準。"""
    import mujoco

    from common.robot_model import build_spec

    m = build_spec(robot_cfg, fixed_base=True).compile()
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    out: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {}
    for side in ("left", "right"):
        b = m.body(f"{side}_wrist_yaw_link").id
        rb, pb = d.xmat[b].reshape(3, 3), d.xpos[b]
        pts = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != b or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = m.geom_dataid[g]
            if "hand" not in m.mesh(mid).name:
                continue
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            world = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
            pts.append((world - pb) @ rb)
        allp = np.vstack(pts)
        nx, nz = grid
        xs = np.linspace(allp[:, 0].min(), allp[:, 0].max(), nx + 1)
        zs = np.linspace(allp[:, 2].min(), allp[:, 2].max(), nz + 1)
        boxes = []
        for i in range(nx):
            for k in range(nz):
                sel = ((allp[:, 0] >= xs[i]) & (allp[:, 0] <= xs[i + 1])
                       & (allp[:, 2] >= zs[k]) & (allp[:, 2] <= zs[k + 1]))
                if not sel.any():
                    continue
                lo, hi = allp[sel].min(axis=0), allp[sel].max(axis=0)
                boxes.append(((lo + hi) / 2, np.maximum((hi - lo) / 2, 0.002)))
        out[side] = boxes
    return out


def install(robot_cfg: dict[str, Any], grid: tuple[int, int] = DEFAULT_GRID) -> None:
    """J1-gen の add_hand_boxes を、小さな箱の集まりを足す関数に差し替える。"""
    import mujoco

    from common import collision

    parts = hand_part_boxes(robot_cfg, grid)

    def add_hand_part_boxes(spec: Any, boxes: Any = None) -> None:  # noqa: ARG001（J1-gen と同じ呼び方にそろえる）
        for side, items in parts.items():
            body = spec.body(f"{side}_wrist_yaw_link")
            for j, (center, half) in enumerate(items):
                g = body.add_geom()
                g.name = f"{side}_hand_collision_box_{j}"
                g.type = mujoco.mjtGeom.mjGEOM_BOX
                g.pos = list(center)
                g.size = list(half)
                g.contype = 1
                g.conaffinity = 1
                g.group = 3
                g.rgba = [1, 0, 0, 0.3]

    collision.add_hand_boxes = add_hand_part_boxes
