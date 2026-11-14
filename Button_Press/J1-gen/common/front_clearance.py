"""腰をひねる前に、体の前の物（柱など）までの距離を確かめる（common/elevator_press.py の press.waist_scan）。

腰を回すと、肩と腕が体の前へ出る。衝突判定（common/collision.py）は動かす腕と他の物の組しか見ていないので、
胴体・頭・押さない方の腕と柱の距離は見ていない。そこで、ひねる前に深度の画像から前の物までの距離を出し、
ひねっても上体が当たらない距離（clear_x_m）より近ければ、ひねらない。

- front_distance: 深度 → pelvis 座標の点のうち、上体が通る高さと幅にある点の x の小さい側（下位 percentile %）。
  最小にしないのは、深度の外れ値（飛んだ点）で距離が極端に小さくなるのを避けるため。点が足りなければ None（確かめられない）。
- upper_body_front_x / max_front_x: モデル（MuJoCo。common/collision.py と同じ、ハンドを包む箱つき）で、腰 yaw を振ったときに
  上体（腰 yaw より先の胴体・頭・両腕・手）の形状がいちばん前へ出る x（pelvis 座標）。
  clear_x_m = max_front_x（始めの立った姿勢、腰 yaw −20〜+20°）+ 余裕 5 cm、を設定の値の下限にする（tests で確かめる）。
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .elevator_buttons import pelvis_points
from .robot_model import JOINT_NAMES, build_spec

MARGIN_M = 0.05  # clear_x_m に足す余裕 [m]（深度の誤差・指令と実際の腰のずれ・床の凹凸で体が前へ傾く分）


def front_distance(depth_m: np.ndarray, K: np.ndarray, rot: np.ndarray, pos: np.ndarray,
                   z_range_m: tuple[float, float], half_width_m: float, percentile: float,
                   min_points: int, min_depth_m: float = 0.2, max_depth_m: float = 3.0) -> tuple[float | None, int]:
    """体の前の物までの距離 [m]（pelvis から前向きの x）と、使った点の数。点が min_points 未満なら (None, 点の数)。

    z_range_m（pelvis の z。腰〜肩）と |y| <= half_width_m の範囲にある点だけを使う（上体が通る範囲）。
    """
    depth = np.asarray(depth_m, dtype=np.float64)
    P = pelvis_points(depth, np.asarray(K, dtype=float), rot, pos).reshape(-1, 3)
    d = depth.reshape(-1)
    ok = (d > min_depth_m) & (d < max_depth_m) & np.isfinite(d)
    ok &= (P[:, 2] >= z_range_m[0]) & (P[:, 2] <= z_range_m[1]) & (np.abs(P[:, 1]) <= half_width_m)
    n = int(ok.sum())
    if n < int(min_points):
        return None, n
    return float(np.percentile(P[ok, 0], float(percentile))), n


def _upper_body_bodies(model: Any) -> set[int]:
    """腰 yaw より先の body（胴体・頭・両腕・手。脚は腰をひねっても動かないので含めない）。"""
    waist = model.body("waist_yaw_link").id
    out: set[int] = set()
    for b in range(1, model.nbody):
        k = b
        while k > 0 and k != waist:
            k = int(model.body_parentid[k])
        if k == waist:
            out.add(b)
    return out


class UpperBodyFront:
    """上体の形状がいちばん前へ出る x を、関節角から求める（モデルは 1 回だけ作る）。"""

    def __init__(self, robot_cfg: dict[str, Any]) -> None:
        import mujoco

        from .collision import add_hand_boxes, hand_boxes

        self._mj = mujoco
        spec = build_spec(robot_cfg, fixed_base=True)
        add_hand_boxes(spec, hand_boxes(robot_cfg))  # ハンドは見た目のメッシュしか無いので、包む箱（実物より大きめ）を使う
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self._qadr = np.array([self.model.joint(n).qposadr[0] for n in JOINT_NAMES])
        self._pelvis = self.model.body("pelvis").id
        self._bodies = _upper_body_bodies(self.model)

    def front_x(self, q: np.ndarray) -> float:
        """関節角 q（29）での、上体の形状の x の最大（pelvis 座標）。メッシュは頂点、箱は 8 隅、それ以外は外接球。"""
        mj, m, d = self._mj, self.model, self.data
        d.qpos[self._qadr] = np.asarray(q, dtype=float)
        mj.mj_forward(m, d)
        pel = d.xpos[self._pelvis]
        best = -np.inf
        for g in range(m.ngeom):
            if int(m.geom_bodyid[g]) not in self._bodies:
                continue
            rg, pg = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
            t = int(m.geom_type[g])
            if t == mj.mjtGeom.mjGEOM_MESH:
                mid = int(m.geom_dataid[g])
                v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
                x = float(((v @ rg.T + pg) - pel)[:, 0].max())
            elif t == mj.mjtGeom.mjGEOM_BOX:
                corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * m.geom_size[g]
                x = float(((corners @ rg.T + pg) - pel)[:, 0].max())
            else:
                x = float((pg - pel)[0] + m.geom_rbound[g])
            best = max(best, x)
        return best


def max_front_x(robot_cfg: dict[str, Any], q_stand: np.ndarray, yaws_deg: list[float]) -> float:
    """始めの姿勢 q_stand の腰 yaw（motor 12）を yaws_deg に入れ替えたときの、上体の前端の x の最大 [m]。"""
    body = UpperBodyFront(robot_cfg)
    out = -np.inf
    for yaw in yaws_deg:
        q = np.asarray(q_stand, dtype=float).copy()
        q[12] = np.radians(yaw)
        out = max(out, body.front_x(q))
    return float(out)


def required_clear_x(robot_cfg: dict[str, Any], q_stand: np.ndarray, max_deg: float, step_deg: float = 5.0) -> float:
    """clear_x_m の下限 = 腰 yaw を −max_deg〜+max_deg（step_deg 刻み）に振ったときの上体の前端 + MARGIN_M。"""
    n = int(round(max_deg / step_deg))
    yaws = [k * step_deg for k in range(-n, n + 1)]
    return max_front_x(robot_cfg, q_stand, yaws) + MARGIN_M
