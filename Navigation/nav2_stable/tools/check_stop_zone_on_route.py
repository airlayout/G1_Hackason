#!/usr/bin/env python3
"""巡回路を歩いたとき、Collision Monitor の停止範囲に**壁が入らないか**を地図だけで確かめる。

Collision Monitor（nav2_params.yaml の `collision_monitor`）は、機体の前の箱
（前方 0.35〜1.0m・左右 ±0.25m）に点が入ると速度を 0 にする。箱を大きく取りすぎると、
人がいなくても**壁や家具で止まり続ける**。実機に出す前に、巡回路の上で箱に
地図の占有セルが入る場所を洗い出す。

## やること

1. 占有格子を読み、Nav2 と同じく壁から離れるほど安くなるコストを作る
   （inflation_radius 0.40 / cost_scaling_factor 3.0 / 内接半径 0.20 を近似）
2. 巡回路の隣り合う点の間に最小コスト経路を引く（NavFn の代わり）
3. 経路に沿って機体を前向きに進め、各姿勢で停止範囲に入る占有セルを数える

⚠️ **近似である。** 実際の経路（NavFn + RPP）とは数 cm〜数十 cm ずれるし、
地図の占有セルは高さを持たない（Collision Monitor は 0.15〜1.0m の点だけを見る）。
「止まりそうな場所の候補」を挙げる道具として使うこと。

    .venv/bin/python nav2_stable/tools/check_stop_zone_on_route.py \\
        --map nav2_stable/maps/grids/room_a_map_20260911_edited.yaml \\
        --route nav2_stable/g1_ws/src/g1_navigation/config/patrol_room_a.yaml

依存: numpy / scipy / scikit-image（Navigation/.venv に入っている）。yaml は使わない
（この 2 種類の yaml は単純なので、正規表現で読む）。
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import numpy as np
from scipy import ndimage
from skimage.graph import route_through_array

INSCRIBED_M = 0.20       # footprint 0.5×0.4m の内接半径
INFLATION_M = 0.40       # nav2_params.yaml の inflation_radius
COST_SCALING = 3.0       # nav2_params.yaml の cost_scaling_factor


def read_map(yaml_path: Path) -> tuple[np.ndarray, float, tuple[float, float]]:
    """map_server 形式の yaml+pgm を読み、占有(True)の配列・解像度・原点を返す。

    配列は**行 0 が地図の下端（y 最小）**になるよう上下を反転して返す。
    """
    text = yaml_path.read_text()
    image = re.search(r"image:\s*(\S+)", text).group(1)
    res = float(re.search(r"resolution:\s*([\d.]+)", text).group(1))
    ox, oy = (float(v) for v in re.search(r"origin:\s*\[([^,]+),([^,]+),", text).groups())
    occ_th = float(re.search(r"occupied_thresh:\s*([\d.]+)", text).group(1))

    raw = (yaml_path.parent / image).read_bytes()
    # P5 のヘッダは「P5 幅 高さ 最大値」の 4 語。コメント行は無い前提（本リポジトリの地図はそう）
    parts = raw.split(maxsplit=4)
    w, h, maxval = int(parts[1]), int(parts[2]), int(parts[3])
    pix = np.frombuffer(parts[4][:w * h], dtype=np.uint8).reshape(h, w)
    # negate: 0 なら 白=自由、黒=占有。占有確率 = (255 - 値) / 255
    occupied = (maxval - pix.astype(np.float32)) / maxval > occ_th
    return occupied[::-1].copy(), res, (ox, oy)


def read_route(route_path: Path) -> list[tuple[str, float, float]]:
    """patrol_*.yaml から (名前, x, y) を順に読む。"""
    pts = []
    for m in re.finditer(r"name:\s*(\w+),\s*x:\s*([-\d.]+),\s*y:\s*([-\d.]+)", route_path.read_text()):
        pts.append((m.group(1), float(m.group(2)), float(m.group(3))))
    if len(pts) < 2:
        raise SystemExit(f"[check] 巡回路の点が 2 つ未満: {route_path}")
    return pts


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--map", type=Path, required=True)
    ap.add_argument("--route", type=Path, required=True)
    ap.add_argument("--x-near", type=float, default=0.35, help="停止範囲の後ろ端[m]")
    ap.add_argument("--x-far", type=float, default=1.0, help="停止範囲の前端[m]")
    ap.add_argument("--half-width", type=float, default=0.25, help="停止範囲の半幅[m]")
    args = ap.parse_args()

    occ, res, (ox, oy) = read_map(args.map)
    route = read_route(args.route)
    dist = ndimage.distance_transform_edt(~occ) * res

    # NavFn が見るコストの近似: 内接半径の内側は通れない、その外は指数で減衰
    cost = np.ones_like(dist)
    band = (dist > INSCRIBED_M) & (dist < INFLATION_M)
    cost[band] += 252.0 * np.exp(-COST_SCALING * (dist[band] - INSCRIBED_M)) / 50.0
    cost[dist <= INSCRIBED_M] = 1e6

    def to_cell(x: float, y: float) -> tuple[int, int]:
        return int((y - oy) / res), int((x - ox) / res)

    occ_rc = np.argwhere(occ)
    occ_xy = np.stack([ox + (occ_rc[:, 1] + 0.5) * res, oy + (occ_rc[:, 0] + 0.5) * res], axis=1)

    print(f"[check] 地図 {args.map.name}  停止範囲 x=[{args.x_near}, {args.x_far}] "
          f"y=±{args.half_width}")
    total_len = 0.0
    total_hit = 0.0
    legs = list(zip(route, route[1:] + route[:1]))
    for (n0, x0, y0), (n1, x1, y1) in legs:
        cells, _ = route_through_array(cost, to_cell(x0, y0), to_cell(x1, y1),
                                       fully_connected=True, geometric=True)
        path = np.array([(ox + (c + 0.5) * res, oy + (r + 0.5) * res) for r, c in cells])
        seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        length = float(s[-1])
        # 5cm ごとに姿勢を取る。向きは 0.3m 先の点への方位（RPP の先読みに近い）
        hit_len = 0.0
        worst = (0, 0.0, 0.0)
        for si in np.arange(0.0, length, 0.05):
            p = np.array([np.interp(si, s, path[:, 0]), np.interp(si, s, path[:, 1])])
            sa = min(si + 0.3, length)
            q = np.array([np.interp(sa, s, path[:, 0]), np.interp(sa, s, path[:, 1])])
            if np.allclose(p, q):
                continue
            yaw = math.atan2(q[1] - p[1], q[0] - p[0])
            d = occ_xy - p
            near = np.abs(d).max(axis=1) < args.x_far + 0.1
            d = d[near]
            lx = d[:, 0] * math.cos(yaw) + d[:, 1] * math.sin(yaw)
            ly = -d[:, 0] * math.sin(yaw) + d[:, 1] * math.cos(yaw)
            n_in = int(((lx >= args.x_near) & (lx <= args.x_far) & (np.abs(ly) <= args.half_width)).sum())
            if n_in > 0:
                hit_len += 0.05
                if n_in > worst[0]:
                    worst = (n_in, float(p[0]), float(p[1]))
        total_len += length
        total_hit += hit_len
        mark = "✅" if hit_len == 0 else "⚠️"
        msg = (f"[check] {mark} {n0}→{n1}  経路 {length:5.2f}m  "
               f"停止範囲に壁が入る区間 {hit_len:4.2f}m")
        if worst[0]:
            msg += f"（最多 {worst[0]} セル @ x={worst[1]:.2f}, y={worst[2]:.2f}）"
        print(msg)
    print(f"[check] 合計: 経路 {total_len:.1f}m のうち {total_hit:.2f}m で壁が停止範囲に入る")


if __name__ == "__main__":
    main()
