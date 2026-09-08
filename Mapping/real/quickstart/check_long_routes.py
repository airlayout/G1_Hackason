#!/usr/bin/env python3
"""長距離のゴール候補が、robot_radius を膨らませた 2D 地図上で繋がっているかを
BFS で確かめる。**Isaac Sim を立てる前にやる**（起動 4〜5 分 + 走行 1 回 2〜4 分かかる）。

nav_map_clean / robot_radius 0.25 / 4 連結。`check_planning.py` は Nav2 を立てないと
動かないが、こちらは地図だけで済むので**ゴール候補の絞り込みはここでやる**。

## ⚠️ 「BFS 経路 ÷ 直線」を迂回率と読んではいけない

4 連結は斜めに進めないので、純粋な斜め移動でも **1.41 倍**になる。
正しい比較相手は**マンハッタン距離 |dx|+|dy|**（4 連結の下限）で、
**BFS が下限ちょうどなら迂回ゼロ**である。この表はその差 (extra) を出す。

実測（2026-09-08、nav_map_clean）: 迂回ゼロの長距離ルートは 8 通りあり、
最長は wp 7→11 の直線 23.80 m / 経路 32.90 m。
"""
import itertools, json, math
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

R = Path("Mapping/real/runs/20260906T135940_UiS_room_v3")
ROBOT_RADIUS = 0.25          # IsaacSim_Env/config/nav2.yaml の値
MIN_STRAIGHT_M = 8.0         # これ未満の組は出さない
EXTRA_TOL_M = 0.35           # マンハッタン下限との差がこれ未満なら迂回ゼロ

meta = {}
for line in (R / "map/nav_map_clean.yaml").read_text().splitlines():
    if ":" in line:
        k, _, v = line.partition(":")
        meta[k.strip()] = v.strip()
res = float(meta["resolution"])
ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
img = np.asarray(Image.open(R / "map/nav_map_clean.pgm"), dtype=np.float64)
occ = ((255.0 - img) / 255.0) > float(meta["occupied_thresh"])
h, w = occ.shape
# robot_radius ぶん膨らませる（機体の中心が入れないセル）
rad = int(math.ceil(ROBOT_RADIUS / res))
blocked = ndimage.binary_dilation(occ, ndimage.generate_binary_structure(2, 1),
                                  iterations=rad)
print(f"[map] {w}x{h} / {res} m / origin ({ox:.4f}, {oy:.4f})")
print(f"[map] 占有 {occ.sum():,} → robot_radius {ROBOT_RADIUS} m 膨張後 "
      f"{blocked.sum():,} セル（自由 {(~blocked).sum():,}）")


def cell(x, y):
    return (h - 1 - int((y - oy) / res), int((x - ox) / res))


def bfs(a, b):
    """a→b の 4 連結経路長 [m]。届かなければ None。"""
    if blocked[a] or blocked[b]:
        return None
    dist = {a: 0}
    q = deque([a])
    while q:
        c = q.popleft()
        if c == b:
            return dist[c] * res
        for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + d[0], c[1] + d[1])
            if 0 <= n[0] < h and 0 <= n[1] < w and not blocked[n] and n not in dist:
                dist[n] = dist[c] + 1
                q.append(n)
    return None


wps = json.loads((R / "measure_20260908/waypoints.json").read_text())
cells = [cell(p["x"], p["y"]) for p in wps]
free = [not blocked[c] for c in cells]
print(f"\n[waypoint] 14 点のうち膨張後も自由なのは {sum(free)} 点")
for i, (p, f) in enumerate(zip(wps, free)):
    if not f:
        print(f"   ⚠️ {i:2d} ({p['x']:+6.2f}, {p['y']:+6.2f}) は膨張後に塞がる（使えない）")

rows = []
for (i, a), (j, b) in itertools.combinations(enumerate(wps), 2):
    straight = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
    if straight < MIN_STRAIGHT_M:
        continue
    if not (free[i] and free[j]):
        continue
    pl = bfs(cells[i], cells[j])
    rows.append((straight, pl, i, j))

rows.sort(reverse=True)
print(f"\n直線 {MIN_STRAIGHT_M:.0f} m 以上の組 {len(rows)} 通り"
      f"（余分 = BFS 経路 − マンハッタン下限。0 なら迂回ゼロ）")
print(" 直線[m] 経路[m]  下限[m] 余分[m] from → to")
ok = []
for s, pl, i, j in rows:
    if pl is None:
        print(f"  {s:6.2f}   届かない        "
              f"{i:2d}({wps[i]['x']:+6.2f},{wps[i]['y']:+6.2f}) → "
              f"{j:2d}({wps[j]['x']:+6.2f},{wps[j]['y']:+6.2f})")
        continue
    man = abs(wps[i]["x"] - wps[j]["x"]) + abs(wps[i]["y"] - wps[j]["y"])
    extra = pl - man
    mark = "  <= 迂回ゼロ" if extra < EXTRA_TOL_M else ""
    print(f"  {s:6.2f}  {pl:6.2f}  {man:6.2f}  {extra:6.2f}  "
          f"{i:2d}({wps[i]['x']:+6.2f},{wps[i]['y']:+6.2f}) → "
          f"{j:2d}({wps[j]['x']:+6.2f},{wps[j]['y']:+6.2f}){mark}")
    if extra < EXTRA_TOL_M:
        ok.append((s, pl, i, j))
print(f"\n== 直線 {MIN_STRAIGHT_M:.0f} m 以上で迂回ゼロ（下限 +{EXTRA_TOL_M} m 未満）: "
      f"{len(ok)} 通り ==")
for s, pl, i, j in ok:
    print(f"   直線 {s:5.2f} m / 経路 {pl:5.2f} m   "
          f"wp {i:2d} ({wps[i]['x']:+6.2f}, {wps[i]['y']:+6.2f}) → "
          f"wp {j:2d} ({wps[j]['x']:+6.2f}, {wps[j]['y']:+6.2f})")
