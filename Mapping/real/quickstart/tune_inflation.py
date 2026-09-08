#!/usr/bin/env python3
"""長距離が着かない原因を地図だけで切り分け、`inflation_radius` を決める。

## 何が分かるのか

2026-09-08、長距離（直線 15 m 級）が 0/3 だった。測位のずれは中央 0.2 mm、
経路追従の横ずれも中央 0.03 m で、**どちらも無罪**。落ちていたのは
**経路が余裕 0.20〜0.36 m の隙間を選ぶこと**である。

このスクリプトは 2 つを出す。

1. **最大ボトルネック幅**（widest path）— A→B で「一番狭い所」を最大化する道の幅。
   これが実測の経路より広ければ、**部屋の制約ではなくプランナの選び方の問題**である
2. **`inflation_radius` / `cost_scaling_factor` の掃引** — 設定ごとに
   NavFn が選ぶ経路の長さ・最狭部・狭い所を通る長さ

## なぜ `inflation_radius` が効くのか

`inflation_layer` は **`inflation_radius` を超えた所のコストを 0 にする**。
既定の 0.35 m では、余裕 0.36 m の廊下と 0.60 m の廊下が NavFn には
**まったく同じ（コスト 0）に見える**。だから幾何的に短い方＝狭い方を選ぶ。
`inflation_radius` を広げると、初めて「広い方が安い」になる。

## ⚠️ `robot_radius` を上げるのは筋が違う

NavFn は**障害物セル自体（254）しか塞がない**。`robot_radius` 以内は
inscribed（253）＝「高いコスト」で、**通れてしまう**
（実測でも余裕 0.20 m の所を通っていた）。上げてもその経路は消えず、
一方で膨張後の自由空間が削れて**到達できない waypoint が出る**
（この部屋は自由セルの 28.6 % が既に 0.25 m 未満）。

## ⚠️ local costmap の inflation は上げない

`RegulatedPurePursuitController` の `simulate_ahead_time` の衝突判定は
local costmap を見る。上げると少し狭い所で動けなくなる。上げるのは
**global だけ**である。

    .venv/bin/python Mapping/real/quickstart/tune_inflation.py
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

R = Path("Mapping/real/runs/20260906T135940_UiS_room_v3")
# navfn の内部定数（1 セルの通過コスト = NEUTRAL + FACTOR * costmap のコスト）
COST_NEUTRAL, COST_FACTOR = 50.0, 0.8
LETHAL, INSCRIBED, INFLATED_MAX = 254.0, 253.0, 252.0
TIGHT_M = 0.50               # 余裕がこれ未満の所を「狭い」と数える
NB = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
      (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142))
# 既定と提案（2026-09-08 の掃引で決めた）
CURRENT = (0.25, 0.35, 5.0)
PROPOSED = (0.25, 0.80, 3.0)


def load():
    meta: dict[str, str] = {}
    for line in (R / "map/nav_map_clean.yaml").read_text().splitlines():
        if ":" in line:
            k, _, v = line.partition(":")
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    img = np.asarray(Image.open(R / "map/nav_map_clean.pgm"), dtype=np.float64)
    occ = ((255.0 - img) / 255.0) > float(meta["occupied_thresh"])
    clear = ndimage.distance_transform_edt(~occ) * res
    return occ, clear, res, ox, oy


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pairs", type=int, nargs="*", default=None,
                    help="waypoint の組（i j i j …）。既定は長距離の代表 6 組")
    args = ap.parse_args()

    occ, clear, res, ox, oy = load()
    h, w = occ.shape

    def cell(x: float, y: float) -> tuple[int, int]:
        return (h - 1 - int((y - oy) / res), int((x - ox) / res))

    def bottleneck(a, b, lo=0.10, hi=3.0) -> float:
        """a→b の最大ボトルネック幅 [m]。二分探索 + 連結成分。"""
        if clear[a] < lo or clear[b] < lo:
            return 0.0
        best = 0.0
        for _ in range(24):
            mid = (lo + hi) / 2
            lab, _ = ndimage.label(clear >= mid,
                                   structure=ndimage.generate_binary_structure(2, 1))
            if lab[a] and lab[a] == lab[b]:
                best, lo = mid, mid
            else:
                hi = mid
        return best

    def cost_field(inscribed: float, infl_r: float, scaling: float) -> np.ndarray:
        c = np.zeros_like(clear)
        c[occ] = LETHAL
        m = (~occ) & (clear <= inscribed)
        c[m] = INSCRIBED
        m2 = (~occ) & (clear > inscribed) & (clear <= infl_r)
        c[m2] = INFLATED_MAX * np.exp(-scaling * (clear[m2] - inscribed))
        return c

    def plan(a, b, cost) -> list | None:
        """navfn 相当の最小コスト経路。⚠️ 障害物セルだけ通行不可（253 は通る）。"""
        trav = COST_NEUTRAL + COST_FACTOR * cost
        if occ[a] or occ[b]:
            return None
        dist = np.full(clear.shape, math.inf)
        prev: dict = {}
        dist[a] = 0.0
        pq = [(0.0, a)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == b:
                break
            for dr, dc, k in NB:
                v = (u[0] + dr, u[1] + dc)
                if not (0 <= v[0] < h and 0 <= v[1] < w) or occ[v]:
                    continue
                nd = d + k * trav[v]
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if dist[b] == math.inf:
            return None
        path, c = [], b
        while c != a:
            path.append(c)
            c = prev[c]
        path.append(a)
        return path[::-1]

    def stats(p) -> tuple[float, float, float]:
        cs = np.array([clear[c] for c in p])
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(p, p[1:])) * res
        return length, float(cs.min()), res * float((cs < TIGHT_M).sum())

    wps = json.loads((R / "measure_20260908/waypoints.json").read_text())
    pairs = ([(args.pairs[k], args.pairs[k + 1])
              for k in range(0, len(args.pairs) - 1, 2)] if args.pairs
             else [(7, 2), (2, 13), (4, 7), (2, 7), (6, 12), (4, 9)])

    # ⚠️ 「自由セルの何 %」は範囲で大きく変わる。地図には建物の外の広い所も
    # 入っているので、部屋の中（waypoint が居る範囲）に限った値も併記する。
    ys, xs = np.nonzero(~occ)
    wx = ox + xs * res
    wy = oy + (h - 1 - ys) * res
    room = (wx >= -2.0) & (wx <= 19.0) & (wy >= -12.0) & (wy <= 19.0)
    allc = clear[~occ]
    roomc = allc[room]
    print(f"[map] {w}x{h} / {res} m")
    print(f"[map] 自由セルの余裕（地図全体 {len(allc):,} セル）: "
          f"< 0.25 m が {100 * (allc < 0.25).mean():.1f} %、"
          f"< 0.50 m が {100 * (allc < 0.50).mean():.1f} %")
    print(f"[map] 同（部屋の中 x -2〜19 / y -12〜19 の {len(roomc):,} セル）: "
          f"< 0.25 m が {100 * (roomc < 0.25).mean():.1f} %、"
          f"< 0.50 m が {100 * (roomc < 0.50).mean():.1f} %、"
          f"< 1.00 m が {100 * (roomc < 1.00).mean():.1f} %")

    print("\n== ① 最大ボトルネック幅（この幅の道が在る ＝ 上限）==")
    for i, j in pairs:
        a, b = cell(wps[i]["x"], wps[i]["y"]), cell(wps[j]["x"], wps[j]["y"])
        st = math.hypot(wps[i]["x"] - wps[j]["x"], wps[i]["y"] - wps[j]["y"])
        print(f"  wp{i:<2d}→wp{j:<2d}  直線 {st:5.2f} m  最大ボトルネック "
              f"{bottleneck(a, b):.2f} m")

    print(f"\n== ② inflation の掃引（区間 wp{pairs[0][0]}→wp{pairs[0][1]}）==")
    a0 = cell(wps[pairs[0][0]]["x"], wps[pairs[0][0]]["y"])
    b0 = cell(wps[pairs[0][1]]["x"], wps[pairs[0][1]]["y"])
    print(f"{'robot_r':>8}{'inflation_r':>12}{'scaling':>9} | "
          f"{'経路長':>8}{'最狭部':>8}{'狭い所':>8}")
    for cfg in ((0.25, 0.35, 5.0), (0.25, 0.55, 5.0), (0.25, 0.80, 3.0),
                (0.25, 1.00, 3.0), (0.25, 1.50, 1.5), (0.35, 0.80, 3.0)):
        p = plan(a0, b0, cost_field(*cfg))
        if p is None:
            print(f"{cfg[0]:8.2f}{cfg[1]:12.2f}{cfg[2]:9.1f} | 経路なし")
            continue
        L, mn, tight = stats(p)
        tag = ("  ← いまの設定" if cfg == CURRENT
               else "  ← 提案" if cfg == PROPOSED else "")
        print(f"{cfg[0]:8.2f}{cfg[1]:12.2f}{cfg[2]:9.1f} | "
              f"{L:7.2f}m{mn:7.2f}m{tight:7.1f}m{tag}")

    print(f"\n== ③ 提案設定を全区間で確かめる "
          f"（inflation {PROPOSED[1]} / scaling {PROPOSED[2]}）==")
    cur, new = cost_field(*CURRENT), cost_field(*PROPOSED)
    tot_c = tot_n = 0.0
    print(f"{'区間':12s} | {'いま 長さ/最狭/狭い所':>24} | "
          f"{'提案 長さ/最狭/狭い所':>24} | 長さの増分")
    for i, j in pairs:
        a, b = cell(wps[i]["x"], wps[i]["y"]), cell(wps[j]["x"], wps[j]["y"])
        pc, pn = plan(a, b, cur), plan(a, b, new)
        if pc is None or pn is None:
            print(f"wp{i}→wp{j}: 経路なし")
            continue
        Lc, mc, tc = stats(pc)
        Ln, mn, tn = stats(pn)
        tot_c += Lc
        tot_n += Ln
        print(f"wp{i:<2d}→wp{j:<2d}     | {Lc:7.2f}m {mc:5.2f}m {tc:5.1f}m       | "
              f"{Ln:7.2f}m {mn:5.2f}m {tn:5.1f}m       | {100 * (Ln / Lc - 1):+6.1f} %")
    print(f"{'合計':12s} | {tot_c:7.2f}m                  | {tot_n:7.2f}m"
          f"                  | {100 * (tot_n / tot_c - 1):+6.1f} %")

    print("\n== ④ 副作用: 提案設定で到達できなくなる waypoint ==")
    hub = np.unravel_index(int(np.argmax(np.where(~occ, clear, 0))), clear.shape)
    bad = [i for i, wp in enumerate(wps)
           if plan(hub, cell(wp["x"], wp["y"]), new) is None]
    print(f"  {len(wps)} 点中 到達不能 {len(bad)} 点"
          + (f" → {bad}" if bad else "（inflation を上げても到達性は落ちない）"))


if __name__ == "__main__":
    main()
