#!/usr/bin/env python3
"""Isaac Sim を立てる**前に** spawn と `--range` を地図だけで決める。

## なぜ必要か

`IsaacSim_Env` の G1 は**指令ゼロでも -x へ約 3.4 cm/s 後退する**（機体の後ろ向き。
spawn の yaw は 0 なので world の -x）。Nav2 が使えるようになるまでの待ち時間ぶん
動くので、**測定開始点は spawn ではない**。実測 3 回で 1.74 / 2.26 / 2.27 m。

UiS_room_v3 の部屋の中の**最大 clearance は 2.85 m しか無い**。つまり
「clearance の大きい waypoint を spawn にする」だけでは足りない。
2026-09-08 に wp 4（clearance 2.12 m だが**半径 2 m のポケット**）を選んで、
-x へ 2.27 m 漂流した先の**余裕 0.22 m の壁際**から測り始めてしまい、
経路も `/cmd_vel`（vx=0.30）も出るのに機体だけ動かず 0/3 になった。

## 何を出すか

1. spawn から -x へ 3.5 m ぶんの余裕の掃引（漂流しても自由空間に居るか）
2. 漂流後の位置から `check_navigation.py --range` の**貪欲な選択を再現**して、
   選ばれる 3 本の脚それぞれの 4 連結経路長と迂回量を出す

⚠️ 迂回量の比較相手は**マンハッタン距離 |dx|+|dy|**（4 連結の下限）である。
「経路 ÷ 直線」は純粋な斜めでも 1.41 倍になるので迂回率ではない。
`check_long_routes.py` と同じ約束。

    .venv/bin/python Mapping/real/quickstart/pick_spawn.py
"""
from __future__ import annotations

import argparse
import json
import math
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

R = Path("Mapping/real/runs/20260906T135940_UiS_room_v3")
ROBOT_RADIUS = 0.25          # IsaacSim_Env/config/nav2.yaml の値
DRIFT_M = 2.3                # 起動待ちの漂流（実測 0.33 / 1.74 / 2.26 / 2.27 m の安全側）
SWEEP_MARGIN_M = 1.0         # 漂流の見積りを超えて動いた分も見る余白
EXTRA_TOL_M = 0.35           # マンハッタン下限との差がこれ未満なら迂回ゼロ


def load_map() -> tuple[np.ndarray, np.ndarray, float, float, float]:
    meta: dict[str, str] = {}
    for line in (R / "map/nav_map_clean.yaml").read_text().splitlines():
        if ":" in line:
            k, _, v = line.partition(":")
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    img = np.asarray(Image.open(R / "map/nav_map_clean.pgm"), dtype=np.float64)
    occ = ((255.0 - img) / 255.0) > float(meta["occupied_thresh"])
    return occ, np.asarray([ox, oy]), res, ox, oy


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--drift", type=float, default=DRIFT_M,
                    help="起動待ちに -x へ漂流する距離 [m]")
    ap.add_argument("--bands", type=float, nargs="*",
                    default=[12.0, 20.0, 15.0, 20.0],
                    help="試す --range の組（lo hi lo hi …）")
    ap.add_argument("--tries", type=int, default=3)
    args = ap.parse_args()

    occ, _, res, ox, oy = load_map()
    h, w = occ.shape
    # 各自由セルから最も近い占有セルまでの距離 [m]
    clear = ndimage.distance_transform_edt(~occ) * res
    # 機体の中心が入れないセル
    rad = int(math.ceil(ROBOT_RADIUS / res))
    blocked = ndimage.binary_dilation(
        occ, ndimage.generate_binary_structure(2, 1), iterations=rad)

    def cell(x: float, y: float) -> tuple[int, int]:
        return (h - 1 - int((y - oy) / res), int((x - ox) / res))

    def inside(c: tuple[int, int]) -> bool:
        return 0 <= c[0] < h and 0 <= c[1] < w

    def bfs(a: tuple[int, int], b: tuple[int, int]) -> float | None:
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
                if inside(n) and not blocked[n] and n not in dist:
                    dist[n] = dist[c] + 1
                    q.append(n)
        return None

    wps = json.loads((R / "measure_20260908/waypoints.json").read_text())
    print(f"[map] {w}x{h} / {res} m / origin ({ox:.4f}, {oy:.4f})")
    print(f"[drift] spawn から -x へ {args.drift} m 動いた先を「測定開始点」とする")

    sweep_m = args.drift + SWEEP_MARGIN_M

    def sweep(sx: float, sy: float) -> float:
        """spawn から -x へ「漂流 + 余白」ぶん動いたときの最小余裕 [m]。"""
        n = int(sweep_m / res)
        vals = []
        for k in range(n + 1):
            c = cell(sx - k * res, sy)
            vals.append(clear[c] if inside(c) else 0.0)
        return min(vals)

    def simulate(sx: float, sy: float, lo: float, hi: float):
        """漂流後の位置から check_navigation.py と同じ貪欲選択を再現する。"""
        px, py = sx - args.drift, sy
        c0 = cell(px, py)
        if not inside(c0) or blocked[c0]:
            got = clear[c0] if inside(c0) else 0.0
            return None, f"測定開始点が膨張後に塞がっている（余裕 {got:.2f} m）"
        legs = []
        for _ in range(args.tries):
            band = sorted((math.hypot(wp["x"] - px, wp["y"] - py), i, wp)
                          for i, wp in enumerate(wps)
                          if lo <= math.hypot(wp["x"] - px, wp["y"] - py) <= hi)
            if not band:
                return legs, f"{lo:.0f}〜{hi:.0f} m にゴール候補が無い"
            straight, i, wp = band[0]
            path = bfs(cell(px, py), cell(wp["x"], wp["y"]))
            if path is None:
                return legs, f"wp {i} へ 4 連結で届かない"
            manhattan = abs(wp["x"] - px) + abs(wp["y"] - py)
            # セル中心に丸める分だけ下限を割ることがあるので 0 で止める
            legs.append((i, straight, path, max(0.0, path - manhattan)))
            px, py = wp["x"], wp["y"]
        return legs, None

    bands = [(args.bands[k], args.bands[k + 1])
             for k in range(0, len(args.bands) - 1, 2)]
    rows, rejected = [], []
    for si, sp in enumerate(wps):
        corridor = sweep(sp["x"], sp["y"])
        for lo, hi in bands:
            legs, err = simulate(sp["x"], sp["y"], lo, hi)
            if err or not legs or len(legs) < args.tries:
                continue
            # ⚠️ 漂流通路の余裕は**フィルタ**である。並び順のタイブレークにすると、
            # 迂回ゼロだが壁に着く spawn を推してしまう（2026-09-08 に踏んだ）。
            if corridor < ROBOT_RADIUS:
                rejected.append((si, lo, hi, corridor))
                continue
            rows.append((max(l[3] for l in legs), -corridor, si, lo, hi,
                         corridor, legs))
    rows.sort()

    print(f"\n== spawn と --range の候補（迂回の小さい順）==")
    print(f"  漂流通路 = spawn から -x へ {sweep_m:.1f} m 動く間の最小余裕。"
          f"robot_radius {ROBOT_RADIUS} m 未満は除外した")
    print(f"{'最大迂回':>8} {'漂流通路':>8}  spawn                帯        "
          f"3 本の脚（wp / 直線 / 4連結 / 迂回）")
    for mx, _, si, lo, hi, corridor, legs in rows[:12]:
        legs_s = "  ".join(f"wp{i:<2d} {st:5.1f}/{p:5.1f}/+{ex:.2f}"
                           for i, st, p, ex in legs)
        mark = " <= 全部迂回ゼロ" if mx < EXTRA_TOL_M else ""
        print(f"{mx:8.2f} {corridor:8.2f}  wp{si:<2d}({wps[si]['x']:+6.2f},"
              f"{wps[si]['y']:+6.2f}) [{lo:.0f},{hi:.0f}]  {legs_s}{mark}")

    if rejected:
        worst = sorted(rejected, key=lambda r: r[3])[:4]
        print(f"\n  除外した {len(rejected)} 件のうち余裕の小さいもの: "
              + "、".join(f"wp{si}[{lo:.0f},{hi:.0f}] {c:.2f} m"
                          for si, lo, hi, c in worst))

    if not rows:
        print("  （候補なし。--drift を見直すか waypoints を増やす）")
        return

    _, _, si, lo, hi, corridor, legs = rows[0]
    sp = wps[si]
    print(f"\n== おすすめ ==")
    print(f"  SPAWN_X={sp['x']:.2f} SPAWN_Y={sp['y']:.2f}   "
          f"（clearance {sp['clearance']:.2f} m / "
          f"漂流通路の最小余裕 {corridor:.2f} m）")
    print(f"  --range {lo:.0f} {hi:.0f}")
    total = sum(l[2] for l in legs)
    print(f"  4 連結経路の合計 {total:.1f} m。実測 0.076 m/s（wall clock）だと "
          f"約 {total / 0.076:.0f} s")
    print(f"  ⚠️ --timeout は wall clock。1 本ぶんの最長 "
          f"{max(l[2] for l in legs) / 0.076:.0f} s に余裕を足して決める")


if __name__ == "__main__":
    main()
