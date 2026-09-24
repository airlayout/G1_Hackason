#!/usr/bin/env python3
"""巡回路が地図の上で**成立するか**を机上で確かめる。**ROS も機体も要らない。**

各区間（点1→点2→…→点1）について、機体が入れる幅を考慮した経路を実際に引き、
**引けるか / 距離 / 所要時間**を出す。引けない区間はその場で分かる。

    python3 patrol_dryrun.py ../maps/grids/room_a_map_20260911_edited.yaml \\
        --waypoints ~/G1/_local/map_view/patrol/marks.json \\
        --out-yaml ../g1_ws/src/g1_navigation/config/patrol_room_a.yaml \\
        --png ~/G1/_local/map_view/patrol_dryrun.png

⚠️ **これは幾何の確認であって、Nav2 の確認ではない。** 実際の走行では
RPP の追従誤差・復帰動作・`collision ahead` が効くので、ここで通っても
モック（`backend:=mock`）や実機で詰まることはある。順番としては
「ここで通らないなら、その先は見るまでもない」という足切りに使う。

📌 **機体が入れる幅**は `inflation_radius` ではなく**内接半径**で見る
（既定 0.20m = footprint 0.5x0.4m の内接円）。Nav2 が「衝突」とみなすのはこの半径。
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage


def load_map(map_yaml: Path):
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    grid = np.array(Image.open(map_yaml.parent / meta["image"]))
    return grid, float(meta["resolution"]), float(meta["origin"][0]), float(meta["origin"][1])


def dijkstra(passable: np.ndarray, start: tuple[int, int], goal: tuple[int, int]):
    """8近傍のダイクストラ。戻り値は (経路, 長さ[セル])。届かなければ (None, inf)。"""
    h, w = passable.shape
    dist = np.full((h, w), np.inf, np.float32)
    prev = np.full((h, w, 2), -1, np.int32)
    dist[start] = 0.0
    pq = [(0.0, start)]
    diag = math.sqrt(2.0)
    while pq:
        d, (r, c) = heapq.heappop(pq)
        if (r, c) == goal:
            break
        if d > dist[r, c]:
            continue
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if not (0 <= nr < h and 0 <= nc < w) or not passable[nr, nc]:
                    continue
                nd = d + (diag if dr and dc else 1.0)
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    prev[nr, nc] = (r, c)
                    heapq.heappush(pq, (nd, (nr, nc)))
    if not np.isfinite(dist[goal]):
        return None, math.inf
    path = [goal]
    while path[-1] != start:
        r, c = path[-1]
        path.append(tuple(prev[r, c]))
    return path[::-1], float(dist[goal])


def main() -> int:
    ap = argparse.ArgumentParser(description="巡回路が地図の上で成立するかを机上で確かめる")
    ap.add_argument("map_yaml", type=Path)
    ap.add_argument("--waypoints", type=Path, required=True,
                    help="mark_map.py の marks.json、または patrol の yaml")
    ap.add_argument("--speed", type=float, default=0.30,
                    help="所要時間の見積りに使う速度[m/s]（既定 0.30 = max_vx）")
    ap.add_argument("--inscribed", type=float, default=0.20, help="機体の内接半径[m]（既定 0.20）")
    ap.add_argument("--snap", type=float, default=0.6,
                    help="点が入れない場所にあるとき、この距離[m]まで近くの通れるセルへ寄せる")
    ap.add_argument("--no-loop", action="store_true", help="最後の点から最初へ戻る区間を見ない")
    ap.add_argument("--out-yaml", type=Path, default=None, help="巡回路を yaml に書き出す")
    ap.add_argument("--png", type=Path, default=None, help="経路を重ねた絵を書き出す")
    args = ap.parse_args()

    grid, res, ox, oy = load_map(args.map_yaml)
    h, w = grid.shape
    occ, free = grid < 50, grid > 250
    cells = int(round(args.inscribed / res))
    infl = ndimage.binary_dilation(occ, ndimage.generate_binary_structure(2, 2), iterations=cells)
    passable = free & ~infl

    if args.waypoints.suffix == ".json":
        pts = [(m["x"], m["y"]) for m in
               json.loads(args.waypoints.read_text(encoding="utf-8"))["marks"]]
    else:
        pts = [(p["x"], p["y"]) for p in
               yaml.safe_load(args.waypoints.read_text(encoding="utf-8"))["waypoints"]]
    if len(pts) < 2:
        raise SystemExit("[dryrun] 点が 2 個以上要る")

    def to_rc(x, y):
        return h - 1 - int((y - oy) / res), int((x - ox) / res)

    # 通れない場所にある点は近くへ寄せる（クリックの誤差を救う）
    dist_to_pass, idx = ndimage.distance_transform_edt(~passable, return_indices=True)
    snapped = []
    for i, (x, y) in enumerate(pts):
        r, c = to_rc(x, y)
        r, c = min(max(r, 0), h - 1), min(max(c, 0), w - 1)
        if passable[r, c]:
            snapped.append(((r, c), 0.0))
            continue
        d = float(dist_to_pass[r, c]) * res
        nr, nc = int(idx[0][r, c]), int(idx[1][r, c])
        if d > args.snap:
            print(f"[dryrun] ⚠️ 点{i+1} ({x:.2f},{y:.2f}) は通れる場所から {d:.2f}m 離れている"
                  f"（許容 {args.snap}m）。**ここへは行けない**")
            snapped.append((None, d))
        else:
            print(f"[dryrun] 点{i+1} ({x:.2f},{y:.2f}) を {d:.2f}m 寄せた")
            snapped.append(((nr, nc), d))

    legs = list(zip(range(len(pts)), range(1, len(pts)))) + \
        ([] if args.no_loop else [(len(pts) - 1, 0)])
    total_len = 0.0
    ok = True
    paths = []
    print(f"\n{'区間':>8}  {'結果':6}  {'距離':>8}  {'所要(概算)':>10}")
    for a, b in legs:
        pa, pb = snapped[a][0], snapped[b][0]
        if pa is None or pb is None:
            print(f"{a+1:>3}→{b+1:<4}  ❌到達不可  （点が通れる場所にない）")
            ok = False
            continue
        path, cost = dijkstra(passable, pa, pb)
        if path is None:
            print(f"{a+1:>3}→{b+1:<4}  ❌経路なし  （通れる範囲が繋がっていない）")
            ok = False
            continue
        length = cost * res
        total_len += length
        paths.append(path)
        print(f"{a+1:>3}→{b+1:<4}  ✅通る    {length:>7.1f}m  {length/args.speed:>8.0f}秒")

    print(f"\n[dryrun] {'✅ 一周できる' if ok else '❌ 一周できない'}  "
          f"合計 {total_len:.1f}m / 約 {total_len/args.speed/60:.1f} 分"
          f"（{args.speed} m/s・停止や旋回の時間は含まない）")

    if args.out_yaml and ok:
        args.out_yaml.parent.mkdir(parents=True, exist_ok=True)
        body = {"frame_id": "map",
                "waypoints": [{"name": f"p{i+1}", "x": round(x, 2), "y": round(y, 2), "yaw_deg": 0}
                              for i, (x, y) in enumerate(pts)]}
        args.out_yaml.write_text(
            "# patrol_dryrun.py が書き出した巡回路。⚠️ **机上で経路が引けることまでしか"
            "確かめていない。**\n"
            "# 実機で使う前に、現地で record_waypoints.py を使って取り直すのが本筋。\n"
            + yaml.safe_dump(body, allow_unicode=True, sort_keys=False, default_flow_style=None),
            encoding="utf-8")
        print(f"[dryrun] 巡回路を書き出した: {args.out_yaml}")

    if args.png:
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib import font_manager
        for nm in ("Noto Sans CJK JP", "Noto Sans CJK SC"):
            if any(nm == f.name for f in font_manager.fontManager.ttflist):
                matplotlib.rcParams["font.family"] = nm
                break
        import matplotlib.pyplot as plt
        rgb = np.full((h, w, 3), 200, np.uint8)
        rgb[free] = (255, 226, 160)
        rgb[passable] = (255, 255, 255)
        rgb[occ] = (0, 0, 0)
        fig, ax = plt.subplots(figsize=(11, 15))
        # ⚠️ pgm は保存時に上下反転されている。表示で戻す
        ax.imshow(np.flipud(rgb), origin="lower",
                  extent=[ox, ox + w * res, oy, oy + h * res], interpolation="nearest")
        for path in paths:
            px = [ox + (c + 0.5) * res for _, c in path]
            py = [oy + (h - r - 0.5) * res for r, _ in path]
            ax.plot(px, py, "-", color="tab:blue", lw=2.0)
        for i, (x, y) in enumerate(pts):
            ax.plot(x, y, "o", ms=12, mfc="red", mec="white", mew=1.5)
            ax.annotate(str(i + 1), (x, y), color="red", fontsize=13, fontweight="bold",
                        xytext=(10, 8), textcoords="offset points")
        ax.set_title(f"巡回路の机上確認  合計 {total_len:.1f}m / 約 {total_len/args.speed/60:.1f} 分"
                     f"\n{'一周できる' if ok else '一周できない'}（青=引けた経路）")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(color="0.8", lw=0.3)
        fig.tight_layout()
        fig.savefig(args.png, dpi=130)
        print(f"[dryrun] 絵: {args.png}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
