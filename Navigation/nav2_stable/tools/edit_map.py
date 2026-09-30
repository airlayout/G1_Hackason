#!/usr/bin/env python3
"""地図の一部を手で開ける／塞ぐ。**編集した場所は台帳に残す。**

地図が現況と食い違うとき（什器が動いた、収録中の人が焼き付いた等）に、
その区画だけを直す。2026-09-24 に room_a で「地図では塞がっているが実際は通路」
という指摘が出たのが発端。

    # 印(mark_map.py の marks.json)の範囲を開ける
    python3 edit_map.py ../maps/grids/room_a_map_20260911.yaml \\
        --clear-marks ~/G1/_local/map_view/marks.json --margin 0.5 \\
        --out ../maps/grids/room_a_map_20260911_edited --why "実地で通路と確認"

    # 座標で指定して開ける / 塞ぐ
    python3 edit_map.py <yaml> --clear-rect -0.7 -2.2 1.7 2.2 --out <prefix> --why "..."
    python3 edit_map.py <yaml> --block-rect 3.0 1.0 4.0 2.0 --out <prefix> --why "..."

## ⚠️ 危ないので必ず読むこと

- **塊ごと消してはいけない。** 実測では、印が触れていた occupied の塊は
  **6.0 × 15.5m** に伸びていた＝**壁**だった。だからこの道具は
  **矩形の内側しか触らない**（連結成分をたどらない）。
- **開けた場所に本当に物があると、地図上は通れることになる。** 実物は
  local costmap が見て止めるが、**global は知らないので迂回路を引けない**
  （global_costmap に obstacle_layer が無いため。D-21/D-24）。
- **編集は必ず台帳（`maps/grids/EDITS.md`）に残る。** 次に地図を取り直したとき、
  その場所が本当はどうだったのかを確認するため。
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

FREE, OCC = 254, 0


def load_pgm(path: Path) -> np.ndarray:
    with path.open("rb") as fp:
        if fp.readline().strip() != b"P5":
            raise SystemExit("[edit] P5(binary PGM)ではない")
        dims = fp.readline()
        while dims.startswith(b"#"):
            dims = fp.readline()
        w, h = (int(v) for v in dims.split())
        int(fp.readline())
        return np.frombuffer(fp.read(w * h), np.uint8).reshape(h, w)


def save_pgm(path: Path, grid: np.ndarray) -> None:
    with path.open("wb") as fp:
        fp.write(f"P5\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode("ascii"))
        fp.write(grid.astype(np.uint8).tobytes())


def passable_m2(grid: np.ndarray, res: float) -> float:
    occ, free = grid < 50, grid > 250
    infl = ndimage.binary_dilation(occ, ndimage.generate_binary_structure(2, 2), iterations=4)
    p = free & ~infl
    lbl, n = ndimage.label(p)
    if not n:
        return 0.0
    return float(ndimage.sum(p, lbl, range(1, n + 1)).max()) * res * res


def main() -> int:
    ap = argparse.ArgumentParser(description="地図の一部を開ける/塞ぐ（矩形のみ）")
    ap.add_argument("map_yaml", type=Path)
    ap.add_argument("--out", type=Path, required=True, help="出力の接頭辞")
    ap.add_argument("--clear-marks", type=Path, help="mark_map.py の marks.json。その外接矩形を開ける")
    ap.add_argument("--clear-rect", type=float, nargs=4, metavar=("X0", "Y0", "X1", "Y1"))
    ap.add_argument("--block-rect", type=float, nargs=4, metavar=("X0", "Y0", "X1", "Y1"))
    ap.add_argument("--radius", type=float, default=0.4,
                    help="印の**まわりだけ**この半径[m]を開ける（既定 0.4）。既定の動作")
    ap.add_argument("--use-bbox", action="store_true",
                    help="印の外接矩形をまとめて開ける。⚠️ **開け過ぎになりやすい**"
                         "（2026-09-24 に 25 個の印で 14.7x9.7m を対象にしてしまった）")
    ap.add_argument("--margin", type=float, default=0.5, help="--use-bbox のときの余白[m]")
    ap.add_argument("--why", required=True, help="**なぜ編集したか**。台帳に残る")
    ap.add_argument("--ledger", type=Path, default=None, help="既定は出力先と同じ場所の EDITS.md")
    args = ap.parse_args()

    meta = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8"))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    grid = load_pgm(args.map_yaml.parent / meta["image"])
    h, w = grid.shape
    before = passable_m2(grid, res)

    def to_slice(x0, y0, x1, y1):
        # pgm は上下反転（map_server 慣例: 原点は左下）
        c0, c1 = int((min(x0, x1) - ox) / res), int((max(x0, x1) - ox) / res) + 1
        r0 = h - 1 - int((max(y0, y1) - oy) / res)
        r1 = h - int((min(y0, y1) - oy) / res)
        return (slice(max(0, r0), min(h, r1)), slice(max(0, c0), min(w, c1)))

    ops = []
    if args.clear_marks:
        marks = json.loads(args.clear_marks.read_text(encoding="utf-8"))["marks"]
        if not marks:
            raise SystemExit("[edit] 印が0個")
        if args.use_bbox:
            xs = [m["x"] for m in marks]
            ys = [m["y"] for m in marks]
            rect = (min(xs) - args.margin, min(ys) - args.margin,
                    max(xs) + args.margin, max(ys) + args.margin)
            ops.append(("clear", rect, f"印 {len(marks)} 個の外接矩形 + 余白 {args.margin}m"))
        else:
            # ⚠️ **既定は印ごとの円。** 外接矩形は印が散っているとすぐ広がりすぎる。
            for m in marks:
                ops.append(("clear_disc", (m["x"], m["y"], args.radius),
                            f"印 {m['n']} の半径 {args.radius}m"))
    if args.clear_rect:
        ops.append(("clear", tuple(args.clear_rect), "座標指定"))
    if args.block_rect:
        ops.append(("block", tuple(args.block_rect), "座標指定"))
    if not ops:
        raise SystemExit("[edit] --clear-marks / --clear-rect / --block-rect のどれかが要る")

    # 円を切るための座標グリッド（セル中心の地図座標）
    cols = ox + (np.arange(w) + 0.5) * res
    rows = oy + (h - np.arange(h) - 0.5) * res
    gx, gy = np.meshgrid(cols, rows)

    out = grid.copy()
    records = []
    for kind, rect, note in ops:
        if kind == "clear_disc":
            cx, cy, rad = rect
            disc = (gx - cx) ** 2 + (gy - cy) ** 2 <= rad * rad
            target = disc & (out < 50)
            changed = int(target.sum())
            out[target] = FREE
            records.append({"kind": "clear_disc", "rect": [round(cx, 2), round(cy, 2), rad],
                            "cells": changed, "m2": round(changed * res * res, 2), "note": note})
            continue
        sl = to_slice(*rect)
        region = out[sl]
        if kind == "clear":
            changed = int((region < 50).sum())
            region[region < 50] = FREE      # ⚠️ occupied だけを触る。未観測はそのまま
        else:
            changed = int((region >= 50).sum())
            region[:] = OCC
        out[sl] = region
        records.append({"kind": kind, "rect": [round(v, 2) for v in rect],
                        "cells": changed, "m2": round(changed * res * res, 2), "note": note})
        print(f"[edit] {kind}: x {rect[0]:.2f}..{rect[2]:.2f} / y {rect[1]:.2f}..{rect[3]:.2f} "
              f"→ **{changed} セル ({changed*res*res:.2f} m²)** を変更（{note}）")

    if any(r["kind"] == "clear_disc" for r in records):
        tot = sum(r["cells"] for r in records)
        print(f"[edit] 印ごとの円: {len(records)} 箇所 / 合計 **{tot} セル "
              f"({tot*res*res:.2f} m²)** を開けた（半径 {args.radius}m）")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    save_pgm(args.out.with_suffix(".pgm"), out)
    meta_out = dict(meta)
    meta_out["image"] = args.out.with_suffix(".pgm").name
    args.out.with_suffix(".yaml").write_text(
        yaml.safe_dump(meta_out, default_flow_style=None, sort_keys=False), encoding="utf-8")
    after = passable_m2(out, res)
    print(f"[edit] 通れる面積(内接半径で膨張後) {before:.1f} m² → **{after:.1f} m²**")
    print(f"[edit] 書き出し: {args.out.with_suffix('.pgm')}, {args.out.with_suffix('.yaml')}")

    ledger = args.ledger or args.out.parent / "EDITS.md"
    if not ledger.exists():
        ledger.write_text("# 地図の手編集の台帳\n\n"
                          "⚠️ **ここに載っている編集は「人がそう判断した」だけで、"
                          "測ったわけではない。** 地図を取り直したら、必ずこの一覧の場所を"
                          "確認してから台帳を畳むこと。\n", encoding="utf-8")
    with ledger.open("a", encoding="utf-8") as fp:
        fp.write(f"\n## {dt.date.today()} {args.out.name}\n\n")
        fp.write(f"- 元: `{args.map_yaml.name}`\n- 理由: **{args.why}**\n")
        for r in records:
            if r["kind"] == "clear_disc":
                fp.write(f"- clear_disc: 中心 ({r['rect'][0]}, {r['rect'][1]}) "
                         f"半径 {r['rect'][2]}m → {r['cells']} セル ({r['m2']} m²)"
                         f"（{r['note']}）\n")
            else:
                fp.write(f"- {r['kind']}: x {r['rect'][0]}..{r['rect'][2]} / "
                         f"y {r['rect'][1]}..{r['rect'][3]} → {r['cells']} セル ({r['m2']} m²)"
                         f"（{r['note']}）\n")
        fp.write(f"- 通れる面積: {before:.1f} → {after:.1f} m²\n")
    print(f"[edit] 台帳に追記: {ledger}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
