#!/usr/bin/env python3
"""占有格子から「浮いた孤立障害物」を消す。**実機不要。**

2026-09-24 に実機で `RegulatedPurePursuitController detected collision ahead!` を
繰り返して巡回が止まった（[../findings/real_run_20260924.md](../findings/real_run_20260924.md)）。
地図を数えたところ、**occupied の塊の 72〜80% が 2 セル以下の孤立点**で、
うち **491〜532 個が自由空間に浮いていた**。global costmap は静的地図しか見ないので、
1 セルでも膨張半径 0.40m を伴って現れ、経路上に乗れば「衝突」と判定される。
実際、機体の内接半径で膨張させると**通れる面積が 34〜41% 減る**。

    python3 denoise_map.py ../maps/grids/room_a_map_20260911.yaml \\
        --out ../maps/grids/room_a_map_20260911_denoised --overlay /tmp/removed.png

⚠️⚠️ **本物の細い柱・ポール・机の脚も消えうる。** 2 セル = 10cm なので、
細い実在物と区別できない。**必ず `--overlay` を見て、消したものを目で確認すること。**
消えて困るものがあるなら `--max-blob-cells` を下げる（1 なら単独セルだけ消す）。

📌 **消したセルは近傍の多数決で埋める**（free が多ければ free、unknown が多ければ unknown）。
一律 free にすると、未観測領域の中の孤立点が**通れる場所として捏造**されてしまう。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy import ndimage

FREE, OCC, UNKNOWN = 254, 0, 205   # map_server の慣例（free は 254 でも 255 でも可）


def load_pgm(path: Path) -> np.ndarray:
    with path.open("rb") as fp:
        magic = fp.readline().strip()
        if magic != b"P5":
            raise SystemExit(f"[denoise] P5(binary PGM)ではない: {magic!r}")
        dims = fp.readline()
        while dims.startswith(b"#"):
            dims = fp.readline()
        width, height = (int(v) for v in dims.split())
        maxval = int(fp.readline())
        if maxval > 255:
            raise SystemExit("[denoise] 16bit PGM は未対応")
        data = np.frombuffer(fp.read(width * height), dtype=np.uint8)
    return data.reshape(height, width)


def save_pgm(path: Path, grid: np.ndarray) -> None:
    with path.open("wb") as fp:
        fp.write(f"P5\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode("ascii"))
        fp.write(grid.astype(np.uint8).tobytes())


def stats(grid: np.ndarray, res: float) -> dict:
    occ, free = grid < 50, grid > 250
    inflated = ndimage.binary_dilation(
        occ, ndimage.generate_binary_structure(2, 2), iterations=4)   # 内接半径 0.20m
    passable = free & ~inflated
    lbl, n = ndimage.label(passable)
    sizes = ndimage.sum(passable, lbl, range(1, n + 1)) if n else np.array([0])
    return {
        "occupied": occ.mean(),
        "free": free.mean(),
        "unknown": (~occ & ~free).mean(),
        "passable_m2": float(sizes.max()) * res * res,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="占有格子の孤立した occupied を消す")
    ap.add_argument("map_yaml", type=Path, help="入力の .yaml（.pgm は yaml の image から引く）")
    ap.add_argument("--out", type=Path, required=True, help="出力の接頭辞（.pgm と .yaml を書く）")
    ap.add_argument("--max-blob-cells", type=int, default=2,
                    help="この大きさ以下の occupied の塊を消す（既定 2 ＝ 10cm 角まで）")
    ap.add_argument("--overlay", type=Path, default=None,
                    help="消したセルを赤で重ねた PNG を書く（**必ず見ること**）")
    args = ap.parse_args()

    meta = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8"))
    pgm = args.map_yaml.parent / meta["image"]
    grid = load_pgm(pgm)
    res = float(meta["resolution"])

    occ = grid < 50
    lbl, n = ndimage.label(occ, structure=np.ones((3, 3)))
    sizes = ndimage.sum(occ, lbl, range(1, n + 1))
    small_ids = np.where(sizes <= args.max_blob_cells)[0] + 1
    removed = np.isin(lbl, small_ids)

    # 近傍の多数決で埋める（一律 free にすると未観測の中に自由空間を捏造する）
    free_votes = ndimage.uniform_filter((grid > 250).astype(float), size=5)
    unknown_votes = ndimage.uniform_filter((~occ & (grid <= 250)).astype(float), size=5)
    out = grid.copy()
    out[removed & (free_votes >= unknown_votes)] = FREE
    out[removed & (free_votes < unknown_votes)] = UNKNOWN

    before, after = stats(grid, res), stats(out, res)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    save_pgm(args.out.with_suffix(".pgm"), out)
    meta_out = dict(meta)
    meta_out["image"] = args.out.with_suffix(".pgm").name
    args.out.with_suffix(".yaml").write_text(
        yaml.safe_dump(meta_out, default_flow_style=None, sort_keys=False), encoding="utf-8")

    print(f"[denoise] {args.map_yaml.name}: occupied の塊 {n} 個 → "
          f"**{len(small_ids)} 個（{args.max_blob_cells}セル以下）を消した**")
    for key, unit in (("occupied", "%"), ("free", "%"), ("unknown", "%")):
        print(f"[denoise]   {key:9s} {before[key]:6.1%} → {after[key]:6.1%}")
    print(f"[denoise]   **通れる面積(内接半径で膨張後) {before['passable_m2']:.1f} m² → "
          f"{after['passable_m2']:.1f} m²**（{after['passable_m2'] / max(before['passable_m2'], 1e-9):.0%}）")
    print(f"[denoise] 書き出し: {args.out.with_suffix('.pgm')}, {args.out.with_suffix('.yaml')}")

    if args.overlay:
        try:
            from PIL import Image
        except ImportError:
            print("[denoise] Pillow が無いので overlay は書けない", file=sys.stderr)
            return 0
        rgb = np.dstack([grid] * 3)
        # 消したセルは見落とさないよう膨らませて描く（1セルでは見えない）
        mark = ndimage.binary_dilation(removed, np.ones((3, 3)))
        rgb[mark] = [255, 0, 0]
        Image.fromarray(rgb.astype(np.uint8)).save(args.overlay)
        print(f"[denoise] 消したセルを赤で重ねた: {args.overlay}  ⚠️ **必ず目で確認すること**")
    return 0


if __name__ == "__main__":
    sys.exit(main())
