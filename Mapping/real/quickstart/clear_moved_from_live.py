#!/usr/bin/env python3
"""**実機の LiDAR が「もう無い」と確かめた占有を、走る地図から消す。**（Mac で動かす）

    G1_Hackason/.venv/bin/python quickstart/clear_moved_from_live.py \\
        --nav-map runs/<id>/map/nav_map_run.yaml \\
        --seed-projected runs/<id>/map/seed_projected.yaml \\
        --live runs/_live/<stamp>/live_projected_map.npz \\
        --out runs/<id>/map/nav_map_run.pgm

## 何をするか（2026-09-28）

`octomap_server` は種から始めて、レイが通ったボクセルを空きに書き換える。
走らせた後の `/projected_map`（live）と、起動時の種を 2D に落とした物（seed_projected）を比べると、
**「種では占有・今日は空き」＝記録のあとに動かされて、今は無い物**が分かる。
その塊だけを `nav_map_run` から消す。

- **消すだけで、足さない。** live で増えた占有（家具の移動先など）は入れない
  （ユーザー判断。足すと追従者や一時的な物まで焼き込む）
- **塊で判定する。** `--min-cells` 未満の塊は位相の刻みやノイズとして触らない
- 格子は live と seed_projected が同じ（octomap の格子）。`nav_map_run` は原点に端数があり
  位相が (−0.004, +0.028) m ずれるので、**nav_map_run のセル中心が落ちる live のセル**で引く

⚠️ live の空きは「今日そこをレイが通った」証拠。**見ていない所（-1）は消さない。**
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage


def load_pgm_yaml(path: Path) -> "tuple[np.ndarray, float, float, float, Path]":
    y = yaml.safe_load(path.read_text())
    img_path = path.parent / y["image"]
    img = np.array(Image.open(img_path))
    return img, float(y["resolution"]), float(y["origin"][0]), float(y["origin"][1]), img_path


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--nav-map", type=Path, required=True, help="消す相手の nav_map_run.yaml")
    p.add_argument("--seed-projected", type=Path, required=True, help="起動時の種を 2D に落とした .yaml")
    p.add_argument("--live", type=Path, required=True, help="走らせた後の /projected_map（npz: grid,res,ox,oy）")
    p.add_argument("--out", type=Path, required=True, help="書き出す .pgm（yaml は nav-map のものをそのまま使う）")
    p.add_argument("--min-cells", type=int, default=3)
    a = p.parse_args()

    nav, res, nox, noy, _ = load_pgm_yaml(a.nav_map)
    seed_img, sres, sox, soy, _ = load_pgm_yaml(a.seed_projected)
    live = np.load(a.live)
    g = live["grid"]
    lres, lox, loy = float(live["res"]), float(live["ox"]), float(live["oy"])
    if not (abs(sres - lres) < 1e-6 and abs(sox - lox) < 1e-6 and abs(soy - loy) < 1e-6):
        print(f"[clear] ⛔ seed_projected と live の格子が違う: ({sres},{sox},{soy}) 対 ({lres},{lox},{loy})",
              file=sys.stderr)
        return 2
    seed_occ = np.flipud(seed_img) < 100                 # 行 0 = y の小さい側
    h, w = min(g.shape[0], seed_occ.shape[0]), min(g.shape[1], seed_occ.shape[1])
    g, seed_occ = g[:h, :w], seed_occ[:h, :w]
    gone = seed_occ & (g >= 0) & (g < 25)                # 種では占有・今日は空き（見た所だけ）
    lab, n = ndimage.label(gone, structure=np.ones((3, 3)))
    sizes = ndimage.sum(gone, lab, range(1, n + 1))
    gone_big = np.isin(lab, np.flatnonzero(sizes >= a.min_cells) + 1)

    # nav_map_run の各セル中心 → live の格子
    H, W = nav.shape
    cx = nox + (np.arange(W) + 0.5) * res
    cy = noy + (H - 1 - np.arange(H) + 0.5) * res        # PGM は行 0 が y の大きい側
    li = np.floor((cy[:, None] - loy) / lres).astype(int) * np.ones((1, W), int)
    lj = np.floor((cx[None, :] - lox) / lres).astype(int) * np.ones((H, 1), int)
    inside = (li >= 0) & (li < h) & (lj >= 0) & (lj < w)
    hit = np.zeros_like(inside)
    hit[inside] = gone_big[li[inside], lj[inside]]
    occ = nav < 100
    clear = occ & hit
    out = nav.copy()
    out[clear] = 254

    a.out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(out).save(a.out)
    print(f"[clear] 消えた塊 {int((sizes >= a.min_cells).sum())} 個 / {int(gone_big.sum())} セル（live の格子）")
    print(f"[clear] nav_map_run の占有 {int(occ.sum()):,} → {int((out < 100).sum()):,}"
          f"（消した {int(clear.sum()):,} セル）→ {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
