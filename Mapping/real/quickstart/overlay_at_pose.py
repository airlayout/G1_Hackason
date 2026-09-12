#!/usr/bin/env python3
"""**1 つの姿勢**での重畳を測る。再定位が返した姿勢の裏を取るため。

## なぜ要るのか

`measure_overlay.py` は bag の `/tf`（＝測位器が出した姿勢）で点群を地図に載せる。
だが再定位の検証では「**測位器とは別の姿勢**を当てたらどうなるか」を知りたい。
とくに歩行の記録には真値が無いので、**ゲート（残差）とは独立な物差し**が要る
（計画書 合否 2: 旧 `nav_map` 基準で重畳 ≥ 78%）。

残差ゲートと重畳は別の地図を見ている（残差は 3D の `map.mm`、重畳は 2D の `nav_map`）ので、
**片方が壊れてももう片方で気づける**。

使い方:
    Navigation/.venv/bin/python quickstart/overlay_at_pose.py \\
        runs/reloc_eval_20260911/walk_r1.xyz \\
        runs/20260906T135940_UiS_room_v3/map/nav_map_ref.yaml \\
        --pose 1.033 0.141 -30.0
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from measure_overlay import read_map  # noqa: E402

# measure_overlay.py と同じ帯を使う（別の値でごまかさない）
BAND_LO, BAND_HI = 0.02, 1.82


def overlay(points: np.ndarray, pose, occ, res, ox, oy, tol_cells: int = 1) -> dict:
    """base_link 系の点を pose に置いて、占有セルに乗った割合を返す。"""
    x, y, yaw_deg = pose
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    p = points[(points[:, 2] >= BAND_LO) & (points[:, 2] <= BAND_HI)]
    if len(p) == 0:
        raise SystemExit("帯 {}..{} m に点が無い".format(BAND_LO, BAND_HI))
    wx = x + c * p[:, 0] - s * p[:, 1]
    wy = y + s * p[:, 0] + c * p[:, 1]
    col = np.floor((wx - ox) / res).astype(int)
    iy = np.floor((wy - oy) / res).astype(int)
    h, w = occ.shape
    inside = (col >= 0) & (col < w) & (iy >= 0) & (iy < h)
    # ⚠️ **上下反転が要る。** `measure_overlay.read_map` は PGM を反転せずに返すので、
    #    行の添字は h-1-iy になる（`check_map_clearance.load_map` は反転済みで返す。
    #    同じリポジトリに 2 つの流儀があるので、どちらを import したかで変わる）。
    #    2026-09-11 にこれを落として 78.2% を 35.3% と出した
    row = h - 1 - iy
    k = 2 * tol_cells + 1
    occ_tol = binary_dilation(occ, np.ones((k, k), bool))
    hit = int(occ[row[inside], col[inside]].sum())
    hit_tol = int(occ_tol[row[inside], col[inside]].sum())
    n = int(inside.sum())
    return {"points": len(p), "inside": n,
            "hit_pct": 100.0 * hit / n if n else 0.0,
            "hit_tol_pct": 100.0 * hit_tol / n if n else 0.0}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scan", type=Path, help="base_link 系の XYZ（export_scan.py の出力）")
    ap.add_argument("map_yaml", type=Path, help="⚠️ 間引いていない旧 nav_map を渡す")
    ap.add_argument("--pose", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW_DEG"))
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

    occ, res, ox, oy = read_map(a.map_yaml)
    pts = np.loadtxt(a.scan)
    r = overlay(pts, tuple(a.pose), occ, res, ox, oy)
    if a.quiet:
        print("{:.1f}".format(r["hit_pct"]))
    else:
        print("基準地図 {} / 占有 {} セル".format(a.map_yaml, int(occ.sum())))
        print("姿勢 ({:.3f}, {:.3f}, {:.2f} deg) / 帯 {}..{} m の {} 点（枠内 {}）".format(
            a.pose[0], a.pose[1], a.pose[2], BAND_LO, BAND_HI, r["points"], r["inside"]))
        print("  占有セルに乗った割合          {:.1f} %".format(r["hit_pct"]))
        print("  ±1 セル(10cm) まで許した割合  {:.1f} %".format(r["hit_tol_pct"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
