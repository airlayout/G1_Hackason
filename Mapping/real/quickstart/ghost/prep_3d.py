#!/usr/bin/env python3
"""ゴースト除去の前後を **3D ビューア用の PCD** に落とす（2026-09-13）。

`pcd_to_html.py --fragment` に食わせて作業ログに埋め込む。**床を z=0 に揃え**、
部屋の箱で切る（2026-09-12 の追記 2 の 3D ビューアと同じ流儀にして、並べて見られるようにする）。

    Navigation/.venv/bin/python quickstart/ghost/prep_3d.py <出力先>
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
REAL = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REAL / "python"))
from g1_mapping.rebuild import write_pcd            # noqa: E402
from score import WORK, SESSION, REGIONS            # noqa: E402

# 部屋だけを見る箱。天井は床上 2.0 m で切る（机とモニタは残る）
ROOM = dict(x=(-10.0, 24.0), y=(-18.0, 24.0), z=(-0.10, 2.00))
CASES = [
    ("前",      SESSION / "map" / "ref" / "map_full_raw.txt"),
    ("09-12案", WORK / "cand" / "all" / "1a_管A（09-12採用案）.txt"),
    ("今回",    WORK / "cand" / "all" / "z4_全部.txt"),
]


def clip(q: np.ndarray, box: dict) -> np.ndarray:
    m = np.ones(len(q), bool)
    for i, k in enumerate("xyz"):
        m &= (q[:, i] >= box[k][0]) & (q[:, i] <= box[k][1])
    return q[m]


def main() -> int:
    out = Path(sys.argv[1] if len(sys.argv) > 1 else WORK / "pcd3d")
    out.mkdir(parents=True, exist_ok=True)

    # ⚠️ 床は**前の点群 1 つで決めて 3 つに使い回す。**それぞれで 5 パーセンタイルを
    # 取ると、点を消したぶん床の推定がずれて 3 つの高さが揃わなくなる
    base = pd.read_csv(CASES[0][1], sep=r"\s+")[["x", "y", "z"]].to_numpy(np.float64)
    floor = float(np.percentile(base[:, 2], 5.0))
    print("床 z = {:+.3f} -> 0（3 つ共通）".format(floor))

    bx, by = REGIONS["B_開けた床"][:2], REGIONS["B_開けた床"][2:]
    zoomB = dict(x=(bx[0] - 1.0, bx[1] + 1.0), y=(by[0] - 1.0, by[1] + 1.0), z=(-0.10, 2.00))

    for label, path in CASES:
        p = pd.read_csv(path, sep=r"\s+")[["x", "y", "z"]].to_numpy(np.float64)
        p = p[np.isfinite(p).all(1)]
        p[:, 2] -= floor
        for tag, box in (("room", ROOM), ("zoomB", zoomB)):
            q = clip(p, box)
            f = out / "{}_{}.pcd".format(tag, label)
            write_pcd(f, q.astype(np.float32))
            print("  {:<6} {:<8} {:>9,} 点 -> {}".format(tag, label, len(q), f.name))
    print("\n次: pcd_to_html.py --fragment に食わせる")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
