#!/usr/bin/env python3
"""計測一式に入っている `quicklook_cloud_odom.npz`(odom で融合済みの点群)を .pcd にする。

Mapping トラックの収録スクリプトは、生の Livox スキャンを odom で融合した点群を
`quicklook_cloud_odom.npz`(`xyz` 配列)として残す。**これをそのまま
`pointcloud_to_occupancy_grid.py` の入力にできる**ので、bag から点群を取り出して
自分で融合し直す必要がない（2026-09-24）。

使い方:

    python3 quicklook_npz_to_pcd.py <計測一式>/quicklook_cloud_odom.npz \\
        --out ../maps/clouds/room_b_sorasta_20260923.pcd

⚠️ **SLAM 補正は入っていない**（収録側の note のとおり）。融合に使った odom の
ドリフトと LiDAR 外部パラメータの誤差がそのまま乗る。壁が二重に見えるなら
そこを疑うこと。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description="quicklook の npz 点群を .pcd にする")
    ap.add_argument("npz", type=Path, help="quicklook_cloud_odom.npz")
    ap.add_argument("--out", type=Path, required=True, help="出力する .pcd")
    ap.add_argument("--key", default="xyz", help="npz の中の配列名(既定: xyz)")
    args = ap.parse_args()

    data = np.load(args.npz)
    if args.key not in data.files:
        raise SystemExit(f"[pcd] {args.key} が無い。入っているのは: {data.files}")
    xyz = np.asarray(data[args.key], dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise SystemExit(f"[pcd] 形が (N,3) でない: {xyz.shape}")
    xyz = xyz[np.isfinite(xyz).all(axis=1)]

    count = len(xyz)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("wb") as fp:
        fp.write(header.encode("ascii"))
        fp.write(xyz.tobytes())

    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    print(f"[pcd] {count} 点 → {args.out}")
    print(f"[pcd] 範囲: x {lo[0]:.2f}..{hi[0]:.2f} / y {lo[1]:.2f}..{hi[1]:.2f} "
          f"/ z {lo[2]:.2f}..{hi[2]:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
