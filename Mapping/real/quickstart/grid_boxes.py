#!/usr/bin/env python3
"""占有グリッドを箱のリストにする。**numpy 以外に依存しない。**

`pcd_to_mjcf.py`（MuJoCo 用）と `IsaacSim_Env/src/build_scene_usd.py`
（Isaac Sim 用）の両方がこれを使う。**同じ npz から同じ箱が出ることが
両者の前提**なので、実装を 2 つに分けない。

⚠️ Isaac Sim の入っている機械には open3d / scipy が無い。
`pcd_to_mjcf.py` はそれらを import するので、あちらから読み込むと落ちる。
このモジュールを numpy だけに保つのはそのため。
"""
from __future__ import annotations

import numpy as np


def boxes_from_grid(mask: np.ndarray, level: np.ndarray, cell: float,
                    origin: np.ndarray) -> "list[tuple[float, float, float, float, float]]":
    """行ごとに、同じ高さ段の連続セルを1つの箱へまとめる（行方向のランレングス）。

    高さを量子化してから繋げるのが要点。生の最大高さで繋げると1セルごとに段が
    変わって箱が減らない。

    Returns:
        (中心 x, 中心 y, 中心 z, 半幅 x, 半高 z) の並び。半幅 y は cell / 2。
    """
    boxes = []
    nrow, ncol = mask.shape
    for row in range(nrow):
        occupied_row, level_row = mask[row], level[row]
        col = 0
        while col < ncol:
            if not occupied_row[col]:
                col += 1
                continue
            height = level_row[col]
            end = col
            while end + 1 < ncol and occupied_row[end + 1] and level_row[end + 1] == height:
                end += 1
            width = end - col + 1
            boxes.append((
                float(origin[0] + (col + width / 2.0) * cell),   # 中心 x
                float(origin[1] + (row + 0.5) * cell),           # 中心 y
                float(height / 2.0),                             # 中心 z
                float(width * cell / 2.0),                       # 半幅 x
                float(height / 2.0),                             # 半高 z
            ))
            col = end + 1
    return boxes
