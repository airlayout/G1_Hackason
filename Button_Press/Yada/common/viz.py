"""画像の見せ方の部品（サンプルの画像などで使う）。シミュレーターに依存しない。"""

from __future__ import annotations

import numpy as np

DEPTH_NEAR, DEPTH_FAR = 0.2, 1.5


def depth_to_rgb(depth: np.ndarray, near: float = DEPTH_NEAR, far: float = DEPTH_FAR) -> np.ndarray:
    """深度 [m] を色にする（近いほど明るい黄色、遠いほど暗い青。0 = 測れない画素は黒）。"""
    s = np.clip((far - depth) / (far - near), 0.0, 1.0)
    # 暗い青 → 緑 → 黄色の 3 色を線形につなぐ
    stops = np.array([[20, 20, 80], [30, 150, 120], [250, 230, 60]], dtype=float)
    x = s * 2.0
    i = np.clip(x.astype(int), 0, 1)
    f = (x - i)[..., None]
    rgb = stops[i] * (1 - f) + stops[i + 1] * f
    rgb[~(depth > 0)] = 0
    return rgb.astype(np.uint8)
