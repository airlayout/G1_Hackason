"""試験用の人工の点群: 廊下の壁に押して開ける扉（板・左右の柱の段差・レバー）、廊下の反対の壁、床。

座標は扉の座標（原点＝開口の中心の床・+x＝押して入る向き・閉じた板の厚みの真ん中が x = 0）。
ライダーを扉の座標の姿勢 (x, y, yaw)・床からの高さ h・前後の傾き pitch に置き、上下逆さまの取付（x 軸まわり 180°）で
ライダーの座標の点にして返す。見えるかどうか（陰）は考えない（板の向こうの部屋の点は作らない）。
"""
from __future__ import annotations

import math

import numpy as np

WIDTH = 0.95
THICK = 0.04
JAMB_W = 0.08  # 柱の段差の幅
JAMB_PROUD = 0.03  # 柱が板の面から手前に出る量
BACK_WALL_X = -2.5  # 廊下の反対の壁
FLIP = np.diag([1.0, -1.0, -1.0])


def _plane(u0, u1, z0, z1, step, fn):
    u, z = np.meshgrid(np.arange(u0, u1, step), np.arange(z0, z1, step))
    return fn(u.ravel(), z.ravel())


def scene(angle_deg: float = 0.0, step: float = 0.01, lever: bool = True) -> np.ndarray:
    """扉の座標の点 [N, 3]"""
    face = -THICK / 2.0
    pts = []
    # 扉の板（廊下側の面）。蝶番は −y（把手は左）、回る軸は廊下側の面の蝶番の縁とする
    a = math.radians(angle_deg)
    hinge = np.array([face, -WIDTH / 2.0])
    along = np.array([math.sin(a), math.cos(a)])  # 閉じて +y、開くと +x（部屋）へ
    def slab(u, z):
        xy = hinge + np.outer(u, along)
        return np.c_[xy, z]
    pts.append(_plane(0.0, WIDTH, 0.0, 2.0, step, slab))
    if lever and angle_deg == 0.0:
        lu = np.arange(WIDTH - 0.064 - 0.14, WIDTH - 0.064, step)
        for dx in np.arange(0.04, 0.07, step):
            pts.append(np.c_[np.full_like(lu, face - dx), -WIDTH / 2.0 + lu, np.full_like(lu, 1.0)])
    # 柱の段差（左右）と、その外の壁（板の面と同じ x に戻る）
    for side in (+1, -1):
        y0 = side * WIDTH / 2.0
        pts.append(_plane(0.0, JAMB_W, 0.0, 2.0, step, lambda u, z: np.c_[np.full_like(u, face - JAMB_PROUD), y0 + side * u, z]))
        pts.append(_plane(JAMB_W, 2.5, 0.0, 2.0, step, lambda u, z: np.c_[np.full_like(u, face), y0 + side * u, z]))
    # 廊下の反対の壁と、廊下の端の壁（横の手がかり）
    pts.append(_plane(-3.0, 3.0, 0.0, 2.0, step, lambda u, z: np.c_[np.full_like(u, BACK_WALL_X), u, z]))
    pts.append(_plane(BACK_WALL_X, face, 0.0, 2.0, step, lambda u, z: np.c_[u, np.full_like(u, -3.0), z]))
    # 床
    fx, fy = np.meshgrid(np.arange(BACK_WALL_X, face, 0.015), np.arange(-3.0, 3.0, 0.015))
    pts.append(np.c_[fx.ravel(), fy.ravel(), np.zeros(fx.size)])
    return np.concatenate(pts)


def lidar_scan(world: np.ndarray, x: float, y: float, yaw: float, h: float = 1.21, pitch_deg: float = 0.0,
               noise: float = 0.0, rng: np.random.Generator | None = None, max_range: float = 6.0,
               roll_deg: float = 0.0) -> np.ndarray:
    """扉の座標の点を、(x, y, yaw)・高さ h・前傾 pitch・横の傾き roll（ZYX の順）に置いたライダーの座標（上下逆さまの取付）へ"""
    c, s = math.cos(yaw), math.sin(yaw)
    p = world - np.array([x, y, h])
    p = p @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # 世界 → 水平な機体の向き
    pt = math.radians(pitch_deg)
    cp, sp = math.cos(pt), math.sin(pt)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])  # 頭が前に傾く（下を向く）
    rr = math.radians(roll_deg)
    cr, sr = math.cos(rr), math.sin(rr)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    p = p @ (ry @ rx)  # 機体 → 傾いた頭（頭の向き = Rz(yaw)·Ry(pitch)·Rx(roll)）
    p = p @ FLIP.T  # 上下逆さまの取付
    r = np.linalg.norm(p, axis=1)
    keep = (r > 0.3) & (r < max_range)
    # 実物のように近いほど密に（1 m より遠い点は距離の 2 乗に反比例して間引く。床が近くでいちばん密になる）
    rng_keep = np.random.default_rng(12345)
    keep &= rng_keep.random(len(r)) < np.minimum(1.0, (1.0 / np.maximum(r, 1e-6)) ** 2)
    p = p[keep]
    if noise > 0:
        p = p + (rng or np.random.default_rng(0)).normal(0.0, noise, p.shape)
    return p
