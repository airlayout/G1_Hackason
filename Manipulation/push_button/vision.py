"""RGB-D のボタン検出枠を、腕が使う3D押下目標に変換する。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class ButtonTarget:
    # ボタン表面中心。座標系は T_base_optical の行き先。
    face_xyz: tuple[float, float, float]
    # ボタンを押し込む向き。単位ベクトル。
    press_direction: tuple[float, float, float]
    depth_m: float
    depth_valid_fraction: float
    plane_inlier_fraction: float


def validate_reachable_target(target: ButtonTarget) -> None:
    """現行の右腕と正面パネルの模擬作業範囲だけを受け入れる。"""
    x, y, z = target.face_xyz
    direction = np.asarray(target.press_direction)
    if (not np.all(np.isfinite((x, y, z)))
            or not (0.19 <= x <= 0.49 and -0.45 <= y <= -0.10
                    and 0.7 <= z <= 1.2)):
        raise ValueError("ボタン位置が右腕の作業範囲外です")
    if (direction.shape != (3,) or not np.all(np.isfinite(direction))
            or abs(np.linalg.norm(direction) - 1.0) > 0.02
            or direction[0] < 0.9):
        raise ValueError("押下方向が正面パネルの許容範囲外です")


def _points(pixels: np.ndarray, depths: np.ndarray,
            intr: CameraIntrinsics) -> np.ndarray:
    u, v = pixels[:, 0], pixels[:, 1]
    return np.column_stack(((u - intr.cx) * depths / intr.fx,
                            (v - intr.cy) * depths / intr.fy, depths))


def estimate_button_target(
    depth_m: np.ndarray,
    bbox: tuple[float, float, float, float],
    intrinsics: CameraIntrinsics,
    T_base_optical: np.ndarray,
    *,
    min_depth_m: float = 0.15,
    max_depth_m: float = 2.0,
) -> ButtonTarget:
    """カラー画素に位置合わせ済みの深度と、同じ画像の検出枠を使う。

    ボタン中心は検出枠中央の深度中央値から求める。押す方向は検出枠の周辺に
    見えるパネル面を RANSAC で推定する。深度欠損や面推定失敗時は押下しない。
    """
    depth = np.asarray(depth_m)
    transform = np.asarray(T_base_optical, dtype=float)
    if depth.ndim != 2 or not np.issubdtype(depth.dtype, np.number):
        raise ValueError("深度画像は2次元の数値配列が必要です")
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("T_base_optical は有限な4x4行列が必要です")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=0.02)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=0.02)
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-6)):
        raise ValueError("T_base_optical の回転または同次座標が不正です")
    if min(intrinsics.fx, intrinsics.fy) <= 0 or not all(
            np.isfinite((intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy))):
        raise ValueError("カメラ内部パラメータが不正です")
    height, width = depth.shape
    x1, y1, x2, y2 = map(float, bbox)
    if (not np.all(np.isfinite((x1, y1, x2, y2))) or x2 - x1 < 6
            or y2 - y1 < 6 or x1 < 0 or y1 < 0 or x2 > width or y2 > height):
        raise ValueError("ボタン検出枠が画像外または小さすぎます")

    center_u, center_v = (x1 + x2) / 2, (y1 + y2) / 2
    half_w, half_h = (x2 - x1) / 2, (y2 - y1) / 2
    ys, xs = np.mgrid[:height, :width]
    central = ((np.abs(xs - center_u) <= 0.35 * half_w)
               & (np.abs(ys - center_v) <= 0.35 * half_h))
    valid = np.isfinite(depth) & (depth >= min_depth_m) & (depth <= max_depth_m)
    central_valid = central & valid
    valid_fraction = float(central_valid.sum() / central.sum())
    if central_valid.sum() < 12 or valid_fraction < 0.6:
        raise ValueError("ボタン中央の有効深度が不足しています")
    face_depth = float(np.median(depth[central_valid]))
    face_optical = _points(np.array([[center_u, center_v]]),
                           np.array([face_depth]), intrinsics)[0]

    # 検出枠の外周だけを使い、ボタンの突出面がパネル平面に混ざらないようにする。
    radius_x = np.abs(xs - center_u) / half_w
    radius_y = np.abs(ys - center_v) / half_h
    ring = ((np.maximum(radius_x, radius_y) >= 1.15)
            & (np.maximum(radius_x, radius_y) <= 1.8) & valid)
    ring_pixels = np.column_stack((xs[ring], ys[ring]))
    ring_depths = depth[ring]
    if ring_pixels.shape[0] < 30:
        raise ValueError("ボタン周囲のパネル深度が不足しています")
    if ring_pixels.shape[0] > 4000:
        take = np.linspace(0, ring_pixels.shape[0] - 1, 4000, dtype=int)
        ring_pixels, ring_depths = ring_pixels[take], ring_depths[take]
    panel_points = _points(ring_pixels, ring_depths, intrinsics)

    rng = np.random.default_rng(0)
    best_inliers = np.zeros(len(panel_points), dtype=bool)
    for _ in range(100):
        sample = panel_points[rng.choice(len(panel_points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        inliers = np.abs((panel_points - sample[0]) @ normal) < 0.008
        if inliers.sum() > best_inliers.sum():
            best_inliers = inliers
    inlier_fraction = float(best_inliers.mean())
    if best_inliers.sum() < 30 or inlier_fraction < 0.55:
        raise ValueError("パネル面を安定して推定できません")
    fitted = panel_points[best_inliers]
    _, _, vh = np.linalg.svd(fitted - fitted.mean(axis=0), full_matrices=False)
    outward_optical = vh[-1]
    if outward_optical[2] > 0:
        outward_optical = -outward_optical
    if -outward_optical[2] < 0.6:
        raise ValueError("パネル面がカメラに対して斜めすぎます")
    press_base = rotation @ -outward_optical
    press_base /= np.linalg.norm(press_base)
    face_base = rotation @ face_optical + transform[:3, 3]
    return ButtonTarget(tuple(map(float, face_base)),
                        tuple(map(float, press_base)), face_depth,
                        valid_fraction, inlier_fraction)
