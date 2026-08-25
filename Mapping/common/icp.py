"""ICP（Iterative Closest Point）によるスキャンの位置合わせ。

自己位置推定の中核。point-to-point 版（Kabsch/SVD で剛体変換を解く）を使っている。
point-to-plane のほうが収束は速いが、地図側の法線を毎フレーム再計算する必要があり、
scipy だけで実装すると法線推定のほうが ICP 本体より重くなる。今回の対象は
壁・柱・箱で幾何拘束が十分な屋内なので point-to-point で足りると判断した。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class IcpResult:
    """ICP の結果。`pose` は source を target に重ねる 4x4 同次変換。"""

    pose: np.ndarray
    fitness: float
    """対応点が見つかった source 点の割合。低いと位置合わせが信用できない。"""
    inlier_rmse: float
    """対応点間距離の RMSE[m]。"""
    n_iterations: int


def _kabsch(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """対応の付いた 2 つの点群から、src を dst に重ねる剛体変換を最小二乗で解く。"""
    src_c = src.mean(axis=0)
    dst_c = dst.mean(axis=0)
    h = (src - src_c).T @ (dst - dst_c)
    u, _, vt = np.linalg.svd(h)
    # det<0 だと鏡映（左手系への反転）になってしまうので、最小特異値の軸を反転して防ぐ
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    pose = np.eye(4)
    pose[:3, :3] = rot
    pose[:3, 3] = dst_c - rot @ src_c
    return pose


def icp(
    source: np.ndarray,
    target_tree: cKDTree,
    target_points: np.ndarray,
    init_pose: np.ndarray,
    max_correspondence_dist: float = 1.0,
    max_iterations: int = 30,
    tolerance: float = 1e-4,
) -> IcpResult:
    """`source`（センサー座標系の点群）を `target_points`（ワールド座標系の地図）に合わせる。

    `init_pose` は初期推定。ICP は局所最適解しか探さないので、ここが実際の姿勢から
    遠いと壁 1 枚ぶんずれた解に落ちる。呼び出し側（`slam.py`）は等速度モデルで
    初期値を与えている。

    `max_correspondence_dist` は反復とともに段階的に狭める。最初から狭いと
    初期値の誤差ぶんの対応が全部切れて動けず、広いままだと遠くの無関係な面に
    引っ張られるため。
    """
    pose = init_pose.copy()
    fitness = 0.0
    rmse = float("inf")
    used_iterations = 0
    prev_rmse = float("inf")

    for i in range(max_iterations):
        used_iterations = i + 1
        # 対応距離のしきい値を max -> max/4 へ線形に絞る
        shrink = 1.0 - 0.75 * (i / max(max_iterations - 1, 1))
        threshold = max_correspondence_dist * shrink

        current = source @ pose[:3, :3].T + pose[:3, 3]
        dist, idx = target_tree.query(current, k=1, distance_upper_bound=threshold)
        matched = np.isfinite(dist)
        n_matched = int(matched.sum())
        if n_matched < 30:
            # 対応がほぼ無い＝初期値が外れているか地図が薄い。無理に解かず打ち切る。
            break

        fitness = n_matched / len(source)
        rmse = float(np.sqrt(np.mean(dist[matched] ** 2)))

        delta = _kabsch(current[matched], target_points[idx[matched]])
        pose = delta @ pose

        if abs(prev_rmse - rmse) < tolerance:
            break
        prev_rmse = rmse

    return IcpResult(pose=pose, fitness=fitness, inlier_rmse=rmse, n_iterations=used_iterations)
