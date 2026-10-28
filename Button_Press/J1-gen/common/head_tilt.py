"""頭カメラの向き（pitch / roll）を、深度から床や机の面を見つけて求める。実機日の最初に、首が傾いていないかを確かめる。

実機の頭カメラの向きは、首の傾きで変わる（2026-10-14 は約 45°、10-21 は 71°）。URDF（robot.yaml）の値のまま
位置を求めると、向きが違う分だけ、ボトルやボタンの位置が大きくずれる（24° で 20 cm 以上）。

やること: 深度の点群から平面を RANSAC で数枚見つける → 各平面について、「その平面が pelvis 座標で水平（法線が真上）に
なるカメラの pitch / roll」を求める → 床と机のように複数の平面で同じ値になれば、その値を信用する。
真上の向きは、IMU（骨盤）があればそれから、無ければ pelvis の z 軸とする。腰の角度は FK で見る。
前提: 立った状態で、床か机の面が画像に広く写っていること。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .camera_geometry import R_LINK_OPTICAL, HeadCameraTransform, rpy_to_matrix
from .rgbd_protocol import RgbdFrame


@dataclass
class PlaneTilt:
    inlier_fraction: float  # 有効な点のうち、この平面に乗った割合
    pitch_deg: float        # この平面を水平にするカメラの pitch（URDF の rpy の pitch と同じ決まり）
    roll_deg: float
    residual_deg: float     # 合わせたあとに残る法線の傾き。大きければ水平な面ではない（壁など）
    height_m: float         # カメラから平面までの距離 [m]


def imu_up_in_pelvis(imu_quat: np.ndarray | None) -> np.ndarray:
    """pelvis 座標での真上の向き。imu_quat は (w, x, y, z)（Unitree の IMU）。無ければ pelvis の z 軸。"""
    if imu_quat is None:
        return np.array([0.0, 0.0, 1.0])
    w, x, y, z = (float(v) for v in imu_quat)
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return r.T @ np.array([0.0, 0.0, 1.0])


def pelvis_tilt_deg(imu_quat: np.ndarray | None) -> float:
    """骨盤の傾き [°]（真上と pelvis の z 軸の角度）。"""
    return float(np.degrees(np.arccos(np.clip(imu_up_in_pelvis(imu_quat)[2], -1.0, 1.0))))


def points_from_depth(frame: RgbdFrame, step: int = 4, z_min: float = 0.2, z_max: float = 3.0) -> np.ndarray:
    """深度の点群（光学座標 [m]、(N, 3)）。step 画素おき。"""
    d = frame.depth_m()[::step, ::step]
    h, w = frame.depth.shape
    v, u = np.mgrid[0:h:step, 0:w:step]
    ok = (d > z_min) & (d < z_max)
    z = d[ok].astype(float)
    i = frame.intrinsics
    return np.stack([(u[ok] - i.cx) * z / i.fx, (v[ok] - i.cy) * z / i.fy, z], axis=1)


def find_planes(points: np.ndarray, max_planes: int = 3, threshold_m: float = 0.015, iterations: int = 300,
                min_fraction: float = 0.05, seed: int = 0) -> list[tuple[np.ndarray, float, float]]:
    """RANSAC で平面を数枚見つける。(法線（カメラに向く向き）, カメラから平面までの距離 [m], 乗った割合) の一覧。"""
    rng = np.random.default_rng(seed)
    rest, total, planes = points, len(points), []
    for _ in range(max_planes):
        if len(rest) < max(30, min_fraction * total):
            break
        best: np.ndarray | None = None
        for _ in range(iterations):
            p = rest[rng.choice(len(rest), 3, replace=False)]
            n = np.cross(p[1] - p[0], p[2] - p[0])
            norm = np.linalg.norm(n)
            if norm < 1e-9:
                continue
            n /= norm
            inl = np.abs((rest - p[0]) @ n) < threshold_m
            if best is None or inl.sum() > best.sum():
                best = inl
        if best is None or best.sum() < min_fraction * total:
            break
        pts = rest[best]
        c = pts.mean(axis=0)
        n = np.linalg.svd(pts - c, full_matrices=False)[2][-1]
        if n @ c > 0:
            n = -n  # カメラ（原点）に向ける
        planes.append((n, float(abs(n @ c)), float(best.sum() / total)))
        rest = rest[~best]
    return planes


def solve_tilt(n_cam: np.ndarray, r_parent: np.ndarray, yaw_rad: float, up: np.ndarray,
               pitch_range: tuple[float, float] = (30.0, 85.0), roll_range: tuple[float, float] = (-10.0, 10.0),
               step_deg: float = 0.25) -> tuple[float, float, float]:
    """法線（光学座標）が、pelvis 座標で up と一致するカメラの (pitch, roll, 残る角度) [°]。格子探索。"""
    pitches = np.arange(pitch_range[0], pitch_range[1] + 1e-9, step_deg)
    rolls = np.arange(roll_range[0], roll_range[1] + 1e-9, step_deg)
    best = (1e9, 0.0, 0.0)
    for pit in pitches:
        for rol in rolls:
            r = r_parent @ rpy_to_matrix(np.radians(rol), np.radians(pit), yaw_rad) @ R_LINK_OPTICAL
            c = float(np.clip((r @ n_cam) @ up, -1.0, 1.0))
            ang = float(np.degrees(np.arccos(c)))
            if ang < best[0]:
                best = (ang, float(pit), float(rol))
    return best[1], best[2], best[0]


def estimate_head_tilt(frame: RgbdFrame, transform: HeadCameraTransform, robot_cfg: dict[str, Any],
                       q_waist: np.ndarray, imu_quat: np.ndarray | None = None) -> list[PlaneTilt]:
    """1 フレームの深度から、見つけた平面ごとの (pitch, roll) を返す。"""
    transform.pelvis_from_optical(np.asarray(q_waist, dtype=float))  # 腰の FK を更新する
    r_parent = transform.data.oMf[transform.parent_frame].rotation.copy()
    yaw = float(robot_cfg["head_camera"]["rpy"][2])
    up = imu_up_in_pelvis(imu_quat)
    out = []
    for n, dist, frac in find_planes(points_from_depth(frame)):
        pit, rol, res = solve_tilt(n, r_parent, yaw, up)
        out.append(PlaneTilt(frac, pit, rol, res, dist))
    return out


def judge(planes: list[PlaneTilt], urdf_pitch_deg: float, max_residual_deg: float = 2.0) -> dict[str, Any]:
    """水平な面（残りの角度が小さいもの）の pitch の中央値と、URDF との差、判定の文を返す。"""
    good = [p for p in planes if p.residual_deg <= max_residual_deg]
    if not good:
        return {"pitch_deg": None, "roll_deg": None, "n_planes": 0, "spread_deg": None,
                "verdict": "水平な面（床・机）が見つからない。床か机の面が広く写るように、立ち直して撮り直す"}
    pit = float(np.median([p.pitch_deg for p in good]))
    rol = float(np.median([p.roll_deg for p in good]))
    spread = float(max(p.pitch_deg for p in good) - min(p.pitch_deg for p in good))
    diff = pit - urdf_pitch_deg
    if spread > 5.0:
        verdict = f"平面ごとの pitch が {spread:.1f}° ばらついた。信用せず、撮り直す（床と机が両方写る向きで）"
    elif abs(diff) < 5.0:
        verdict = f"pitch {pit:.1f}° は URDF（{urdf_pitch_deg:.1f}°）とほぼ同じ。首は傾いていない。上書き（head_camera_mount）は要らない"
    else:
        verdict = (f"pitch {pit:.1f}° は URDF（{urdf_pitch_deg:.1f}°）と {diff:+.1f}° 違う。首が傾いている可能性が高い。"
                   f"傾きを直して測り直す。直せないなら camera.yaml・camera_wifi.yaml の head_camera_mount の"
                   f"コメントを外し、rpy_deg を [{rol:.1f}, {pit:.1f}, 0.0] にする（depth_anchor も確認）")
    if len(good) == 1:
        verdict += "（水平な面が 1 枚だけ。床と机の両方が写ると、ばらつきも見られて確かになる）"
    return {"pitch_deg": pit, "roll_deg": rol, "n_planes": len(good), "spread_deg": spread, "verdict": verdict}
