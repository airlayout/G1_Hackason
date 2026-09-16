#!/usr/bin/env python3
"""AprilTag の検出と姿勢推定。Mac でも PC2 でも同じものを使う。

## なぜ OpenCV の aruco なのか

PC2 は**インターネットに出られない**（2026-09-15 に確認）。一方で system の
python3.8 に OpenCV 5.0.0 が入っていて、`cv2.aruco` が AprilTag の辞書
（36h11 / 25h9 / 16h5）をそのまま持っている。**追加インストールが要らない**。
pupil-apriltags や apriltag_ros を持ち込むより、この道の方が短い。

## 平面の姿勢は 2 つある（ここがいちばん効く）

正方形 1 枚から解く姿勢は**必ず 2 解**ある（平面ターゲットの曖昧性）。正面に近い
ほど 2 解が似てきて、**再投影誤差では選べなくなる**。選び間違えると法線が鏡像に
なり、タグから逆算したロボットの向きが数十度ずれる。**黙って間違う**のが厄介で、
この計画がこれまで測位で踏んできたのと同じ型である。

だからここでは `solvePnPGeneric` で**両方の解を取り**、
`ambiguity = 誤差の小さい方 / 大きい方` を必ず返す。1.0 に近いほど危ない。
合否に使うときは `ambiguity < 0.5` を目安にし、それ以外は「向きは信じない」。
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

FAMILIES = {
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
}
# 一辺がこれを切ると読めない。2026-09-15 に**実機の IR 画像へ合成して測った**値で、
# 24 px は読めて 20 px は読めなかった（range_study.py）。余裕を見て 30 px を目安にする。
MIN_SIDE_PIXELS = 24.0
SAFE_SIDE_PIXELS = 30.0
# 2 解の誤差比がこれを超えたら向きを信じない。
AMBIGUITY_LIMIT = 0.5


class Detection:
    """タグ 1 つぶんの検出結果。"""

    def __init__(self, tag_id: int, corners: np.ndarray) -> None:
        self.tag_id = int(tag_id)
        self.corners = corners.astype(np.float64).reshape(4, 2)
        self.pose = None  # solve_pose が入れる

    @property
    def side_pixels(self) -> float:
        """4 辺の長さの平均 [px]。距離の目安になる。"""

        return float(np.mean([np.linalg.norm(self.corners[i] - self.corners[(i + 1) % 4])
                              for i in range(4)]))

    @property
    def center(self) -> np.ndarray:
        return self.corners.mean(axis=0)

    def where_in_frame(self, width: int, height: int) -> str:
        """画面のどこに写っているかを日本語で返す。貼る位置を決めるのに使う。"""

        x, y = self.center
        horizontal = "左" if x < width / 3 else ("右" if x > width * 2 / 3 else "中央")
        vertical = "上" if y < height / 3 else ("下" if y > height * 2 / 3 else "中")
        return f"{horizontal}{vertical}"


class Pose:
    """カメラ座標系から見たタグの姿勢。"""

    def __init__(self, rvec: np.ndarray, tvec: np.ndarray,
                 error: float, ambiguity: float) -> None:
        self.rvec = rvec.reshape(3)
        self.tvec = tvec.reshape(3)
        self.reprojection_error = float(error)
        self.ambiguity = float(ambiguity)

    @property
    def distance_m(self) -> float:
        return float(np.linalg.norm(self.tvec))

    @property
    def incidence_deg(self) -> float:
        """タグの法線と、カメラからタグへの視線との角度。0 が真正面。"""

        rotation, _ = cv2.Rodrigues(self.rvec)
        normal = rotation[:, 2]
        direction = self.tvec / max(np.linalg.norm(self.tvec), 1e-9)
        cosine = float(np.clip(abs(normal @ direction), -1.0, 1.0))
        return float(np.degrees(np.arccos(cosine)))

    @property
    def trustworthy(self) -> bool:
        return self.ambiguity < AMBIGUITY_LIMIT

    def matrix(self) -> np.ndarray:
        """4x4 の同次変換 T_camera_tag。"""

        rotation, _ = cv2.Rodrigues(self.rvec)
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = self.tvec
        return transform


def make_detector(family: str = "36h11", refine: bool = True) -> cv2.aruco.ArucoDetector:
    """AprilTag 用の detector を作る。

    `refine=True`（既定）は角を精密化する。2026-09-15 に測った代と効き:

    | | 1280x720 の時間（Orin NX）| 取付測定のばらつき |
    |---|---|---|
    | `refine=True`（APRILTAG）| **86.8 ms** | 高さ σ5.2 mm / 俯角 σ0.11 deg |
    | `refine=False`（NONE）| **16.7 ms** | 高さ σ17.7 mm / 俯角 σ0.48 deg |

    校正・取付・起動時の測位は精度が要るので True。
    歩きながら常時回すときは False も選べる（⚠️ PC2 の CPU は測位と取り合いになる）。
    """

    if family not in FAMILIES:
        raise ValueError(f"未対応の family: {family}（{sorted(FAMILIES)} のどれか）")
    parameters = cv2.aruco.DetectorParameters()
    # AprilTag 由来の角精密化。SUBPIX より平面ターゲットに合う（σ が 3 倍ちがう）。
    parameters.cornerRefinementMethod = (cv2.aruco.CORNER_REFINE_APRILTAG if refine
                                         else cv2.aruco.CORNER_REFINE_NONE)
    # 遠くの小さいタグを拾うため、初期の閾値窓を細かい方へ広げる。
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 43
    parameters.adaptiveThreshWinSizeStep = 8
    parameters.minMarkerPerimeterRate = 0.01
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def to_gray(image: np.ndarray) -> np.ndarray:
    return image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def detect(detector: cv2.aruco.ArucoDetector, image: np.ndarray) -> list:
    """画像 1 枚から検出結果の一覧を返す。"""

    corners, ids, _ = detector.detectMarkers(to_gray(image))
    if ids is None:
        return []
    return [Detection(tag_id, corner) for corner, tag_id in zip(corners, ids.ravel())]


def object_points(tag_m: float) -> np.ndarray:
    """タグ座標系での 4 隅。OpenCV の検出順（左上→右上→右下→左下）に合わせる。

    タグ座標系は、原点がタグの中心、+X が右、+Y が上、+Z が紙から手前へ出る向き。
    """

    half = tag_m / 2.0
    return np.array([[-half, half, 0.0], [half, half, 0.0],
                     [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float64)


def solve_pose(detection: Detection, tag_m: float, camera_matrix: np.ndarray,
               distortion: np.ndarray) -> Pose:
    """平面 4 点から姿勢を解く。**2 解を取り、曖昧さも返す**（冒頭の注記）。"""

    retval, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        object_points(tag_m), detection.corners, camera_matrix, distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not retval or len(rvecs) == 0:
        raise RuntimeError("solvePnP が解を返さなかった")
    values = [float(e) for e in np.asarray(errors).ravel()] if errors is not None else []
    if len(values) >= 2:
        best, second = sorted(values)[:2]
        ambiguity = best / second if second > 1e-9 else 1.0
        index = int(np.argmin(values))
    else:
        best = values[0] if values else 0.0
        ambiguity = 0.0
        index = 0
    pose = Pose(rvecs[index], tvecs[index], best, ambiguity)
    detection.pose = pose
    return pose


def default_camera_matrix(width: int, height: int, hfov_deg: float = 87.0) -> np.ndarray:
    """校正前に使う仮の内部パラメータ。**距離が画角の誤差ぶんだけ系統的にずれる**。

    D435i の IR は出力の時点で rectified なので歪みは実質ゼロ。だから校正で
    直すのは主に焦点距離である。`calibrate_intrinsics.py` を通したら置き換える。
    """

    focal = (width / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    return np.array([[focal, 0.0, width / 2.0 - 0.5],
                     [0.0, focal, height / 2.0 - 0.5],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def load_intrinsics(path) -> tuple:
    """`calibrate_intrinsics.py` が書いた JSON を読む。"""

    data = json.loads(Path(path).read_text())
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    distortion = np.array(data["distortion"], dtype=np.float64).reshape(1, -1)
    return camera_matrix, distortion, data


def annotate(image: np.ndarray, detections: list) -> np.ndarray:
    """検出を描き込んだ BGR 画像を返す。SSH 越しでも後から目で見られるように。"""

    canvas = image if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    canvas = canvas.copy()
    for found in detections:
        points = found.corners.astype(np.int32)
        ok = found.pose is None or found.pose.trustworthy
        color = (0, 220, 0) if ok else (0, 140, 255)
        cv2.polylines(canvas, [points], True, color, 2)
        cv2.circle(canvas, tuple(points[0]), 5, (255, 0, 0), -1)  # 左上を青で示す
        label = f"ID{found.tag_id} {found.side_pixels:.0f}px"
        if found.pose is not None:
            label += f" {found.pose.distance_m:.2f}m {found.pose.incidence_deg:.0f}deg"
            if not found.pose.trustworthy:
                label += " AMBIG"
        cv2.putText(canvas, label, tuple(points[0] + np.array([4, -8])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return canvas
