"""ボトル（対象）の検出 → 3D 座標 → pelvis 座標。タスク4。

1. 検出（Perception の YoloDetector。事前学習済みの COCO の bottle クラス）で枠を得る
2. 枠の中の基準点（anchor。枠の幅・高さに対する割合）のまわり（roi_fraction の大きさ）の深度から、
   0（測れなかった画素）と範囲外を除いて中央値を取る。
   透明な PET ボトルは赤外線が透けて深度が取れないので、ラベル付きのボトルを使う（HANDOFF 6章）
   基準点: 頭カメラは斜め上から見下ろすので、ボトルの枠の中心には肩（上の方）が写る。胴の前面を狙うため
   ボトルは縦 0.7（上から 7 割）にする（MuJoCo で、胴の前面から x 方向 1.5 mm）。正面を向いたボタンなら中心（0.5）
3. 基準点の画素と深度から、内部パラメータでカメラ座標（光学座標）の点を求める（逆投影）
4. 腰の角度を使って pelvis 座標に直す（camera_geometry.py）

用語: 逆投影（deprojection）= 画素 (u, v) と深度 z から 3D の点を求めること。
     x = (u − cx) z / fx、y = (v − cy) z / fy（レンズのゆがみは無視する。D435 のカラーはほぼ 0）
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np

from .camera_geometry import HeadCameraTransform
from .rgbd_protocol import Intrinsics, RgbdFrame


class Detector(Protocol):
    def detect(self, frame: np.ndarray) -> list[Any]: ...


@dataclass
class Detection:
    """検出結果（Perception の Detection と同じ項目）。YOLO 以外の検出器（色、シミュレーションの
    セグメンテーション）はこちらを返す。Perception の detector を読み込むと torch と ultralytics が
    要るので、それらが無い環境（GitHub の CI など）でも使えるように、ここにも持っておく。"""

    class_name: str
    confidence: float
    bbox: tuple[float, float, float, float]  # (x1, y1, x2, y2) 画素


@dataclass
class Located:
    class_name: str
    confidence: float
    bbox: tuple[float, float, float, float]
    pixel: tuple[float, float]  # 深度を取った点（枠の中の基準点）
    depth_m: float | None  # 中央値の深度 [m]。取れなければ None
    valid_pixels: int  # 中央値に使った画素の数
    p_optical: np.ndarray | None  # カメラ座標 [m]
    p_pelvis: np.ndarray | None  # pelvis 座標 [m]
    reason: str = ""  # 位置が求まらなかった理由

    @property
    def ok(self) -> bool:
        return self.p_pelvis is not None

    def summary(self) -> str:
        if not self.ok:
            return f"{self.class_name}（{self.confidence:.2f}）: 位置が求まらない（{self.reason}）"
        assert self.p_pelvis is not None and self.depth_m is not None
        return (f"{self.class_name}（{self.confidence:.2f}）: 深度 {self.depth_m:.3f} m（{self.valid_pixels} 画素）、"
                f"pelvis 座標 {np.round(self.p_pelvis, 3)} m")


def deproject(u: float, v: float, z: float, intr: Intrinsics) -> np.ndarray:
    """画素 (u, v) と深度 z [m] → 光学座標の点 [m]。"""
    return np.array([(u - intr.cx) * z / intr.fx, (v - intr.cy) * z / intr.fy, z])


def anchor_pixel(bbox: tuple[float, float, float, float], anchor: tuple[float, float]) -> tuple[float, float]:
    """枠の中の基準点の画素 (u, v)。anchor は枠の幅・高さに対する割合（(0.5, 0.5) が中心）。"""
    x1, y1, x2, y2 = bbox
    return x1 + (x2 - x1) * anchor[0], y1 + (y2 - y1) * anchor[1]


def bbox_depth(
    depth_m: np.ndarray,
    bbox: tuple[float, float, float, float],
    roi_fraction: float,
    z_min: float,
    z_max: float,
    min_valid_pixels: int,
    anchor: tuple[float, float] = (0.5, 0.5),
) -> tuple[float | None, int, str]:
    """基準点のまわりの領域（幅・高さとも枠の roi_fraction 倍）の深度の中央値。取れなければ (None, 画素数, 理由)。"""
    h, w = depth_m.shape
    x1, y1, x2, y2 = bbox
    cx, cy = anchor_pixel(bbox, anchor)
    hw, hh = max(1.0, (x2 - x1) * roi_fraction / 2), max(1.0, (y2 - y1) * roi_fraction / 2)
    u0, u1 = int(max(0, np.floor(cx - hw))), int(min(w, np.ceil(cx + hw)))
    v0, v1 = int(max(0, np.floor(cy - hh))), int(min(h, np.ceil(cy + hh)))
    roi = depth_m[v0:v1, u0:u1]
    valid = roi[(roi > 0) & (roi >= z_min) & (roi <= z_max)]
    if valid.size < min_valid_pixels:
        zero = int(np.sum(roi == 0))
        return None, int(valid.size), (
            f"有効な深度が {valid.size} 画素（必要 {min_valid_pixels}）。領域 {roi.size} 画素のうち "
            f"0（測れなかった）が {zero} 画素。透明な物や近すぎ（< 約 0.2 m）の可能性"
        )
    return float(np.median(valid)), int(valid.size), ""


class Locator:
    def __init__(self, robot_cfg: dict[str, Any], loc_cfg: dict[str, Any], detector: Detector | None) -> None:
        self.cfg = loc_cfg
        self.detector = detector
        self.transform = HeadCameraTransform(robot_cfg, loc_cfg["calibration"]["offset_pelvis_m"])

    def locate_bbox(
        self, frame: RgbdFrame, bbox: tuple[float, float, float, float], q_waist: np.ndarray,
        class_name: str = "", confidence: float = 1.0,
    ) -> Located:
        """1 つの枠について、深度と pelvis 座標を求める。"""
        d = self.cfg["depth"]
        anchor = (float(d["anchor"][0]), float(d["anchor"][1]))
        z, n, reason = bbox_depth(frame.depth_m(), bbox, float(d["roi_fraction"]), float(d["min_m"]),
                                  float(d["max_m"]), int(d["min_valid_pixels"]), anchor)
        u, v = anchor_pixel(bbox, anchor)
        if z is None:
            return Located(class_name, confidence, bbox, (u, v), None, n, None, None, reason)
        p_opt = deproject(u, v, z, frame.intrinsics)
        p_pel = self.transform.to_pelvis(p_opt, q_waist)
        return Located(class_name, confidence, bbox, (u, v), z, n, p_opt, p_pel)

    def locate(self, frame: RgbdFrame, q_waist: np.ndarray) -> list[Located]:
        """検出して、見つかった対象すべての位置を求める（確からしさの高い順）。"""
        if self.detector is None:
            raise RuntimeError("検出器（detector）が無い")
        dets = self.detector.detect(frame.color_bgr)
        out = [self.locate_bbox(frame, tuple(d.bbox), q_waist, d.class_name, d.confidence) for d in dets]
        return sorted(out, key=lambda x: -x.confidence)

    def best(self, located: list[Located]) -> Located | None:
        """位置が求まったもののうち、確からしさが最も高いもの。"""
        ok = [x for x in located if x.ok]
        return ok[0] if ok else None


class ColorDetector:
    """色（HSV の範囲）で対象を探す検出器。YOLO が対象を見つけられないときの予備と、模擬ロボットのリハーサル用。

    HSV: 色相（H: 0〜179、OpenCV の単位）・彩度（S）・明るさ（V）。赤は色相が 0 付近と 179 付近に分かれるので、
    範囲を 2 つ書ける。一番大きい色の塊を 1 つ返す（面積が min_area_px より小さければ何も返さない）。
    """

    def __init__(self, hsv_ranges: list[list[list[int]]], min_area_px: int, class_name: str) -> None:
        self.ranges = [(np.array(lo, np.uint8), np.array(hi, np.uint8)) for lo, hi in hsv_ranges]
        self.min_area = int(min_area_px)
        self.class_name = class_name

    def detect(self, frame: np.ndarray) -> list[Any]:
        import cv2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        if n <= 1:
            return []
        i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        x, y, w, h, area = stats[i]
        if area < self.min_area:
            return []
        conf = float(area) / float(w * h)  # 枠の中で色が占める割合（確からしさの代わり）
        return [Detection(class_name=self.class_name, confidence=conf,
                                bbox=(float(x), float(y), float(x + w), float(y + h)))]


def make_detector(loc_cfg: dict[str, Any]) -> Detector:
    """設定の detector.type に応じて、Perception の YoloDetector か、色の検出器を作る。"""
    from .perception_bridge import perception

    det = loc_cfg["detector"]
    if det.get("type", "yolo") == "color":
        c = det["color"]
        return ColorDetector(c["hsv_ranges"], int(c["min_area_px"]), str(det["classes"][0]))
    return perception("detector").YoloDetector(
        model_name=det["model"], classes=list(det["classes"]),
        confidence_threshold=float(det["confidence_threshold"]), device=det.get("device", "auto"),
    )
