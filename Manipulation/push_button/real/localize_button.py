#!/usr/bin/env python3
"""RealSense RGB-D + 公開 YOLO 重みでボタンの3D目標を計測する（腕は動かさない）。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Callable

import numpy as np

BUTTON_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = BUTTON_ROOT.parents[1]
sys.path.insert(0, str(BUTTON_ROOT))
sys.path.insert(0, str(REPO_ROOT / "Perception"))

from vision import (CameraIntrinsics, estimate_button_target,
                    validate_reachable_target)
from alignment import TargetTracker


def load_transform(path: str | None) -> tuple[np.ndarray, str]:
    if path is None:
        return np.eye(4), "camera_optical"
    with Path(path).open(encoding="utf-8") as stream:
        payload = json.load(stream)
    if "T_base_optical" not in payload:
        raise ValueError("校正ファイルに T_base_optical がありません")
    return np.asarray(payload["T_base_optical"], dtype=float), "robot_base"


class RgbdLocalizer:
    """実行時の再計測用。初回だけ画素で選び、その後は3D位置で追跡する。"""

    def __init__(self, weights: str, transform: np.ndarray | Callable[[], np.ndarray],
                 target_pixel: tuple[int, int] | None = None,
                 confidence: float = 0.25):
        import pyrealsense2 as rs
        from common.detector.yolo_detector import YoloDetector

        self.rs = rs
        self.transform = transform
        self.target_pixel = target_pixel
        self.detector = YoloDetector(model_name=weights, classes=["button"],
                                    confidence_threshold=confidence)
        self.tracker = TargetTracker()
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        profile = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.last_preview = None
        self.last_bbox = None

    def close(self) -> None:
        self.pipe.stop()

    def observe(self, timeout_s: float = 2.0):
        self.tracker.begin_measurement()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            # 推論中に溜まった画像を捨て、次の新しいframesetを取得する。
            while self.pipe.poll_for_frames():
                pass
            aligned = self.align.process(self.pipe.wait_for_frames(1000))
            captured_at = time.monotonic()
            transform = self.transform() if callable(self.transform) else self.transform
            color = aligned.get_color_frame()
            depth = aligned.get_depth_frame()
            if not color or not depth:
                self.tracker.samples.clear()
                continue
            intr = depth.profile.as_video_stream_profile().get_intrinsics()
            intrinsics = CameraIntrinsics(intr.fx, intr.fy, intr.ppx, intr.ppy)
            image = np.asanyarray(color.get_data())
            depth_m = np.asanyarray(depth.get_data()).astype(np.float32) * self.scale
            candidates = []
            for detection in self.detector.detect(image):
                if self.tracker.initial is None and self.target_pixel is not None:
                    u, v = self.target_pixel
                    x1, y1, x2, y2 = detection.bbox
                    if not x1 <= u <= x2 or not y1 <= v <= y2:
                        continue
                try:
                    target = estimate_button_target(depth_m, detection.bbox,
                                                    intrinsics, transform,
                                                    expected_face_xyz=self.tracker.previous)
                    validate_reachable_target(target)
                except ValueError:
                    continue
                candidates.append(target)
            if self.tracker.previous is not None:
                candidates = [t for t in candidates if np.linalg.norm(
                    np.array(t.face_xyz) - self.tracker.previous) <= self.tracker.association_m]
            if not candidates:
                self.tracker.samples.clear()
                continue
            observation = self.tracker.update(candidates, captured_at)
            self.last_preview = image.copy()
            if observation is not None:
                return observation
        self.tracker.samples.clear()
        raise RuntimeError("安定した新しいRGB-D計測を取得できません")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", required=True, help="エレベーターボタン用 YOLO .pt")
    parser.add_argument("--class-name", default="button", help="重み内のボタンクラス名")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--base-transform", help="校正済み T_base_optical の JSON")
    parser.add_argument("--target-pixel", type=int, nargs=2, metavar=("U", "V"),
                        help="複数ボタン時に押す候補を示すカラー画像上の画素")
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--output", help="計測結果を保存する JSON")
    parser.add_argument("--preview", help="検出枠と画素座標を描いた画像の保存先 PNG")
    args = parser.parse_args()
    if (args.frames < 3 or args.width <= 0 or args.height <= 0 or args.fps <= 0
            or not 0 < args.confidence < 1):
        parser.error("--frames は3以上、画像サイズとfpsは正、confidenceは0～1が必要です")

    try:
        import pyrealsense2 as rs
    except ImportError as error:
        raise SystemExit("pyrealsense2 が必要です。G1 PC2 のRealSense環境で実行してください") from error
    from common.detector.yolo_detector import YoloDetector

    transform, frame_name = load_transform(args.base_transform)
    if args.preview:
        Path(args.preview).parent.mkdir(parents=True, exist_ok=True)
    detector = YoloDetector(model_name=args.weights, classes=[args.class_name],
                            confidence_threshold=args.confidence)
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    profile = pipe.start(cfg)
    targets = []
    try:
        align = rs.align(rs.stream.color)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        intrinsics = CameraIntrinsics(color_intr.fx, color_intr.fy,
                                      color_intr.ppx, color_intr.ppy)
        for _ in range(args.frames):
            aligned = align.process(pipe.wait_for_frames())
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            image = np.asanyarray(color_frame.get_data())
            depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
            detections = detector.detect(image)
            if args.preview:
                import cv2

                overlay = image.copy()
                for detection in detections:
                    x1, y1, x2, y2 = map(int, detection.bbox)
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(overlay, f"{(x1+x2)//2},{(y1+y2)//2}",
                                (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 1)
                if not cv2.imwrite(args.preview, overlay):
                    raise OSError(f"プレビュー画像を保存できません: {args.preview}")
            if args.target_pixel is not None:
                u, v = args.target_pixel
                detections = [d for d in detections
                              if d.bbox[0] <= u <= d.bbox[2]
                              and d.bbox[1] <= v <= d.bbox[3]]
            candidates = []
            for detection in detections:
                try:
                    candidate = estimate_button_target(depth_m, detection.bbox,
                                                       intrinsics, transform)
                    if frame_name == "robot_base":
                        validate_reachable_target(candidate)
                except ValueError:
                    continue
                candidates.append(candidate)
            if len(candidates) != 1:
                print(f"[skip] 有効なボタン候補数={len(candidates)}。対象を一意に選べません")
                continue
            target = candidates[0]
            targets.append(target)
            print(f"[target] {tuple(round(v, 4) for v in target.face_xyz)} "
                  f"depth={target.depth_m:.3f} m")
    finally:
        pipe.stop()

    if len(targets) < 3:
        raise SystemExit("有効なRGB-D検出が3フレーム未満です。押下目標は出力しません")
    positions = np.array([target.face_xyz for target in targets])
    directions = np.array([target.press_direction for target in targets])
    median_position = np.median(positions, axis=0)
    max_spread = float(np.max(np.linalg.norm(positions - median_position, axis=1)))
    if max_spread > 0.015:
        raise SystemExit(f"フレーム間の位置ばらつき {max_spread*1000:.1f} mm が大きすぎます")
    median_direction = np.median(directions, axis=0)
    median_direction /= np.linalg.norm(median_direction)
    output = {
        "schema": "g1-button-target-v1",
        "frame": frame_name,
        "face_xyz_m": median_position.tolist(),
        "press_direction": median_direction.tolist(),
        "valid_frames": len(targets),
        "max_position_spread_m": max_spread,
        "source": "realsense-aligned-depth+yolo",
    }
    print(json.dumps(output, ensure_ascii=False, indent=2))
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n",
                               encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
