#!/usr/bin/env python3
"""J1-genのRGB-D配信 + YOLOでボタンの3D目標を計測する（腕は動かさない）。"""

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
from button_feedback import LampFrame
from j1gen_camera import J1CameraRuntime, J1RgbdCamera, add_camera_arguments


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
                 confidence: float = 0.25, *, camera_config="camera.yaml", host=None,
                 j1gen_root=None, runtime=None):
        runtime = runtime or J1CameraRuntime(j1gen_root)
        YoloDetector = runtime.module("perception_bridge").perception("detector.yolo_detector").YoloDetector
        self.transform = transform
        self.target_pixel = target_pixel
        self.detector = YoloDetector(model_name=weights, classes=["button"],
                                    confidence_threshold=confidence)
        self.tracker = TargetTracker()
        self.camera = J1RgbdCamera(runtime, camera_config, host)
        self.last_preview = None
        self.last_bbox = None
        self.last_frame = None
        self.last_lamp_frame = None

    def close(self) -> None:
        self.camera.close()

    def _candidates(self, frame):
        transform = self.transform() if callable(self.transform) else self.transform
        intr = frame.intrinsics
        intrinsics = CameraIntrinsics(intr.fx, intr.fy, intr.cx, intr.cy)
        depth_m = frame.depth_m()
        candidates = []
        for detection in self.detector.detect(frame.color_bgr):
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
            if self.tracker.previous is not None and np.linalg.norm(
                    np.array(target.face_xyz)-self.tracker.previous) > self.tracker.association_m:
                continue
            candidates.append((target, tuple(detection.bbox)))
        return candidates

    def observe(self, timeout_s: float = 2.0):
        self.tracker.begin_measurement()
        capture_after = time.time()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame, captured_at = self.camera.read_after(capture_after, deadline)
            candidates = self._candidates(frame)
            # 受信時には新しくても、推論に時間がかかった画像では動かさない。
            if time.monotonic() - captured_at > self.camera.max_age_s:
                self.tracker.samples.clear()
                continue
            if not candidates:
                self.tracker.samples.clear()
                continue
            observation = self.tracker.update([target for target, _ in candidates], captured_at)
            self.last_preview = frame.color_bgr.copy()
            self.last_frame = frame
            self.last_bbox = candidates[0][1]
            self.last_lamp_frame = LampFrame(self.last_preview.copy(), self.last_bbox,
                                             captured_at, frame.frame_id)
            if observation is not None:
                return observation
        self.tracker.samples.clear()
        raise RuntimeError("安定した新しいRGB-D計測を取得できません")

    def lamp_frames(self, after, timeout_s=2.0):
        """再検出は確認のためだけに使い、押下目標・追跡位置を更新しない。"""
        if self.tracker.previous is None:
            raise RuntimeError("点灯確認の対象ボタンが確定していません")
        capture_after = time.time()
        deadline = time.monotonic()+timeout_s
        while time.monotonic() < deadline:
            frame, captured_at = self.camera.read_after(capture_after, deadline)
            candidates = self._candidates(frame)
            if (time.monotonic() >= deadline or captured_at <= after
                    or time.monotonic()-captured_at > self.camera.max_age_s
                    or len(candidates) != 1):
                continue
            yield LampFrame(frame.color_bgr.copy(), candidates[0][1], captured_at, frame.frame_id)
        raise RuntimeError("対象ボタンの点灯確認用画像を取得できません")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", required=True, help="エレベーターボタン用 YOLO .pt")
    parser.add_argument("--class-name", default="button", help="重み内のボタンクラス名")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--base-transform", help="校正済み T_base_optical の JSON")
    parser.add_argument("--target-pixel", type=int, nargs=2, metavar=("U", "V"),
                        help="複数ボタン時に押す候補を示すカラー画像上の画素")
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=5.0, help="1枚の新しいRGB-Dを待つ時間 [秒]")
    parser.add_argument("--output", help="計測結果を保存する JSON")
    parser.add_argument("--preview", help="検出枠と画素座標を描いた画像の保存先 PNG")
    add_camera_arguments(parser)
    args = parser.parse_args(argv)
    if (args.frames < 3 or not np.isfinite(args.timeout) or args.timeout <= 0
            or not 0 < args.confidence < 1):
        parser.error("--framesは3以上、timeoutは正、confidenceは0～1が必要です")

    runtime = J1CameraRuntime(args.j1gen_root)
    YoloDetector = runtime.module("perception_bridge").perception("detector.yolo_detector").YoloDetector

    transform, frame_name = load_transform(args.base_transform)
    if args.preview:
        Path(args.preview).parent.mkdir(parents=True, exist_ok=True)
    detector = YoloDetector(model_name=args.weights, classes=[args.class_name],
                            confidence_threshold=args.confidence)
    camera = J1RgbdCamera(runtime, args.camera_config, args.host)
    targets = []
    try:
        capture_after = time.time()
        for _ in range(args.frames):
            frame, captured_at = camera.read_after(capture_after, time.monotonic() + args.timeout)
            intr = frame.intrinsics
            intrinsics = CameraIntrinsics(intr.fx, intr.fy, intr.cx, intr.cy)
            image, depth_m = frame.color_bgr, frame.depth_m()
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
            if time.monotonic() - captured_at > camera.max_age_s:
                print("[skip] 推論中にRGB-D計測が古くなりました")
                continue
            if len(candidates) != 1:
                print(f"[skip] 有効なボタン候補数={len(candidates)}。対象を一意に選べません")
                continue
            target = candidates[0]
            targets.append(target)
            print(f"[target] {tuple(round(v, 4) for v in target.face_xyz)} "
                  f"depth={target.depth_m:.3f} m")
    finally:
        camera.close()

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
        "source": "j1gen-rgbd-zmq+yolo",
        "j1gen_root": str(runtime.root),
        "camera_config": args.camera_config,
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
