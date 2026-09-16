#!/usr/bin/env python3
"""「どこに貼れば見えるか」を、カメラの取付から計算して表にする。

## なぜ要るのか

G1 の頭カメラは**下を向いている**。だから壁の高い位置に貼ったタグは、
どれだけ大きく刷っても**視野に入らない**。貼ってから気づくと剥がし直しになる。

ここは「カメラの高さと俯角」を入れると、

- 壁で見える高さの上限（距離ごと）
- 床に平置き / 立てて貼る のそれぞれの検出率と位置誤差

を出す。**貼る前に**回して、高さと距離を決める。

⚠️ 合成なのでこれは上限。歩行のぶれ・紙のたわみ・照明は入っていない。

## 使い方

    # まだ取付を測っていないとき（俯角を振って様子を見る）
    Navigation/.venv/bin/python placement_study.py --pitch 28 32 36

    # measure_extrinsics.py で実測したあと
    Navigation/.venv/bin/python placement_study.py --camera-height 1.21 --pitch 31.9 \
        --intrinsics ir_1280x720.json

## 2026-09-15 の結果（カメラ高 1.20 m・俯角 32 度・一辺 160 mm）

| 前方 | 床に平置き | 立てる h=0.3〜0.7 m | 立てる h=0.9 m |
|---|---|---|---|
| 1.5 m | 100 % / 7 mm | 100 % / 6 mm | 100 % / 6 mm |
| 2.0 m | **100 % / 7 mm** | 100 % / 10 mm | 100 % / 7 mm |
| 2.5 m | 92 % / 6 mm | 100 % / 13〜20 mm | 100 % / 19 mm |
| 3.0 m | 83 % / 11 mm | 89〜100 % / 16〜31 mm | **視野外** |
| 3.5 m | 58 % / 14 mm | 89〜100 % / 11 mm | 視野外 |
| 4.0 m | 8 % | 67〜89 % / 12〜21 mm | 視野外 |

**俯角が 32 度なら、画面の上端は水平より 3.9 度下**。壁で見える高さは
おおよそ `カメラ高 − 0.068 × 距離`。だから **0.9 m を超える高さは 3 m 以遠で外れる**。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tag_detect
from range_study import BLUR_SIGMA, NOISE_SIGMA, PAPER_BLACK, PAPER_WHITE

BACKGROUND_LEVEL = 100
# 床に平置きしたタグの姿勢（+Z が上）。
FLAT = np.eye(3)
# 立ててロボット側を向けたタグの姿勢（+Z が base の −x、+Y が base の +z）。
UPRIGHT = np.column_stack([[0, -1, 0], [0, 0, 1], [-1, 0, 0]]).astype(float)


def rotation_z(angle_rad: float) -> np.ndarray:
    cosine, sine = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def rotation_y(angle_rad: float) -> np.ndarray:
    cosine, sine = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]])


def base_to_optical(pitch_down_deg: float) -> np.ndarray:
    level = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    return rotation_y(np.radians(pitch_down_deg)) @ level


def visible_wall_height(camera_height: float, pitch_down_deg: float,
                        focal_y: float, image_height: int, distance: float) -> float:
    """距離 distance の壁で、画面の上端が届く高さ [m]。"""

    half_vertical = np.degrees(np.arctan((image_height / 2.0) / focal_y))
    top_edge_below_horizon = np.radians(pitch_down_deg - half_vertical)
    return camera_height - distance * np.tan(top_edge_below_horizon)


def try_once(detector, dictionary, camera_matrix, distortion, rotation_base_optical,
             translation_base_optical, forward: float, lateral: float, height: float,
             tag_rotation: np.ndarray, yaw_rad: float, tag_m: float,
             image_size: tuple, rng) -> float | None:
    """1 通り試す。視野外なら None、未検出なら 0.0、検出できたら誤差 [mm]。"""

    width, height_px = image_size
    scale = 30
    marker = cv2.aruco.generateImageMarker(dictionary, 3, 8 * scale, borderBits=1)
    canvas = np.full((8 * scale + 2 * scale,) * 2, 255, np.uint8)
    canvas[scale:-scale, scale:-scale] = marker
    size = canvas.shape[0]
    half = tag_m * 10 / 8 / 2
    local = np.array([[-half, half, 0.0], [half, half, 0.0],
                      [half, -half, 0.0], [-half, -half, 0.0]])
    placed = local @ (rotation_z(yaw_rad) @ tag_rotation).T
    placed = placed + np.array([forward, lateral, height])
    optical = (placed - translation_base_optical) @ rotation_base_optical
    if np.any(optical[:, 2] < 0.2):
        return None
    projected = (camera_matrix @ optical.T).T
    pixels = (projected[:, :2] / projected[:, 2:3]).astype(np.float32)
    if (pixels[:, 0].min() < 0 or pixels[:, 0].max() > width
            or pixels[:, 1].min() < 0 or pixels[:, 1].max() > height_px):
        return None
    source = np.array([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]],
                      np.float32)
    transform = cv2.getPerspectiveTransform(source, pixels)
    warped = cv2.warpPerspective(canvas, transform, (width, height_px), borderValue=0)
    mask = cv2.warpPerspective(np.full((size, size), 255, np.uint8), transform,
                               (width, height_px), borderValue=0)
    image = np.full((height_px, width), BACKGROUND_LEVEL, np.uint8).astype(np.float32)
    image[mask > 128] = (warped[mask > 128].astype(np.float32) / 255.0
                         * (PAPER_WHITE - PAPER_BLACK) + PAPER_BLACK)
    image = cv2.GaussianBlur(image, (0, 0), BLUR_SIGMA)
    image = np.clip(image + rng.normal(0.0, NOISE_SIGMA, image.shape), 0, 255).astype(np.uint8)

    found = [f for f in tag_detect.detect(detector, image) if f.tag_id == 3]
    if not found:
        return 0.0
    pose = tag_detect.solve_pose(found[0], tag_m, camera_matrix, distortion)
    estimated = rotation_base_optical @ pose.tvec + translation_base_optical
    error_m = float(np.linalg.norm(estimated[:2] - np.array([forward, lateral])))
    return max(error_m * 1000.0, 0.001)


def summarize(results: list) -> str:
    in_view = [r for r in results if r is not None]
    if not in_view:
        return "視野外"
    hits = [r for r in in_view if r > 0.0]
    rate = 100.0 * len(hits) / len(in_view)
    if not hits:
        return "%3.0f%%    —" % rate
    return "%3.0f%% %4.0fmm" % (rate, np.median(hits))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera-height", type=float, default=1.20,
                        help="床からカメラまで [m]。measure_extrinsics.py の値を入れる")
    parser.add_argument("--pitch", type=float, nargs="+", default=[32.0],
                        help="俯角 [deg]。複数並べると振って比べる")
    parser.add_argument("--tag-mm", type=float, default=160.0)
    parser.add_argument("--distances", type=float, nargs="+",
                        default=[1.5, 2.0, 2.5, 3.0, 3.5, 4.0])
    parser.add_argument("--heights", type=float, nargs="+", default=[0.3, 0.5, 0.7, 0.9],
                        help="立てて貼るときのタグ中心の高さ [m]")
    parser.add_argument("--yaw-samples", type=int, default=9)
    parser.add_argument("--intrinsics", default=None)
    parser.add_argument("--hfov-deg", type=float, default=87.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    arguments = parser.parse_args()

    if arguments.intrinsics:
        camera_matrix, distortion, meta = tag_detect.load_intrinsics(arguments.intrinsics)
        width, height_px = int(meta["width"]), int(meta["height"])
    else:
        width, height_px = arguments.width, arguments.height
        camera_matrix = tag_detect.default_camera_matrix(width, height_px,
                                                         arguments.hfov_deg)
        distortion = np.zeros((1, 5))
        print(f"⚠️ 未校正。画角 {arguments.hfov_deg:.0f}deg からの仮置き "
              f"fx={camera_matrix[0, 0]:.1f}")

    detector = tag_detect.make_detector("36h11")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag_m = arguments.tag_mm / 1000.0
    rng = np.random.default_rng(23)

    for pitch in arguments.pitch:
        rotation = base_to_optical(pitch)
        translation = np.array([0.0, 0.0, arguments.camera_height])
        half_vertical = np.degrees(np.arctan((height_px / 2.0) / camera_matrix[1, 1]))
        print(f"\n=== カメラ高 {arguments.camera_height:.2f} m / 俯角 {pitch:.1f} deg / "
              f"一辺 {arguments.tag_mm:.0f} mm ===")
        top_edge = pitch - half_vertical
        side = "下" if top_edge > 0 else "上"
        print(f"画面の上端は水平より {abs(top_edge):.1f} deg {side} "
              f"（垂直の半画角 {half_vertical:.1f} deg）")
        print("壁で見える高さの上限: " + " / ".join(
            f"{d:.1f}m→{visible_wall_height(arguments.camera_height, pitch, camera_matrix[1,1], height_px, d):.2f}m"
            for d in arguments.distances))

        header = "前方      床に平置き   " + "".join(
            "立て h=%.1fm " % h for h in arguments.heights)
        print("\n" + header)
        for distance in arguments.distances:
            # 床は全方位そのまま。立てはロボット側を向く ±60 度だけを数える。
            flat = [try_once(detector, dictionary, camera_matrix, distortion, rotation,
                             translation, distance, 0.0, 0.0, FLAT,
                             np.radians(y), tag_m, (width, height_px), rng)
                    for y in np.linspace(0.0, 180.0, arguments.yaw_samples)]
            cells = [summarize(flat)]
            for tag_height in arguments.heights:
                upright = [try_once(detector, dictionary, camera_matrix, distortion,
                                    rotation, translation, distance, 0.0, tag_height,
                                    UPRIGHT, np.radians(y), tag_m, (width, height_px), rng)
                           for y in np.linspace(-60.0, 60.0, arguments.yaw_samples)]
                cells.append(summarize(upright))
            print("%4.1f m    " % distance + "".join("%-13s" % c for c in cells))
    print("\n⚠️ 合成なので上限。ぶれ・たわみ・映り込みは入っていない")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
