#!/usr/bin/env python3
"""タグが「どこまで読めるか」を、実機の画像へ合成して測る。

## なぜ合成でやるのか

貼る前に**大きさを決めなければならない**。一度刷ってしまうと貼り直しは高い。
実機のカメラで撮った本物の背景・本物の露出・本物のノイズの上にタグを投影すれば、
印刷する前に「一辺 160 mm で何 m まで読めるか」を数字で出せる。

⚠️ **これは上限である。** 合成には歩行時のぶれも、紙のたわみも、照明の映り込みも
入っていない。実測は必ずこれより短くなる。**合否の根拠には使わない**。
大きさを決めるためだけに使う。

## 使い方

    # 背景に使う実機の画像を PC2 から持ってくる
    ssh unitree@10.42.0.76 'python3 ~/g1_cfg/apriltag/see_tags.py --frames 3 --save-dir /tmp/bg'
    scp unitree@10.42.0.76:/tmp/bg/raw_00003.png ./bg.png

    Navigation/.venv/bin/python range_study.py --background bg.png

## 2026-09-15 の結果（1280x720 / 画角 87deg / 背景は実機の IR）

| 一辺 | どの角度でも通る | まだらになる | 落ちる |
|---|---|---|---|
| 160 mm（A4）| **3.5 m まで**（0〜60 度）| 4.5 m | 5.5 m |
| 240 mm（A3）| **4.5 m まで**（0〜60 度）| 5.5〜6.5 m | 70 度は 4.5 m で落ちる |

70 度の斜めはどちらも 3.5 m が限界。**確実に読ませたい距離は、この表の 7 割で見ておく。**

**床に平らに置いたタグは前方 2 m まで**（俯角 25〜40 度で同じ）。入射角で潰れるのが
効いているので、**紙を大きくしても伸びない**。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tag_detect

# 紙の白と黒が IR 画像で取る輝度。2026-09-15 の実機画像（平均輝度 90）に合わせた。
PAPER_WHITE = 200
PAPER_BLACK = 35
# レンズと再標本化のぼけ。実機の 1 px 程度。
BLUR_SIGMA = 0.6
NOISE_SIGMA = 2.0


def render(background: np.ndarray, camera_matrix: np.ndarray, tag_m: float,
           distance_m: float, yaw_deg: float, tag_id: int,
           dictionary, rng) -> np.ndarray:
    """タグを実寸・実距離・実角度でカメラ画像へ投影して背景に貼る。"""

    height, width = background.shape[:2]
    modules = 8
    scale = 30
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, modules * scale,
                                           borderBits=1)
    canvas = np.full((modules * scale + 2 * scale,) * 2, 255, np.uint8)
    canvas[scale:-scale, scale:-scale] = marker
    size = canvas.shape[0]

    half = tag_m * (modules + 2) / modules / 2.0
    angle = np.radians(yaw_deg)
    rotation = np.array([[np.cos(angle), 0.0, np.sin(angle)],
                         [0.0, 1.0, 0.0],
                         [-np.sin(angle), 0.0, np.cos(angle)]])
    # 画像の v は下向き。キャンバスの上端を y = -half に対応させる。
    corners_3d = np.array([[-half, -half, 0.0], [half, -half, 0.0],
                           [half, half, 0.0], [-half, half, 0.0]]) @ rotation.T
    corners_3d += np.array([0.0, 0.0, distance_m])
    projected = (camera_matrix @ corners_3d.T).T
    destination = (projected[:, :2] / projected[:, 2:3]).astype(np.float32)
    source = np.array([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]],
                      np.float32)
    transform = cv2.getPerspectiveTransform(source, destination)
    warped = cv2.warpPerspective(canvas, transform, (width, height), borderValue=0)
    mask = cv2.warpPerspective(np.full((size, size), 255, np.uint8), transform,
                               (width, height), borderValue=0)

    out = background.astype(np.float32).copy()
    paper = warped.astype(np.float32) / 255.0 * (PAPER_WHITE - PAPER_BLACK) + PAPER_BLACK
    out[mask > 128] = paper[mask > 128]
    out = cv2.GaussianBlur(out, (0, 0), BLUR_SIGMA)
    out = out + rng.normal(0.0, NOISE_SIGMA, out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--background", required=True, help="実機で撮った IR 画像")
    parser.add_argument("--family", default="36h11", choices=sorted(tag_detect.FAMILIES))
    parser.add_argument("--tag-mm", type=float, nargs="+", default=[160.0, 240.0])
    parser.add_argument("--distances", type=float, nargs="+",
                        default=[1.5, 2.5, 3.5, 4.5, 5.5, 6.5])
    parser.add_argument("--angles", type=float, nargs="+", default=[0.0, 30.0, 45.0, 60.0, 70.0])
    parser.add_argument("--hfov-deg", type=float, default=87.0)
    parser.add_argument("--intrinsics", default=None)
    parser.add_argument("--tag-id", type=int, default=5)
    parser.add_argument("--sample-out", default=None, help="1 枚だけ合成画像を残す")
    arguments = parser.parse_args()

    background = cv2.imread(arguments.background, cv2.IMREAD_GRAYSCALE)
    if background is None:
        parser.error(f"背景を読めない: {arguments.background}")
    height, width = background.shape[:2]
    if arguments.intrinsics:
        camera_matrix, distortion, _ = tag_detect.load_intrinsics(arguments.intrinsics)
    else:
        camera_matrix = tag_detect.default_camera_matrix(width, height, arguments.hfov_deg)
        distortion = np.zeros((1, 5))
        print(f"⚠️ 未校正。画角 {arguments.hfov_deg:.0f}deg からの仮置き fx={camera_matrix[0,0]:.1f}")

    detector = tag_detect.make_detector(arguments.family)
    dictionary = cv2.aruco.getPredefinedDictionary(tag_detect.FAMILIES[arguments.family])
    rng = np.random.default_rng(1)
    print(f"背景 {width}x{height} 平均輝度 {int(background.mean())} / {arguments.family}")

    for tag_mm in arguments.tag_mm:
        print(f"\n=== 一辺 {tag_mm:.0f} mm — 推定した距離 [m]、✕ は未検出 ===")
        print("真の距離 " + "".join("%9s" % f"{a:.0f}deg" for a in arguments.angles))
        for distance in arguments.distances:
            cells = []
            for angle in arguments.angles:
                image = render(background, camera_matrix, tag_mm / 1000.0, distance,
                               angle, arguments.tag_id, dictionary, rng)
                found = [f for f in tag_detect.detect(detector, image)
                         if f.tag_id == arguments.tag_id]
                if not found:
                    cells.append("✕")
                    continue
                pose = tag_detect.solve_pose(found[0], tag_mm / 1000.0,
                                             camera_matrix, distortion)
                mark = "" if pose.trustworthy else "?"
                cells.append(f"{pose.distance_m:.2f}{mark}")
            print("%6.1f   " % distance + "".join("%9s" % c for c in cells))

    if arguments.sample_out:
        image = render(background, camera_matrix, arguments.tag_mm[0] / 1000.0,
                       2.5, 30.0, arguments.tag_id, dictionary, rng)
        cv2.imwrite(arguments.sample_out, image)
        print(f"\n合成例: {arguments.sample_out}")
    print("\n⚠️ これは上限。歩行のぶれ・紙のたわみ・照明の映り込みは入っていない")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
