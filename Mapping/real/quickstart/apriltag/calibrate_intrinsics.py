#!/usr/bin/env python3
"""ChArUco ボードの写真から、カメラの内部パラメータ（fx, fy, cx, cy）を測る。

## なぜ要るのか

タグまでの距離は `距離 = fx × タグの実寸 / 画面上の一辺` で出る。**fx が 3 % 狂えば
距離も 3 % 狂う**。3 m で 9 cm。画角の公称値からの仮置きでは、この 3 % が消せない。

## D435i の IR なら歪みはほぼゼロ

D435i のステレオ（IR）出力は**出てくる時点で rectified** なので、歪み係数は実質 0 に
なるはずである。**もし大きな歪みが出たら、それは校正が失敗している合図**なので、
このスクリプトはそこを警告する。

## 撮り方（ここが結果を決める）

1. `charuco_a4.pdf` を等倍で刷り、**硬い板に貼る**（たわむと誤差になる）
2. ロボットは立たせたまま動かさない。**板の方を手で動かす**
3. 20〜30 枚。**画面の四隅を必ず通す**。中央だけだと cx, cy が決まらない
4. 板を**傾ける**（正面だけだと fx と距離が分離できない）。左右上下に 30〜45 度
5. ぶれた写真は入れない（このスクリプトが弾く）

    # PC2 で撮る
    python3 see_tags.py --seconds 30 --save-dir /tmp/calib --quiet
    # Mac へ持ってきて解く
    scp -r unitree@10.42.0.76:/tmp/calib ./calib
    Navigation/.venv/bin/python calibrate_intrinsics.py ./calib -o ir_1280x720.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

SQUARES = (8, 5)
SQUARE_M = 0.028
MARKER_M = 0.021
DICTIONARY = cv2.aruco.DICT_4X4_50
# チェッカー内点がこれ未満の写真は使わない。
MIN_CORNERS = 12
# ぼけ判定。ラプラシアンの分散がこれ未満なら「ぶれ」とみなす。
MIN_SHARPNESS = 40.0
# 再投影 RMS がこれを超えたら校正を信じない。
MAX_RMS_PIXELS = 1.0


def coverage_report(all_corners: list, width: int, height: int) -> str:
    """画面を 3x3 に割って、どこに点が当たったかを出す。四隅が空だと cx, cy が甘い。"""

    grid = np.zeros((3, 3), dtype=int)
    for corners in all_corners:
        for point in corners.reshape(-1, 2):
            column = min(2, int(point[0] / width * 3))
            row = min(2, int(point[1] / height * 3))
            grid[row, column] += 1
    lines = ["画面 3x3 への当たり（0 があると そこは外挿になる）"]
    for row in grid:
        lines.append("  " + "".join("%7d" % value for value in row))
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image_dir", help="ChArUco を撮った画像の置き場")
    parser.add_argument("-o", "--out", default="intrinsics.json")
    parser.add_argument("--pattern", default="*.png")
    parser.add_argument("--min-images", type=int, default=12)
    arguments = parser.parse_args()

    paths = sorted(Path(arguments.image_dir).glob(arguments.pattern))
    if not paths:
        parser.error(f"画像が無い: {arguments.image_dir}/{arguments.pattern}")

    board = cv2.aruco.CharucoBoard(SQUARES, SQUARE_M, MARKER_M,
                                   cv2.aruco.getPredefinedDictionary(DICTIONARY))
    detector = cv2.aruco.CharucoDetector(board)

    all_corners: list = []
    all_ids: list = []
    shape = None
    skipped = {"読めない": 0, "ぶれ": 0, "点が少ない": 0}
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            skipped["読めない"] += 1
            continue
        shape = image.shape[:2]
        sharpness = float(cv2.Laplacian(image, cv2.CV_64F).var())
        if sharpness < MIN_SHARPNESS:
            skipped["ぶれ"] += 1
            continue
        corners, ids, _, _ = detector.detectBoard(image)
        if corners is None or len(corners) < MIN_CORNERS:
            skipped["点が少ない"] += 1
            continue
        all_corners.append(corners)
        all_ids.append(ids)

    print(f"{len(paths)} 枚のうち {len(all_corners)} 枚を使う "
          f"(除外: " + " / ".join(f"{k} {v}" for k, v in skipped.items()) + ")")
    if len(all_corners) < arguments.min_images:
        print(f"⛔ {arguments.min_images} 枚に足りない。撮り直す（板を傾け、四隅を通す）")
        return 1

    height, width = shape
    print(coverage_report(all_corners, width, height))

    object_points = []
    image_points = []
    for corners, ids in zip(all_corners, all_ids):
        obj, img = board.matchImagePoints(corners, ids)
        if obj is None or len(obj) < MIN_CORNERS:
            continue
        object_points.append(obj)
        image_points.append(img)

    rms, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points, image_points, (width, height), None, None)
    focal_x, focal_y = camera_matrix[0, 0], camera_matrix[1, 1]
    center_x, center_y = camera_matrix[0, 2], camera_matrix[1, 2]
    hfov = 2.0 * np.degrees(np.arctan((width / 2.0) / focal_x))
    coefficients = np.asarray(distortion).ravel()

    print(f"\n再投影 RMS = {rms:.3f} px")
    print(f"fx = {focal_x:.2f}   fy = {focal_y:.2f}")
    print(f"cx = {center_x:.2f}   cy = {center_y:.2f}  （画面中心は "
          f"{width/2-0.5:.1f}, {height/2-0.5:.1f}）")
    print(f"水平画角 = {hfov:.2f} deg")
    print("歪み k1,k2,p1,p2,k3 = " + ", ".join(f"{value:+.4f}" for value in coefficients[:5]))

    verdict = 0
    if rms > MAX_RMS_PIXELS:
        print(f"\n⛔ RMS が {MAX_RMS_PIXELS} px を超えた。この値は使わない。"
              "板がたわんでいるか、ぶれた写真が混じっている")
        verdict = 1
    if abs(focal_x - focal_y) / focal_x > 0.02:
        print("⚠️ fx と fy が 2 % 以上違う。ふつう D435i では起きない。撮り直しを勧める")
    if abs(coefficients[0]) > 0.05:
        print(f"⚠️ k1 = {coefficients[0]:+.4f} が大きい。D435i の IR は rectified で "
              "歪みはほぼゼロのはず。**校正が失敗している疑い**")
    if abs(center_x - (width / 2 - 0.5)) > width * 0.05:
        print("⚠️ cx が画面中心から 5 % 以上ずれた。四隅を通せていない可能性")

    payload = {
        "camera_matrix": camera_matrix.tolist(),
        "distortion": coefficients.tolist(),
        "width": int(width),
        "height": int(height),
        "rms": float(rms),
        "images_used": len(object_points),
        "hfov_deg": float(hfov),
        "board": {"squares": list(SQUARES), "square_m": SQUARE_M, "marker_m": MARKER_M},
        "source_dir": str(Path(arguments.image_dir).resolve()),
    }
    Path(arguments.out).write_text(json.dumps(payload, indent=2))
    print(f"\n書いた: {arguments.out}")
    return verdict


if __name__ == "__main__":
    raise SystemExit(main())
