#!/usr/bin/env python3
"""貼ったタグの並びを数字にする。巻尺と突き合わせるための道具。

## 何を出すか

- **タグ間の中心距離** … カメラの取付にも内部パラメータの誤差にも比較的強い量で、
  **巻尺で直接確かめられる唯一の値**。ここが合っていれば「刷った寸法・検出・
  姿勢推定」がまとめて正しいと言える
- **同時に見えた枚数** … 起動時の測位に何枚使えるか
- **床に載っているか** … 全部が同じ平面に乗っているかを残差で見る
- 各タグのカメラ座標での位置と、その ばらつき

## 使い方

    python3 see_tags.py --seconds 10 --save-dir /tmp/tags --save-every 5
    python3 tag_layout.py /tmp/tags --tag-mm 160

⚠️ 内部パラメータが未校正だと、**距離が一律に数 % 伸び縮みする**。
巻尺と数 % ずれていたら、まず `calibrate_intrinsics.py` を通すこと。
"""

from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tag_detect


def plane_residual(points: np.ndarray) -> float:
    """点群に平面を当てたときの残差 [m]。全部が床に載っていれば小さい。"""

    if len(points) < 3:
        return float("nan")
    centered = points - points.mean(axis=0)
    _, singular, _ = np.linalg.svd(centered, full_matrices=False)
    return float(singular[-1] / np.sqrt(len(points)))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image_dir")
    parser.add_argument("--pattern", default="raw_*.png")
    parser.add_argument("--tag-mm", type=float, default=160.0)
    parser.add_argument("--family", default="36h11", choices=sorted(tag_detect.FAMILIES))
    parser.add_argument("--intrinsics", default=None)
    parser.add_argument("--hfov-deg", type=float, default=87.0)
    parser.add_argument("--known-gap", nargs=3, metavar=("ID_A", "ID_B", "METERS"),
                        help="巻尺で測った 2 枚の間隔。ここから焦点距離を逆算する")
    arguments = parser.parse_args()

    paths = sorted(Path(arguments.image_dir).glob(arguments.pattern))
    if not paths:
        parser.error(f"画像が無い: {arguments.image_dir}/{arguments.pattern}")
    first = cv2.imread(str(paths[0]), cv2.IMREAD_GRAYSCALE)
    if first is None:
        parser.error(f"画像を読めない: {paths[0]}")
    height, width = first.shape[:2]

    if arguments.intrinsics:
        camera_matrix, distortion, _ = tag_detect.load_intrinsics(arguments.intrinsics)
    else:
        camera_matrix = tag_detect.default_camera_matrix(width, height, arguments.hfov_deg)
        distortion = np.zeros((1, 5))
        print(f"⚠️ 未校正。画角 {arguments.hfov_deg:.0f}deg からの仮置き "
              f"fx={camera_matrix[0, 0]:.1f} → **距離が一律に数 % ずれる**")

    detector = tag_detect.make_detector(arguments.family)
    tag_m = arguments.tag_mm / 1000.0
    per_tag: dict = {}
    per_frame: list = []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        frame: dict = {}
        for found in tag_detect.detect(detector, image):
            try:
                pose = tag_detect.solve_pose(found, tag_m, camera_matrix, distortion)
            except RuntimeError:
                continue
            frame[found.tag_id] = pose.tvec
            per_tag.setdefault(found.tag_id, []).append(pose.tvec)
        per_frame.append(frame)

    if not per_tag:
        print("⛔ 1 枚も検出できなかった")
        return 1

    counts: dict = {}
    for frame in per_frame:
        counts[len(frame)] = counts.get(len(frame), 0) + 1
    print(f"\n{len(per_frame)} 枚を見た。同時に見えた枚数:")
    for number in sorted(counts, reverse=True):
        print(f"  {number} 枚が同時 … {counts[number]} フレーム "
              f"({100.0 * counts[number] / len(per_frame):.0f} %)")

    print("\nカメラから見た各タグの中心（x 右 / y 下 / z 前）[m]")
    print("ID    x       y       z      距離    ばらつき")
    for tag_id in sorted(per_tag):
        points = np.array(per_tag[tag_id])
        center = points.mean(axis=0)
        scatter = float(np.linalg.norm(points - center, axis=1).std())
        print("%-4d %+.3f  %+.3f  %+.3f   %.3f   %.0f mm" % (
            tag_id, center[0], center[1], center[2],
            float(np.linalg.norm(center)), scatter * 1000))

    print("\n**タグ間の中心距離**（巻尺で確かめる値）")
    print("組み合わせ     推定         ばらつき   見えたフレーム")
    for left, right in itertools.combinations(sorted(per_tag), 2):
        both = [np.linalg.norm(f[left] - f[right]) for f in per_frame
                if left in f and right in f]
        if not both:
            print("ID%d — ID%d      同時に見えたフレームが無い" % (left, right))
            continue
        print("ID%d — ID%d      %.3f m      %.0f mm     %d" % (
            left, right, float(np.median(both)), float(np.std(both)) * 1000, len(both)))

    if arguments.known_gap:
        left, right = int(arguments.known_gap[0]), int(arguments.known_gap[1])
        truth = float(arguments.known_gap[2])
        both = [np.linalg.norm(f[left] - f[right]) for f in per_frame
                if left in f and right in f]
        if not both:
            print(f"\n⚠️ ID{left} と ID{right} が同時に見えたフレームが無い")
        else:
            estimated = float(np.median(both))
            # 距離は fx に比例してずれる: 推定 = 真値 × fx仮 / fx真
            focal = camera_matrix[0, 0] * truth / estimated
            hfov = 2.0 * np.degrees(np.arctan((width / 2.0) / focal))
            error = 100.0 * (estimated / truth - 1.0)
            print(f"\n=== 巻尺 {truth:.3f} m との突き合わせ ===")
            print(f"推定 {estimated:.3f} m → **{error:+.1f} %**")
            print(f"これを説明する焦点距離: fx = {focal:.0f}（画角 {hfov:.1f} deg）")
            if abs(error) < 2.0:
                print("✅ 2 % 以内。仮の内部パラメータのままでも使える")
            else:
                print("⚠️ 2 % を超えた。**ChArUco で校正するまで距離は信じない**")
                print("   （上の fx は 1 つの拘束だけから出した粗い値。校正の代わりにはならない）")

    centers = np.array([np.mean(per_tag[t], axis=0) for t in sorted(per_tag)])
    if len(centers) >= 3:
        # ⚠️ 3 点の平面残差は**必ず 0 になる**（3 点は常に 1 平面を決める）。
        # 「床に載っている」の証拠になるのは 4 枚以上を非一直線に置いたときだけ。
        if len(centers) >= 4:
            residual = plane_residual(centers)
            print(f"\n同じ平面に載っているか: 残差 {residual * 1000:.0f} mm "
                  f"({'床に載っている' if residual < 0.03 else '⚠️ どれかが傾いている'})")
        else:
            print(f"\n同じ平面に載っているか: {len(centers)} 枚では判定できない"
                  "（3 点の平面残差は必ず 0 になる）。4 枚以上を三角に置くと測れる")
        # 3 点がどれだけ一直線に近いか（測位の見通しに効く）
        spans = [np.linalg.norm(a - b) for a, b in itertools.combinations(centers, 2)]
        longest = max(spans)
        area = 0.5 * np.linalg.norm(np.cross(centers[1] - centers[0], centers[2] - centers[0]))
        print("広がり: 最長の間隔 %.2f m / 三角形の高さ %.2f m %s" % (
            longest, 2 * area / longest if longest > 0 else 0.0,
            "（ほぼ一直線）" if (2 * area / longest) < 0.15 * longest else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
