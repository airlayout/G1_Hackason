#!/usr/bin/env python3
"""同じタグを見た 2 地点の**相対姿勢**を、タグだけから求める。

## なぜ要るか

地図との照合（`global_localize.py`）は、部屋が繰り返し構造だと一意に決まらないことがある。
そこへ**独立な拘束**を足す。同じタグが両方の地点から見えているなら、
**地図も測位も使わずに**地点間の相対姿勢が決まる。

これで「地図から出した 2 つの答え」が整合するかを検査できる。
2026-09-16 には、地図が regQ と regR を 6.6 m 離れていると言ったが、
両地点ともタグを 1.5〜2.4 m で見ていたので**あり得ない**と分かった。

## 何を仮定しているか

- タグは動いていない
- カメラの取付は両地点で同じ（⚠️ G1 の頭は姿勢で振れる。日をまたいだら怪しい）
- 2 枚以上の共通タグが見えている（1 枚だと回転が決まらない）

## 使い方

    python3 relative_pose_from_tags.py --session ~/g1_cfg/runs/regQ \\
        --session ~/g1_cfg/runs/regR --extrinsics ~/g1_cfg/apriltag/extrinsics.json
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
from measure_extrinsics import base_to_optical
from register_tags import camera_transform


def tags_in_base(directory: Path, detector, tag_m: float, camera_matrix: np.ndarray,
                 distortion: np.ndarray, base_from_camera: np.ndarray,
                 pattern: str) -> dict:
    """この地点から見た各タグの中心を、base_link 座標で返す。"""

    collected: dict = {}
    for path in sorted(directory.glob(pattern)):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        for found in tag_detect.detect(detector, image):
            try:
                pose = tag_detect.solve_pose(found, tag_m, camera_matrix, distortion)
            except RuntimeError:
                continue
            if not pose.trustworthy:
                continue
            point = base_from_camera[:3, :3] @ pose.tvec + base_from_camera[:3, 3]
            collected.setdefault(found.tag_id, []).append(point)
    return {tag_id: np.median(np.array(points), axis=0)
            for tag_id, points in collected.items()}


def relative_2d(first: dict, second: dict) -> tuple:
    """共通タグから、first → second の相対姿勢 (dx, dy, dyaw) を最小二乗で解く。

    タグは静止しているので、地点 A の base 座標 p_A と地点 B の base 座標 p_B の間には
    `p_A = R(dyaw) p_B + t` が成り立つ。これを Kabsch で解く。
    """

    shared = sorted(set(first) & set(second))
    if len(shared) < 2:
        return None, shared
    a = np.array([first[t][:2] for t in shared])
    b = np.array([second[t][:2] for t in shared])
    a_center, b_center = a.mean(axis=0), b.mean(axis=0)
    # Kabsch。H = P_c^T Q_c を SVD して R = V U^T（P=b が元、Q=a が先）。
    # ⚠️ `u @ vt` は R の**転置**である（2026-09-16 に取り違えて残差 1016 mm を出した）。
    covariance = (b - b_center).T @ (a - a_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    # rotation は b -> a の回転
    yaw = float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))
    translation = a_center - rotation @ b_center
    residual = float(np.linalg.norm((b - b_center) @ rotation.T - (a - a_center), axis=1).max())
    return (translation[0], translation[1], yaw, residual), shared


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session", action="append", required=True)
    parser.add_argument("--extrinsics", required=True)
    parser.add_argument("--tag-mm", type=float, default=160.0)
    parser.add_argument("--family", default="36h11", choices=sorted(tag_detect.FAMILIES))
    parser.add_argument("--pattern", default="raw_*.png")
    parser.add_argument("--intrinsics", default=None)
    parser.add_argument("--hfov-deg", type=float, default=87.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    arguments = parser.parse_args()

    import json
    extrinsics = json.loads(Path(arguments.extrinsics).read_text())
    if arguments.intrinsics:
        camera_matrix, distortion, _ = tag_detect.load_intrinsics(arguments.intrinsics)
    else:
        camera_matrix = tag_detect.default_camera_matrix(
            arguments.width, arguments.height, arguments.hfov_deg)
        distortion = np.zeros((1, 5))
    detector = tag_detect.make_detector(arguments.family)
    base_from_camera = camera_transform(extrinsics)

    tables = {}
    for directory in arguments.session:
        path = Path(directory)
        table = tags_in_base(path, detector, arguments.tag_mm / 1000.0,
                             camera_matrix, distortion, base_from_camera,
                             arguments.pattern)
        tables[path.name] = table
        print(f"{path.name}: タグ {sorted(table)}")
        for tag_id in sorted(table):
            point = table[tag_id]
            print("   ID%d  base 座標 (%+.3f, %+.3f, %+.3f)  距離 %.3f m"
                  % (tag_id, point[0], point[1], point[2], np.linalg.norm(point[:2])))

    print("\n=== 地点間の相対姿勢（タグだけから。地図も測位も使っていない）===")
    for left, right in itertools.combinations(tables, 2):
        result, shared = relative_2d(tables[left], tables[right])
        if result is None:
            print(f"  {left} — {right}: 共通タグが {len(shared)} 枚。2 枚以上ないと解けない")
            continue
        dx, dy, dyaw, residual = result
        print(f"  {left} → {right}: 平行移動 ({dx:+.3f}, {dy:+.3f}) = **{np.hypot(dx, dy):.3f} m** / "
              f"回転 **{dyaw:+.2f} deg**  （共通 {len(shared)} 枚・残差 {residual*1000:.0f} mm）")
        span = max(np.linalg.norm(tables[left][t][:2]) for t in shared) \
            + max(np.linalg.norm(tables[right][t][:2]) for t in shared)
        print(f"     ⇒ この 2 地点は **{np.hypot(dx, dy):.2f} m** 離れている"
              f"（タグの見え方から、離れられる上限は {span:.2f} m）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
