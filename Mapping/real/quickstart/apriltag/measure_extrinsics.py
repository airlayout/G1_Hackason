#!/usr/bin/env python3
"""床に置いた AprilTag 1 枚から、カメラの取付（高さ・俯角・ロール）を測る。

## なぜこれが要るのか

タグからロボットの地図上の位置を出すには `base_link -> camera` が要る。
G1 の頭カメラについては、この値がどこにも無い（URDF もリポジトリに無い）。

## なぜ「床」なのか

タグを**床に平らに置く**と、タグの面がそのまま床面になる。すると:

- タグの法線 = 床の法線 = 「上」 → カメラの**俯角とロール**がそのまま出る
- カメラからタグ平面までの距離 = カメラの**床からの高さ**
- **タグをどこに置いたかを測らなくてよい**。巻尺が要らない

⚠️ **ヨー（左右の向き）はこの方法では出ない。** 床だけでは前方が決まらないため。
ヨーは 0（カメラは機体の正面を向いている）と置く。G1 の頭は胴体に対して
受動的に動きうるので、**歩いたあとや起こし直したあとは測り直す**こと。

⚠️ `base_link` は**床面・水平**で定義してある（2026-09-09 の決定）。だから
ここで出る高さと角度は、そのまま `base_link -> camera` の並進 z と回転になる。

## 使い方

    # PC2 で、床のタグが見える状態で 10 秒録る
    python3 see_tags.py --seconds 10 --save-dir /tmp/floor --intrinsics ir.json
    # 解く（Mac でも PC2 でも）
    python3 measure_extrinsics.py /tmp/floor --intrinsics ir.json --tag-mm 160
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tag_detect

# 床タグは斜めから見えるので、2 解の曖昧さは小さいはず。大きい枚は捨てる。
MAX_AMBIGUITY = 0.6
# 中央値からこれ以上離れた枚は外れ値として捨てる。
OUTLIER_DEG = 3.0
OUTLIER_M = 0.05


def floor_pose_to_extrinsics(pose: tag_detect.Pose) -> tuple:
    """床タグの姿勢から (高さ m, 俯角 deg, ロール deg) を出す。

    カメラ座標系は x 右・y 下・z 前。床タグの +Z 軸は上を向いているので、
    `R_camera_tag` の 3 列目がカメラ座標での「上」になる。
    """

    rotation, _ = cv2.Rodrigues(pose.rvec)
    up = rotation[:, 2]
    if up[1] > 0:           # 「上」が画像の下を向いていたら裏の解。ひっくり返す
        up = -up
    up = up / np.linalg.norm(up)
    height = float(abs(up @ pose.tvec))
    pitch_down = float(np.degrees(np.arcsin(np.clip(-up[2], -1.0, 1.0))))
    # `up` は光学座標で (-sin(roll)cos(pitch), -cos(roll)cos(pitch), -sin(pitch))。
    # 素直に atan2(up[0], -up[1]) を取ると符号が逆になる（2026-09-15 に合成で確認）。
    roll = float(np.degrees(np.arctan2(-up[0], -up[1])))
    return height, pitch_down, roll


def base_to_optical(pitch_down_deg: float, roll_deg: float) -> np.ndarray:
    """base_link から光学座標系（x右・y下・z前）への回転行列を組み立てる。"""

    pitch = np.radians(pitch_down_deg)
    roll = np.radians(roll_deg)
    # 水平に前を向いたカメラ: 光学 x=base -y, y=base -z, z=base +x
    level = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    pitch_about_base_y = np.array([[np.cos(pitch), 0.0, np.sin(pitch)],
                                   [0.0, 1.0, 0.0],
                                   [-np.sin(pitch), 0.0, np.cos(pitch)]])
    roll_about_optical_z = np.array([[np.cos(roll), -np.sin(roll), 0.0],
                                     [np.sin(roll), np.cos(roll), 0.0],
                                     [0.0, 0.0, 1.0]])
    return pitch_about_base_y @ level @ roll_about_optical_z


def rpy_from_matrix(rotation: np.ndarray) -> tuple:
    """ROS の静的 TF が取る roll/pitch/yaw [rad]（R = Rz(yaw) Ry(pitch) Rx(roll)）。"""

    pitch = float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
    yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return roll, pitch, yaw


def robust_mean(values: np.ndarray, limit: float) -> tuple:
    """中央値から limit 以上離れた値を落としてから平均する。"""

    values = np.asarray(values, dtype=float)
    center = float(np.median(values))
    keep = np.abs(values - center) <= limit
    if not np.any(keep):
        keep = np.ones_like(values, dtype=bool)
    return float(values[keep].mean()), float(values[keep].std()), int(keep.sum())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image_dir")
    parser.add_argument("--pattern", default="raw_*.png")
    parser.add_argument("--intrinsics", default=None)
    parser.add_argument("--hfov-deg", type=float, default=87.0)
    parser.add_argument("--tag-mm", type=float, default=160.0)
    parser.add_argument("--tag-id", type=int, default=None,
                        help="床に置いたタグの ID。省くと見えたもの全部を床とみなす")
    parser.add_argument("--family", default="36h11", choices=sorted(tag_detect.FAMILIES))
    parser.add_argument("--out", default=None,
                        help="結果を JSON で保存する（register_tags.py が読む）")
    parser.add_argument("--forward", type=float, default=0.0,
                        help="base_link から見たカメラの前方オフセット [m]。"
                             "⚠️ 床タグでは測れないので既定 0。巻尺で入れてもよい")
    parser.add_argument("--lateral", type=float, default=0.0, help="同じく横方向 [m]")
    parser.add_argument("--yaw-deg", type=float, default=0.0,
                        help="同じくヨー [deg]。⚠️ 床タグでは測れない。"
                             "register_tags.py --solve-camera-yaw が 2 地点から出す")
    arguments = parser.parse_args()

    paths = sorted(Path(arguments.image_dir).glob(arguments.pattern))
    if not paths:
        parser.error(f"画像が無い: {arguments.image_dir}/{arguments.pattern}")

    first = cv2.imread(str(paths[0]), cv2.IMREAD_GRAYSCALE)
    if first is None:
        parser.error(f"画像を読めない: {paths[0]}")
    height_px, width_px = first.shape[:2]
    if arguments.intrinsics:
        camera_matrix, distortion, meta = tag_detect.load_intrinsics(arguments.intrinsics)
        print(f"内部パラメータ {arguments.intrinsics} fx={camera_matrix[0,0]:.1f}")
        if meta.get("width") != width_px:
            print(f"⚠️ 校正は {meta.get('width')} px 幅。画像は {width_px} px。高さがずれる")
    else:
        camera_matrix = tag_detect.default_camera_matrix(width_px, height_px,
                                                         arguments.hfov_deg)
        distortion = np.zeros((1, 5))
        print(f"⚠️ 未校正。画角 {arguments.hfov_deg:.0f}deg からの仮置き。"
              "**高さが画角の誤差ぶんだけ系統的にずれる**")

    detector = tag_detect.make_detector(arguments.family)
    tag_m = arguments.tag_mm / 1000.0
    heights, pitches, rolls, dropped = [], [], [], 0
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        for found in tag_detect.detect(detector, image):
            if arguments.tag_id is not None and found.tag_id != arguments.tag_id:
                continue
            try:
                pose = tag_detect.solve_pose(found, tag_m, camera_matrix, distortion)
            except RuntimeError:
                continue
            if pose.ambiguity > MAX_AMBIGUITY:
                dropped += 1
                continue
            height, pitch, roll = floor_pose_to_extrinsics(pose)
            heights.append(height)
            pitches.append(pitch)
            rolls.append(roll)

    print(f"{len(paths)} 枚 / 使えた観測 {len(heights)} 件 / 曖昧で捨てた {dropped} 件")
    if len(heights) < 5:
        print("⛔ 観測が足りない。タグが床に平らに置かれ、はっきり見えているか確かめる")
        return 1

    height_mean, height_std, height_n = robust_mean(heights, OUTLIER_M)
    pitch_mean, pitch_std, _ = robust_mean(pitches, OUTLIER_DEG)
    roll_mean, roll_std, _ = robust_mean(rolls, OUTLIER_DEG)
    print(f"\nカメラの床からの高さ  {height_mean:.3f} m  (ばらつき {height_std*1000:.0f} mm, "
          f"使った {height_n} 件)")
    print(f"俯角（下向き）        {pitch_mean:.2f} deg  (ばらつき {pitch_std:.2f})")
    print(f"ロール                {roll_mean:.2f} deg  (ばらつき {roll_std:.2f})")

    rotation = base_to_optical(pitch_mean, roll_mean)
    roll_rad, pitch_rad, yaw_rad = rpy_from_matrix(rotation)
    print("\n静的 TF（ヨーは 0 と置いた。子は光学座標系 x右 y下 z前）:")
    print("  ros2 run tf2_ros static_transform_publisher \\")
    print(f"    --x 0 --y 0 --z {height_mean:.4f} \\")
    print(f"    --roll {roll_rad:.6f} --pitch {pitch_rad:.6f} --yaw {yaw_rad:.6f} \\")
    print("    --frame-id base_link --child-frame-id ir_optical_frame")
    print("  ⚠️ ヨーは測れていない。頭が胴体に対して振れていたらここが効く。"
          "**RViz2 で点群と重ねて確かめること**")

    if arguments.out:
        import json
        payload = {
            "height_m": height_mean,
            "pitch_down_deg": pitch_mean,
            "roll_deg": roll_mean,
            "yaw_deg": arguments.yaw_deg,
            "forward_m": arguments.forward,
            "lateral_m": arguments.lateral,
            "height_std_m": height_std,
            "pitch_std_deg": pitch_std,
            "roll_std_deg": roll_std,
            "observations": len(heights),
            "source_dir": str(Path(arguments.image_dir).resolve()),
            "intrinsics": arguments.intrinsics,
            "tag_mm": arguments.tag_mm,
            "note": "ヨーと前後左右のオフセットは床タグでは測れない。既定 0",
        }
        Path(arguments.out).write_text(json.dumps(payload, indent=2, ensure_ascii=False))
        print(f"\n書いた: {arguments.out}")

    if height_std > 0.02:
        print("\n⚠️ 高さのばらつきが 20 mm を超えた。紙がたわんでいるか、内部パラメータが甘い")
    if abs(roll_mean) > 5.0:
        print(f"⚠️ ロールが {roll_mean:.1f} deg ある。機体が傾いているか、タグが床に平らでない")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
