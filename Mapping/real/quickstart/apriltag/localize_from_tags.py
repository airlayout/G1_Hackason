#!/usr/bin/env python3
"""タグを見て、ロボットが地図のどこにいるかを出す（段 5）。

    T_map_base = T_map_tag ・ (T_base_camera ・ T_camera_tag)⁻¹

登録済みのタグが **1 枚見えれば姿勢が決まる**。大域探索も初期値もチューニングも要らない。

## この道具が自分で自分を検査する 3 つの口

1. **タグ間の食い違い** … 2 枚以上見えているとき、それぞれが出す姿勢が一致するか。
   ずれていれば登録か取付が狂っている（1 枚だけだと検査できない）
2. **z / roll / pitch** … 求まった姿勢の z はほぼ 0、roll と pitch もほぼ 0 のはず。
   計算にこれらを拘束していないので、**独立な裏取り**になる
3. **枚ごとのばらつき** … 静止しているのに揺れていれば、検出か取付が怪しい

⚠️ **平面の 2 解の曖昧さが大きい観測は捨てる**（`tag_detect.AMBIGUITY_LIMIT`）。
正面に近いタグは向きが鏡像に転びやすく、**黙って数十度ずれた姿勢を出す**。

## 使い方

    # 録ってある画像から
    python3 localize_from_tags.py --session ~/g1_cfg/runs/regP \\
        --registry ~/g1_cfg/apriltag/tag_registry.json \\
        --extrinsics ~/g1_cfg/apriltag/extrinsics.json

    # いまカメラに写っているもので（PC2）
    python3 localize_from_tags.py --live --seconds 5 \\
        --registry ~/g1_cfg/apriltag/tag_registry.json \\
        --extrinsics ~/g1_cfg/apriltag/extrinsics.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tag_detect
from register_tags import (camera_transform, matrix_to_quaternion,
                           quaternion_to_matrix, transform, yaw_of)

# 静止して見ているのに姿勢がこれ以上揺れたら、検出か取付を疑う。
SCATTER_WARN_M = 0.05
# タグ間でこれ以上食い違ったら、登録か取付が狂っている。
TAG_GAP_WARN_M = 0.10
# 求まった姿勢の z / roll / pitch がこれを超えたら、鎖のどこかが狂っている。
LEVEL_WARN_M = 0.10
LEVEL_WARN_DEG = 5.0


def euler_from_matrix(rotation: np.ndarray) -> tuple:
    """roll, pitch, yaw [deg]（R = Rz(yaw) Ry(pitch) Rx(roll)）。"""

    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def average_transforms(matrices: list) -> np.ndarray:
    """4x4 の集まりを平均する（位置は中央値、回転は四元数の平均）。"""

    positions = np.array([m[:3, 3] for m in matrices])
    quaternions = np.array([matrix_to_quaternion(m[:3, :3]) for m in matrices])
    reference = quaternions[0]
    aligned = np.array([q if q @ reference >= 0 else -q for q in quaternions])
    mean_quaternion = aligned.mean(axis=0)
    mean_quaternion /= np.linalg.norm(mean_quaternion)
    return transform(quaternion_to_matrix(mean_quaternion), np.median(positions, axis=0))


def capture_live(seconds: float, width: int, height: int, fps: int, device: str) -> list:
    """いまカメラに写っているものを集める（PC2 でだけ動く）。"""

    from see_tags import open_ir
    capture = open_ir(device, width, height, fps)
    frames = []
    try:
        for _ in range(12):            # 自動露出が落ち着くまで捨てる
            capture.read()
        import time
        started = time.time()
        while time.time() - started < seconds:
            ok, frame = capture.read()
            if ok:
                frames.append(tag_detect.to_gray(frame))
    finally:
        capture.release()
    return frames


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--registry", required=True)
    parser.add_argument("--extrinsics", required=True)
    parser.add_argument("--session", default=None, help="画像の置き場")
    parser.add_argument("--pattern", default="raw_*.png")
    parser.add_argument("--live", action="store_true", help="カメラから直接")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--tag-mm", type=float, default=None,
                        help="既定は registry に書いてある値")
    parser.add_argument("--intrinsics", default=None)
    parser.add_argument("--hfov-deg", type=float, default=87.0)
    parser.add_argument("--out", default=None, help="求めた姿勢を JSON で書く")
    arguments = parser.parse_args()

    registry = json.loads(Path(arguments.registry).read_text())
    extrinsics = json.loads(Path(arguments.extrinsics).read_text())
    tag_m = (arguments.tag_mm / 1000.0 if arguments.tag_mm
             else float(registry.get("tag_size_m", 0.160)))
    family = registry.get("tag_family", "36h11")
    base_from_camera = camera_transform(extrinsics, registry.get("camera_yaw_deg"))

    map_from_tag = {}
    for tag_id, entry in registry["tags"].items():
        map_from_tag[int(tag_id)] = transform(
            quaternion_to_matrix(np.array(entry["quaternion"])),
            np.array(entry["position"]))
    print(f"登録済みのタグ: {sorted(map_from_tag)} / 一辺 {tag_m*1000:.0f} mm / {family}")

    if arguments.intrinsics:
        camera_matrix, distortion, _ = tag_detect.load_intrinsics(arguments.intrinsics)
    else:
        camera_matrix = tag_detect.default_camera_matrix(
            arguments.width, arguments.height, arguments.hfov_deg)
        distortion = np.zeros((1, 5))

    if arguments.live:
        images = capture_live(arguments.seconds, arguments.width, arguments.height,
                              arguments.fps, arguments.device)
        print(f"カメラから {len(images)} 枚")
    elif arguments.session:
        paths = sorted(Path(arguments.session).glob(arguments.pattern))
        images = [cv2.imread(str(p), cv2.IMREAD_GRAYSCALE) for p in paths]
        images = [im for im in images if im is not None]
        print(f"{arguments.session} から {len(images)} 枚")
    else:
        parser.error("--session か --live のどちらかが要る")

    detector = tag_detect.make_detector(family)
    per_tag: dict = {}
    dropped = {"未登録": 0, "曖昧": 0}
    for image in images:
        for found in tag_detect.detect(detector, image):
            if found.tag_id not in map_from_tag:
                dropped["未登録"] += 1
                continue
            try:
                pose = tag_detect.solve_pose(found, tag_m, camera_matrix, distortion)
            except RuntimeError:
                continue
            if not pose.trustworthy:
                dropped["曖昧"] += 1
                continue
            base_from_tag = base_from_camera @ pose.matrix()
            per_tag.setdefault(found.tag_id, []).append(
                map_from_tag[found.tag_id] @ np.linalg.inv(base_from_tag))

    if not per_tag:
        print("⛔ 登録済みのタグが 1 枚も読めなかった。"
              f"（未登録 {dropped['未登録']} 件 / 曖昧で捨てた {dropped['曖昧']} 件）")
        return 1

    print(f"\n=== タグごとに出した姿勢 ===")
    print("ID    x        y      yaw       z       roll   pitch   観測   ばらつき")
    per_tag_mean = {}
    for tag_id in sorted(per_tag):
        matrices = per_tag[tag_id]
        mean = average_transforms(matrices)
        per_tag_mean[tag_id] = mean
        positions = np.array([m[:3, 3] for m in matrices])
        scatter = float(np.linalg.norm(positions - mean[:3, 3], axis=1).std())
        roll, pitch, yaw = euler_from_matrix(mean[:3, :3])
        print("%-4d %+7.3f %+7.3f %+8.2f  %+6.3f  %+6.2f  %+6.2f   %4d   %5.0f mm" % (
            tag_id, mean[0, 3], mean[1, 3], yaw, mean[2, 3], roll, pitch,
            len(matrices), scatter * 1000))

    combined = average_transforms([m for group in per_tag.values() for m in group])
    roll, pitch, yaw = euler_from_matrix(combined[:3, :3])
    print(f"\n=== まとめた姿勢 ===")
    print("  **(x %+.3f, y %+.3f, yaw %+.2f deg)**" % (combined[0, 3], combined[1, 3], yaw))

    print(f"\n=== 自分で自分を検査する ===")
    verdict = 0
    if len(per_tag_mean) >= 2:
        points = np.array([m[:3, 3] for m in per_tag_mean.values()])
        gap = float(np.linalg.norm(points - points.mean(axis=0), axis=1).max() * 2)
        yaws = [euler_from_matrix(m[:3, :3])[2] for m in per_tag_mean.values()]
        yaw_gap = max(yaws) - min(yaws)
        mark = "✅" if gap < TAG_GAP_WARN_M else "⛔"
        print(f"  {mark} タグ間の食い違い: **{gap*1000:.0f} mm / {yaw_gap:+.2f} deg**"
              f"（{TAG_GAP_WARN_M*1000:.0f} mm 未満なら良い）")
        if gap >= TAG_GAP_WARN_M:
            verdict = 1
    else:
        print("  ⚠️ タグが 1 枚だけ。**タグ間の食い違いは検査できない**")

    level_ok = abs(combined[2, 3]) < LEVEL_WARN_M and abs(roll) < LEVEL_WARN_DEG \
        and abs(pitch) < LEVEL_WARN_DEG
    print(f"  {'✅' if level_ok else '⛔'} z {combined[2,3]:+.3f} m / "
          f"roll {roll:+.2f} deg / pitch {pitch:+.2f} deg"
          f"（計算に拘束していない量。0 に近ければ鎖が通っている）")
    if not level_ok:
        verdict = 1

    all_positions = np.array([m[:3, 3] for group in per_tag.values() for m in group])
    scatter = float(np.linalg.norm(all_positions - combined[:3, 3], axis=1).std())
    print(f"  {'✅' if scatter < SCATTER_WARN_M else '⚠️'} 枚ごとのばらつき {scatter*1000:.0f} mm")
    if dropped["曖昧"]:
        print(f"  ⚠️ 平面の曖昧さで捨てた観測 {dropped['曖昧']} 件")

    print(f"\n=== これを測位に渡すなら ===")
    print(f"  bash ~/g1_cfg/apriltag/amcl_kick.sh "
          f"{combined[0,3]:.3f} {combined[1,3]:.3f} {yaw:.2f}")

    if arguments.out:
        Path(arguments.out).write_text(json.dumps({
            "x": float(combined[0, 3]), "y": float(combined[1, 3]), "yaw_deg": float(yaw),
            "z": float(combined[2, 3]), "roll_deg": roll, "pitch_deg": pitch,
            "tags": {str(k): {"x": float(v[0, 3]), "y": float(v[1, 3]),
                              "yaw_deg": euler_from_matrix(v[:3, :3])[2],
                              "observations": len(per_tag[k])}
                     for k, v in per_tag_mean.items()},
            "scatter_m": scatter, "images": len(images),
        }, indent=2, ensure_ascii=False))
        print(f"\n書いた: {arguments.out}")
    return verdict


if __name__ == "__main__":
    raise SystemExit(main())
