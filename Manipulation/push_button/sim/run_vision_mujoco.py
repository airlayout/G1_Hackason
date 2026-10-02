#!/usr/bin/env python3
"""MuJoCo の RGB-D 像からボタンを見つけ、推定目標で実際に押す。"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision import CameraIntrinsics, estimate_button_target, validate_reachable_target
from trajectory import DEFAULT_BUTTON_STROKE_M

from run_mujoco import (BUTTON_HALF_DEPTH, BUTTON_VERTICAL_SPACING,
                        DEFAULT_TIP_OFFSET, build_model, run)


WIDTH = 640
HEIGHT = 480


def capture(args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray, np.ndarray,
                                                CameraIntrinsics, np.ndarray]:
    model, stand_q = build_model(Path(args.model).expanduser().resolve(),
                                 args.button_x, args.button_y, args.height,
                                 args.stroke, tuple(args.tip_offset),
                                 args.fixed_base, rgbd_camera=True,
                                 button_direction=args.button_direction)
    data = mujoco.MjData(model)
    if args.fixed_base:
        data.qpos[:29] = stand_q
    else:
        data.qpos[:7] = (0, 0, 0.79, 1, 0, 0, 0)
        data.qpos[7:36] = stand_q
    data.ctrl[:] = stand_q
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    try:
        renderer.update_scene(data, camera="button_rgbd")
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
        renderer.enable_segmentation_rendering()
        segmentation = renderer.render().copy()
        renderer.disable_segmentation_rendering()
    finally:
        renderer.close()

    camera_id = model.camera("button_rgbd").id
    camera_rotation = data.cam_xmat[camera_id].reshape(3, 3)
    # MuJoCo カメラは +X=右、+Y=上、-Z=前。光学座標は +Y=下、+Z=前。
    T_world_optical = np.eye(4)
    T_world_optical[:3, :3] = camera_rotation @ np.diag([1, -1, -1])
    T_world_optical[:3, 3] = data.cam_xpos[camera_id]
    focal = HEIGHT / (2 * math.tan(math.radians(model.cam_fovy[camera_id]) / 2))
    intrinsics = CameraIntrinsics(focal, focal, WIDTH / 2, HEIGHT / 2)
    return rgb, depth, segmentation, intrinsics, T_world_optical


def oracle_bbox(segmentation: np.ndarray, model_geom_id: int) -> tuple[float, float, float, float]:
    mask = ((segmentation[:, :, 0] == model_geom_id)
            & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)))
    rows, cols = np.where(mask)
    if len(rows) < 20:
        raise ValueError("模擬ボタンがカメラから見えません")
    return (float(cols.min()), float(rows.min()),
            float(cols.max() + 1), float(rows.max() + 1))


def yolo_bboxes(rgb: np.ndarray, weights: str,
                confidence: float) -> list[tuple[float, float, float, float]]:
    # 既存の Perception 検出器を利用する。モデルは同じ Ultralytics 形式の .pt。
    repo_root = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(repo_root / "Perception"))
    from common.detector.yolo_detector import YoloDetector

    detector = YoloDetector(model_name=weights, classes=["button"],
                            confidence_threshold=confidence)
    # YoloDetector は OpenCV の BGR 入力を受ける。
    detections = detector.detect(rgb[:, :, ::-1].copy())
    if not detections:
        raise ValueError("公開 YOLO モデルはこの MuJoCo 画像でボタンを検出できません")
    return [detection.bbox for detection in detections]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Menagerie の unitree_g1/g1.xml")
    parser.add_argument("--weights", help="公開/追加学習済み YOLO の .pt")
    parser.add_argument("--detector", choices=("oracle", "yolo"),
                        help="省略時は --weights があれば yolo、なければ oracle")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--button-x", type=float, default=0.36)
    parser.add_argument("--button-y", type=float, default=-0.25)
    parser.add_argument("--height", type=float, default=1.0)
    parser.add_argument("--button-direction", choices=("up", "down"), default="up",
                        help="押すボタン。--height は上ボタンの中心高さ")
    parser.add_argument("--stroke", type=float, default=DEFAULT_BUTTON_STROKE_M)
    parser.add_argument("--clearance", type=float, default=0.030)
    parser.add_argument("--tip-offset", type=float, nargs=3,
                        default=list(DEFAULT_TIP_OFFSET))
    parser.add_argument("--fixed-base", action="store_true")
    parser.add_argument("--plan-out")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--snapshot", help="検出枠を描いた MuJoCo RGB 画像の保存先 PNG")
    parser.add_argument("--gif-out", help="押下動作をGIFで保存")
    parser.add_argument("--gif-fps", type=int, default=10)
    args = parser.parse_args()
    detector = args.detector or ("yolo" if args.weights else "oracle")
    if detector == "yolo" and not args.weights:
        parser.error("--detector yolo には --weights が必要です")
    if not 1 <= args.gif_fps <= 30:
        parser.error("--gif-fps は1～30で指定してください")

    rgb, depth, segmentation, intrinsics, transform = capture(args)
    if detector == "oracle":
        # セグメンテーション ID はモデル構築順で固定。検出器を切り分ける試験専用。
        model, _ = build_model(Path(args.model).expanduser().resolve(),
                               args.button_x, args.button_y, args.height,
                               args.stroke, tuple(args.tip_offset),
                               args.fixed_base, rgbd_camera=True,
                               button_direction=args.button_direction)
        bboxes = [oracle_bbox(segmentation,
                              model.geom(f"button_face_{args.button_direction}").id)]
    else:
        bboxes = yolo_bboxes(rgb, args.weights, args.confidence)

    valid_targets = []
    for candidate_bbox in bboxes:
        try:
            candidate = estimate_button_target(depth, candidate_bbox, intrinsics, transform)
            validate_reachable_target(candidate)
        except ValueError as error:
            print(f"[vision] 候補棄却 bbox={candidate_bbox}: {error}")
            continue
        valid_targets.append((candidate_bbox, candidate))
    # YOLO の button 1クラスには上下の意味がない。既知のパネル配置と
    # 指定した上/下の大まかな高さから候補を分ける。3D座標そのものは画像から推定する。
    target_height = (args.height if args.button_direction == "up"
                     else args.height - BUTTON_VERTICAL_SPACING)
    selected = [(bbox, target) for bbox, target in valid_targets
                if abs(target.face_xyz[2] - target_height) < 0.075]
    if len(selected) != 1:
        print(f"RESULT_REJECTED: {args.button_direction} の有効なボタン候補数={len(selected)}"
              f" (全候補 {len(valid_targets)})",
              file=sys.stderr)
        return 2
    bbox, target = selected[0]

    if args.snapshot:
        from PIL import Image, ImageDraw

        snapshot = Image.fromarray(rgb)
        draw = ImageDraw.Draw(snapshot)
        for candidate_bbox in bboxes:
            draw.rectangle(candidate_bbox, outline="red", width=2)
        draw.rectangle(bbox, outline="lime", width=3)
        draw.ellipse((sum(bbox[::2]) / 2 - 4, sum(bbox[1::2]) / 2 - 4,
                      sum(bbox[::2]) / 2 + 4, sum(bbox[1::2]) / 2 + 4),
                     fill="lime")
        output = Path(args.snapshot)
        output.parent.mkdir(parents=True, exist_ok=True)
        snapshot.save(output)

    truth = np.array([args.button_x - BUTTON_HALF_DEPTH, args.button_y, target_height])
    error_mm = float(np.linalg.norm(np.array(target.face_xyz) - truth) * 1000)
    print(f"[vision] detector={detector} bbox={tuple(round(v, 1) for v in bbox)}")
    print(f"[vision] face_xyz={target.face_xyz}, press_direction={target.press_direction}")
    print(f"[vision] 中心誤差={error_mm:.1f} mm, 中央深度有効率="
          f"{target.depth_valid_fraction:.2f}, 面内点率={target.plane_inlier_fraction:.2f}")
    try:
        plan = run(args, target_face_xyz=np.array(target.face_xyz),
                   press_direction=np.array(target.press_direction))
    except ValueError as error:
        print(f"RESULT_REJECTED: {error}", file=sys.stderr)
        return 2
    return 0 if plan["sim_result"]["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
