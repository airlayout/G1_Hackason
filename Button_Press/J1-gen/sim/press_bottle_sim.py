"""全体をつなぐスクリプト（MuJoCo、同じプロセスの中）: 検出 → 目標 → IK → 手前の姿勢 → 押し込み → 戻る（タスク6）。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/press_bottle_sim.py
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/press_bottle_sim.py --target manual --point 0.39 -0.2 0.04

MuJoCo（胴体固定、机とボトル、ハンドの衝突判定の箱あり）を同じプロセスで動かし、頭カメラは MuJoCo の描画、
検出は既定でセグメンテーション（YOLO は作り物のボトルを検出しないため）。流れと記録は実機と同じ（common/pipeline.py）。
実機用のスクリプトをそのまま試したいときは、ループバックの模擬ロボット（sim/sim_robot_server.py）を使う。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.arm import make_backend  # noqa: E402
from common.config import load_config  # noqa: E402
from common.localize import make_detector  # noqa: E402
from common.pipeline import PipelineResult, PressPipeline  # noqa: E402
from common.pipeline_cli import add_common_args, load_pipeline_config, make_logger  # noqa: E402
from common.post_press import PostPressCheck  # noqa: E402
from common.sim_camera import SegmentationDetector, SimHeadCamera  # noqa: E402
from common.sim_scene import BOTTLE, detection_pose  # noqa: E402


def run_sim(args: argparse.Namespace, post_check: PostPressCheck | None = None,
            arm_overrides: dict[str, Any] | None = None) -> tuple[PipelineResult, Any, Any]:
    """MuJoCo で全体の流れを実行する。(結果, バックエンド, 記録) を返す（テストで手先の位置を測るため）。"""
    args.sim_scene_obstacles = True  # シーンの机を障害物として衝突の確認に入れる
    cfg = load_pipeline_config(args)
    scene = load_config("sim_scene.yaml")
    sim = dict(cfg.arm["sim"])
    sim.update(scene=scene, initial_q=list(detection_pose(scene, np.zeros(29))), realtime=False)
    cfg.arm["sim"] = sim
    for k, v in (arm_overrides or {}).items():
        cfg.arm[k] = v
    backend = make_backend(cfg.arm, cfg.robot, dry_run=False, path="sim")
    backend.open()
    camera = SimHeadCamera(backend.model, backend.data, cfg.robot)
    det_type = getattr(args, "sim_detector", "segmentation")
    detector = SegmentationDetector(camera, BOTTLE) if det_type == "segmentation" else make_detector(cfg.localize)
    logger = make_logger(cfg, args.label or "sim", {"kind": "sim", "detector": det_type})
    pipe = PressPipeline(cfg, backend, logger, camera.render, detector, confirm=getattr(args, "confirm", False),
                         post_check=post_check)
    try:
        res = pipe.run()
    finally:
        camera.close()
    return res, backend, logger


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    p.add_argument("--sim-detector", choices=["segmentation", "color", "yolo"], default="segmentation")
    p.add_argument("--confirm", action="store_true", help="各段階で Enter を待つ")
    args = p.parse_args()
    if args.sim_detector in ("color", "yolo"):
        args.detector = args.sim_detector
    res, _, logger = run_sim(args)
    print(f"[press_bottle_sim] 記録: {logger.dir}")
    return res.code


if __name__ == "__main__":
    sys.exit(main())
