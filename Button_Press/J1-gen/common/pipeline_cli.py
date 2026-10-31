"""全体をつなぐスクリプト（real/press_bottle.py、sim/press_bottle_sim.py）で共通の引数と設定の読み込み。"""

from __future__ import annotations

import argparse
import sys
from typing import Any

from .config import deep_merge, load_config, resolve_repo_path
from .pipeline import PipelineConfig
from .run_logger import RunLogger


def add_common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--overlay", help="動作ごとの設定の上書き（configs/ の YAML。例: knock.yaml）")
    p.add_argument("--label", default="", help="記録のフォルダ名に付ける言葉")
    p.add_argument("--arm", choices=["left", "right"], help="動かす腕（既定は configs/pipeline.yaml）")
    p.add_argument("--target", choices=["depth", "manual"], help="対象の位置の与え方（既定は pipeline.yaml）")
    p.add_argument("--point", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="manual: 対象の点（pelvis 座標 [m]）")
    p.add_argument("--taught-pose", help="manual: 教えた姿勢の名前（その中指の先 + --offset の点を対象にする）")
    p.add_argument("--offset", type=float, nargs=3, metavar=("X", "Y", "Z"), help="manual: 定規で測ったずれ [m]")
    p.add_argument("--target-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="depth: 測った面の点から、押す点をずらす量 [m]（pelvis 座標。既定は pipeline.yaml）")
    p.add_argument("--seed-pose", help="IK の初期値に使う教えた姿勢（既定は pipeline.yaml）")
    p.add_argument("--depth-mm", type=float, help="押し込みの深さ [mm]（既定は press.yaml。上限で頭打ち）")
    p.add_argument("--speed-mm-s", type=float, help="押し込みの速さ [mm/s]（既定は press.yaml）")
    p.add_argument("--detector", choices=["yolo", "onnx", "color"],
                   help="検出器（既定は localize.yaml。onnx は torch なしで動く。G1 の PC2 で使う）")
    p.add_argument("--gravity-scale", type=float, help="腕の重力補償の倍率（既定は arm.yaml）")
    p.add_argument("--log-dir", help="記録の保存先（既定は pipeline.yaml の log_dir）")
    p.add_argument("--obstacles-file", help="teach.py --obstacle で作った箱のファイル（既定は configs/obstacles.yaml。"
                                            "--sim-scene-obstacles のときは指定しなければ読まない）")
    p.add_argument("--sim-scene-obstacles", action="store_true",
                   help="configs/sim_scene.yaml の机を障害物に足す（模擬ロボットで試すとき）")


def load_pipeline_config(args: argparse.Namespace) -> PipelineConfig:
    cfg = PipelineConfig(robot=load_config("robot.yaml"), arm=load_config("arm.yaml"),
                         press=load_config("press.yaml"), localize=load_config("localize.yaml"),
                         pipeline=load_config("pipeline.yaml"))
    # 動作ごとの上書き（例: configs/knock.yaml）。press / localize / pipeline / arm の設定に重ねる。
    # コマンドラインの引数は、これより優先する
    overlay = getattr(args, "overlay", None)
    if overlay:
        over = load_config(overlay)
        for name in ("press", "localize", "pipeline", "arm"):
            deep_merge(getattr(cfg, name), over.get(name) or {})
        print(f"[pipeline] 設定の上書き: {overlay}")
    pl = cfg.pipeline
    if args.arm:
        pl["arm"] = args.arm
    if args.target:
        pl["target"]["source"] = args.target
    if args.point is not None:
        pl["target"]["manual"]["point_pelvis_m"] = list(args.point)
    if args.taught_pose:
        pl["target"]["manual"]["taught_pose"] = args.taught_pose
        pl["target"]["manual"]["point_pelvis_m"] = None if args.point is None else list(args.point)
    if args.offset is not None:
        pl["target"]["manual"]["offset_m"] = list(args.offset)
    if getattr(args, "target_offset", None) is not None:
        pl["target"]["depth"]["offset_m"] = list(args.target_offset)
    if args.seed_pose is not None:
        pl["seed_pose"] = args.seed_pose or None
    if args.depth_mm is not None:
        cfg.press["press"]["press_depth_m"] = args.depth_mm / 1000.0
    if getattr(args, "speed_mm_s", None) is not None:
        cfg.press["press"]["press_speed_m_s"] = args.speed_mm_s / 1000.0
    if args.detector:
        cfg.localize["detector"]["type"] = args.detector
    if args.gravity_scale is not None:
        cfg.arm["gravity_compensation"]["scale"] = args.gravity_scale
    if getattr(args, "log_dir", None):
        pl["log_dir"] = args.log_dir
    # teach.py --obstacle で作った箱（configs/obstacles.yaml）も、衝突の確認に使う。
    # シミュレーションの机を使うとき（--sim-scene-obstacles）は、実機の机の箱は --obstacles-file で指定したときだけ読む
    from .realday import OBSTACLES, load_obstacles

    obs_file = getattr(args, "obstacles_file", None)
    if obs_file is None and not getattr(args, "sim_scene_obstacles", False):
        obs_file = OBSTACLES
    extra = load_obstacles(obs_file) if obs_file else []
    if extra:
        cfg.press["obstacles"] = list(cfg.press.get("obstacles") or []) + extra
        print(f"[pipeline] 障害物の箱（{obs_file}）: " + ", ".join(o.get("name", "?") for o in extra))
    if getattr(args, "sim_scene_obstacles", False):
        from .sim_scene import table_obstacle

        scene = load_config(getattr(args, "sim_scene_name", "sim_scene.yaml"))
        cfg.press["obstacles"] = list(cfg.press.get("obstacles") or []) + [table_obstacle(scene)]
    return cfg


def make_logger(cfg: PipelineConfig, label: str, extra: dict[str, Any]) -> RunLogger:
    meta = {"argv": sys.argv, "configs": {"robot": cfg.robot, "arm": cfg.arm, "press": cfg.press,
                                          "localize": cfg.localize, "pipeline": cfg.pipeline}}
    meta.update(extra)
    return RunLogger(resolve_repo_path(cfg.pipeline["log_dir"]), label, meta)
