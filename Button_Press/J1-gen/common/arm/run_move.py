"""腕を指定の関節角へ動かして戻す（sim / real 共通の中身）。

sim/move_arm_sim.py と real/move_arm_real.py から呼ぶ。目標は次のどちらかで与える:
- --target-deg: 片腕 7 関節の絶対角度 [deg]（shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
  wrist_roll, wrist_pitch, wrist_yaw の順）
- --delta-deg:  開始時の姿勢からの変化 [deg]（同じ順。既定は小さな動き）
"""

from __future__ import annotations

import argparse
from typing import Any

import numpy as np

from ..config import load_config
from .gravity import GravityModel, needs_gravity_model
from . import (
    ArmCommander,
    NoMotionError,
    StateTimeoutError,
    StopRequested,
    UnsafeTargetError,
    WaistDeviationError,
    joint_limits,
    make_backend,
)

ARM_JOINT_LABELS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                    "wrist_roll", "wrist_pitch", "wrist_yaw")


def add_common_args(p: argparse.ArgumentParser, default_delta: list[float]) -> None:
    p.add_argument("--arm-config", default="arm.yaml", help="腕の設定（configs/ 基準）")
    p.add_argument("--robot-config", default="robot.yaml", help="機体の設定（configs/ 基準）")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--target-deg", type=float, nargs=7, metavar="DEG", help="片腕 7 関節の絶対角度 [deg]")
    g.add_argument("--delta-deg", type=float, nargs=7, metavar="DEG", default=default_delta,
                   help=f"開始姿勢からの変化 [deg]（既定 {default_delta}）")
    p.add_argument("--arm", choices=["left", "right"], help="動かす腕（既定は設定ファイルの値）")
    p.add_argument("--duration", type=float, help="移動にかける秒数（既定は上限速度から自動）")
    p.add_argument("--no-return", action="store_true", help="目標へ動かしたあと、開始姿勢へ戻さない")
    p.add_argument("--gravity-scale", type=float,
                   help="腕の重力補償の倍率 0.0〜1.0（既定は設定ファイル。実機日は 0 → 0.5 → 1.0 で比べる）")
    p.add_argument("--waist-gravity-scale", type=float,
                   help="腰の重力補償の倍率 0.0〜1.0（プランB のときだけ効く。既定は設定ファイル）")


def run(args: argparse.Namespace, path: str, dry_run: bool, confirm: bool,
        overrides: dict[str, Any] | None = None) -> int:
    arm_cfg = load_config(args.arm_config)
    robot_cfg = load_config(args.robot_config)
    if args.arm:
        arm_cfg["arm"] = args.arm
    if args.gravity_scale is not None:
        arm_cfg["gravity_compensation"]["scale"] = args.gravity_scale
    if args.waist_gravity_scale is not None:
        arm_cfg["gravity_compensation"]["waist_scale"] = args.waist_gravity_scale
    for k, v in (overrides or {}).items():
        arm_cfg[k] = v

    lower, upper = joint_limits(robot_cfg)
    backend = make_backend(arm_cfg, robot_cfg, dry_run=dry_run, path=path)
    try:
        backend.open()
        gravity = GravityModel(robot_cfg) if needs_gravity_model(arm_cfg) else None
        with ArmCommander(backend, arm_cfg, lower, upper, confirm=confirm, gravity=gravity) as arm:
            q_start = arm.commanded_arm_q
            if args.target_deg is not None:
                q_goal = np.radians(args.target_deg)
            else:
                q_goal = q_start + np.radians(args.delta_deg)
            print("[move] 開始 [deg]: " + _fmt(q_start))
            print("[move] 目標 [deg]: " + _fmt(q_goal))
            arm.move_to(q_goal, duration=args.duration, label="目標へ移動")
            if not args.no_return:
                arm.move_to(q_start, duration=args.duration, label="開始姿勢へ戻る")
        if arm.stop_reason:
            print(f"[move] 中止: {arm.stop_reason}")
            return 130
        return 0
    except UnsafeTargetError as e:
        print(f"[move] 目標を拒否した（送信していない）: {e}")
        return 2
    except ValueError as e:
        print(f"[move] 設定の誤り: {e}")
        return 2
    except NoMotionError as e:
        print(f"[move] ❌ {e}")
        return 3
    except WaistDeviationError as e:
        print(f"[move] ❌ {e}")
        return 5
    except StateTimeoutError as e:
        print(f"[move] ❌ {e}")
        return 4
    except StopRequested as e:
        print(f"[move] 中止: {e}")
        return 130
    except RuntimeError as e:  # モータが無効（ゼロトルク）、mode_machine が違う、相手の食い違い、など
        print(f"[move] ❌ {e}")
        return 4
    finally:
        backend.close()


def _fmt(q: np.ndarray) -> str:
    return ", ".join(f"{n}={d:+.1f}" for n, d in zip(ARM_JOINT_LABELS, np.degrees(q)))
