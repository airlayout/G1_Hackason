"""MuJoCo（胴体固定）で、指定した点を押す動作を確かめる（タスク2）。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/press_sim.py --target 0.40 -0.20 0.05 --push-dir 1 0 0

目標は pelvis 座標 [m]（x 前方、y 左、z 上）。IK と軌道は URDF（Pinocchio）で計算し、
MuJoCo（XML）で動かして、手先が手前の点・押し込み終わりの点にどれだけ近づいたかを表示する。
届かない・ぶつかる目標は、何も動かさずに拒否する。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.arm import ArmCommander, StopRequested, UnsafeTargetError, joint_limits, make_backend  # noqa: E402
from common.arm.gravity import GravityModel  # noqa: E402
from common.config import load_config  # noqa: E402
from common.press import execute_press  # noqa: E402
from common.press_planner import PressPlanner  # noqa: E402


def mujoco_fingertip(backend: Any, robot_cfg: dict[str, Any], side: str) -> np.ndarray:
    """MuJoCo（XML）上の手先の位置（pelvis 座標）。IK（URDF）とは別の計算で確かめるため。"""
    m, d = backend.model, backend.data
    ee = robot_cfg["end_effector"][side]
    b = m.body(ee["link"]).id
    pel = m.body("pelvis").id
    rp = d.xmat[pel].reshape(3, 3)
    p = d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.asarray(ee["offset"])
    return rp.T @ (p - d.xpos[pel])


def run_press_sim(
    target: np.ndarray,
    push_dir: np.ndarray,
    side: str = "right",
    arm_overrides: dict[str, Any] | None = None,
    confirm: bool = False,
) -> dict[str, float]:
    """押し込みを MuJoCo で実行し、手先の誤差 [m] を返す。拒否されたら UnsafeTargetError。"""
    robot_cfg = load_config("robot.yaml")
    press_cfg = load_config("press.yaml")
    arm_cfg = load_config("arm.yaml")
    arm_cfg["arm"] = side
    for k, v in (arm_overrides or {}).items():
        arm_cfg[k] = v
    lower, upper = joint_limits(robot_cfg)
    planner = PressPlanner.from_config(robot_cfg, press_cfg, arm_cfg, side)
    backend = make_backend(arm_cfg, robot_cfg, dry_run=False, path="sim")
    backend.open()

    # 動かす前に計画する（dry-run と同じく、ここで拒否されれば何も送らない）
    q_now = backend.read_state().q
    plan = planner.plan(q_now, target, push_dir)
    errs: dict[str, float] = {}
    fk = planner.kin.fk_pos
    gravity = GravityModel(robot_cfg) if float(arm_cfg["gravity_compensation"]["scale"]) > 0 else None
    with ArmCommander(backend, arm_cfg, lower, upper, fk=fk, workspace=planner.workspace,
                      confirm=confirm, gravity=gravity) as arm:
        q_start = arm.commanded_arm_q
        arm.move_to(plan.q_approach, label="手前の姿勢へ移動")
        errs["approach"] = float(np.linalg.norm(mujoco_fingertip(backend, robot_cfg, side) - plan.approach_point))
        arm.follow(plan.press_in, label="押し込み")
        errs["end"] = float(np.linalg.norm(mujoco_fingertip(backend, robot_cfg, side) - plan.end_point))
        arm.follow(plan.press_out, label="戻り")
        arm.move_to(q_start, label="開始姿勢へ戻る")
    if arm.stop_reason:
        raise StopRequested(arm.stop_reason)
    return errs


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", type=float, nargs=3, default=[0.40, -0.20, 0.05], metavar=("X", "Y", "Z"))
    p.add_argument("--push-dir", type=float, nargs=3, default=[1.0, 0.0, 0.0], metavar=("X", "Y", "Z"))
    p.add_argument("--arm", choices=["left", "right"], default="right")
    p.add_argument("--confirm", action="store_true", help="各段階で Enter を待つ")
    args = p.parse_args()
    try:
        errs = run_press_sim(np.array(args.target), np.array(args.push_dir), args.arm, confirm=args.confirm)
    except UnsafeTargetError as e:
        print(f"[press_sim] 目標を拒否した（何も動かしていない）: {e}")
        return 2
    except StopRequested as e:
        print(f"[press_sim] 中止: {e}")
        return 130
    print(f"[press_sim] MuJoCo 上の手先の誤差: 手前の点 {errs['approach'] * 1000:.1f} mm、"
          f"押し込み終わり {errs['end'] * 1000:.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
