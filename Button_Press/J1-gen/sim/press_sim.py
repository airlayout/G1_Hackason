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
from common.arm.gravity import GravityModel, needs_gravity_model  # noqa: E402
from common.config import load_config  # noqa: E402
from common.press import execute_press  # noqa: E402
from common.press_planner import PressPlan, PressPlanner  # noqa: E402


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
    replan: bool = True,
) -> dict[str, float]:
    """押し込みを MuJoCo で実行し、手先の誤差 [m] を返す。拒否されたら UnsafeTargetError。

    replan=True なら、手前の姿勢に着いた時点の実測の腰の角度で軌道を計算し直す（実機と同じ流れ）。
    返す誤差: approach（手前の点）、end（押し込み終わりの点）。
    """
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

    def measure(stage: str, p: PressPlan) -> None:
        tip = mujoco_fingertip(backend, robot_cfg, side)
        if stage not in ("approach", "approach_replanned", "end"):
            return
        ref = p.end_point if stage == "end" else p.approach_point
        key = "end" if stage == "end" else "approach"
        errs[key] = float(np.linalg.norm(tip - ref))

    gravity = GravityModel(robot_cfg) if needs_gravity_model(arm_cfg) else None
    with ArmCommander(backend, arm_cfg, lower, upper, fk=planner.kin.fk_pos, workspace=planner.workspace,
                      confirm=confirm, gravity=gravity) as arm:
        execute_press(arm, plan, hold_s=float(press_cfg["press"]["hold_s"]), return_to=arm.commanded_arm_q,
                      planner=planner, on_stage=measure, replan=replan)
    if arm.stop_reason:
        raise StopRequested(arm.stop_reason)
    return errs


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", type=float, nargs=3, default=[0.40, -0.20, 0.05], metavar=("X", "Y", "Z"))
    p.add_argument("--push-dir", type=float, nargs=3, default=[1.0, 0.0, 0.0], metavar=("X", "Y", "Z"))
    p.add_argument("--arm", choices=["left", "right"], default="right")
    p.add_argument("--confirm", action="store_true", help="各段階で Enter を待つ")
    p.add_argument("--no-replan", action="store_true", help="手前の姿勢で腰の角度を読み直して計算し直すのをやめる")
    args = p.parse_args()
    try:
        errs = run_press_sim(np.array(args.target), np.array(args.push_dir), args.arm, confirm=args.confirm,
                             replan=not args.no_replan)
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
