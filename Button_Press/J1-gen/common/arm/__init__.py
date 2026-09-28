"""腕の指令部分。設定ファイル（configs/arm.yaml）からバックエンドを作る関数もここに置く。"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..robot_model import JOINT_NAMES, load_model
from .backend import ArmBackend
from .commander import ArmCommander, NoMotionError, StateTimeoutError, StopRequested
from .safety import UnsafeTargetError, WorkspaceBox

__all__ = [
    "ArmBackend",
    "ArmCommander",
    "NoMotionError",
    "StateTimeoutError",
    "StopRequested",
    "UnsafeTargetError",
    "WorkspaceBox",
    "joint_limits",
    "make_backend",
]


def joint_limits(robot_cfg: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """29関節の可動範囲（motor 番号順）を公式モデルから読む。"""
    m = load_model(robot_cfg, fixed_base=True)
    rng = np.array([m.joint(n).range for n in JOINT_NAMES])
    return rng[:, 0].copy(), rng[:, 1].copy()


def make_backend(
    arm_cfg: dict[str, Any],
    robot_cfg: dict[str, Any],
    dry_run: bool,
    path: str | None = None,
) -> ArmBackend:
    """設定の path（sim / arm_sdk / lowcmd）に応じてバックエンドを作る。"""
    path = path or arm_cfg["path"]
    if path == "sim":
        from .backend_sim import SimBackend

        sim_cfg = arm_cfg["sim"]
        emulate = sim_cfg.get("emulate", "arm_sdk")
        return SimBackend(
            robot_cfg,
            sim_cfg,
            hold_kp=np.asarray(arm_cfg["lowcmd"]["hold_kp"], dtype=float),
            hold_kd=np.asarray(arm_cfg["lowcmd"]["hold_kd"], dtype=float),
            uses_weight=emulate == "arm_sdk",
            dry_run=dry_run,
        )
    if path in ("arm_sdk", "lowcmd"):
        from .backend_dds import DdsBackend

        return DdsBackend(
            path,
            network_interface=arm_cfg["network_interface"],
            domain_id=int(arm_cfg.get("domain_id", 0)),
            dry_run=dry_run,
        )
    raise ValueError(f"path は sim / arm_sdk / lowcmd のどれか: {path}")
