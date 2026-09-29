"""押下動作で共通に使う関節経路と入力検証。SDK や MuJoCo には依存しない。"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterator


ARM_INDICES = tuple(range(22, 29))  # G1 29DoF の右腕。脚・腰は含めない。
ALL_ARM_INDICES = tuple(range(15, 29))
ARM_JOINTS = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
PHASES = ("approach", "contact", "press", "retract", "home")
DEFAULT_DURATIONS = {
    "approach": 4.0,
    "contact": 2.0,
    "press": 1.5,
    "retract": 2.0,
    "home": 4.0,
}
# 29DoF モデルの関節範囲。実機で異なる場合は送信せず構成を確認する。
JOINT_LIMITS = (
    (-3.0892, 2.6704),
    (-2.2515, 1.5882),
    (-2.618, 2.618),
    (-1.0472, 2.0944),
    (-1.97222, 1.97222),
    (-1.61443, 1.61443),
    (-1.61443, 1.61443),
)


def validate_arm_q(values: object) -> tuple[float, ...]:
    """腕の姿勢が有限で関節範囲内か確認する。"""
    if not isinstance(values, (list, tuple)) or len(values) != 7:
        raise ValueError("右腕の関節角は7個必要です")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError("関節角は数値で指定してください") from error
    for name, q, (lower, upper) in zip(ARM_JOINTS, result, JOINT_LIMITS):
        if not math.isfinite(q) or not lower + 0.02 <= q <= upper - 0.02:
            raise ValueError(f"{name} が関節範囲外です: {q}")
    return result


def load_plan(path: str | Path) -> dict:
    """シミュレーションから出力した押下経路を検証して読む。"""
    with Path(path).open(encoding="utf-8") as stream:
        plan = json.load(stream)
    if plan.get("schema") != "g1-push-button-plan-v1":
        raise ValueError("対応していない経路ファイルです")
    if plan.get("arm_joint_names") != list(ARM_JOINTS):
        raise ValueError("右腕の関節名または並びが異なります")
    if not plan.get("sim_result", {}).get("success", False):
        raise ValueError("MuJoCo で押下が確認されていない経路です")
    poses = plan.get("poses")
    durations = plan.get("durations")
    if not isinstance(poses, dict) or not isinstance(durations, dict):
        raise ValueError("経路の形式が不正です")
    for phase in PHASES:
        validate_arm_q(poses.get(phase))
        try:
            duration = float(durations.get(phase, 0))
        except (TypeError, ValueError) as error:
            raise ValueError(f"{phase} の所要時間が不正です") from error
        if not math.isfinite(duration) or not 0.2 <= duration <= 20:
            raise ValueError(f"{phase} の所要時間が不正です")
    return plan


def interpolate(start: tuple[float, ...], end: tuple[float, ...], duration: float,
                hz: float = 50.0) -> Iterator[tuple[float, ...]]:
    """端点で速度がゼロになる補間。最初の点を含み、最後の点にも到達する。"""
    if duration <= 0 or hz <= 0 or len(start) != len(end):
        raise ValueError("補間条件が不正です")
    steps = max(1, math.ceil(duration * hz))
    for index in range(steps + 1):
        t = index / steps
        blend = t * t * (3.0 - 2.0 * t)
        yield tuple(a + blend * (b - a) for a, b in zip(start, end))
