"""Isaac・PyTorchを読み込まない、両PCで共通の設定。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = ROOT / "configs/vla/groot.json"


def finite_array(value, shape: tuple, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name}: 有限値の{shape}が必要です")
    return array


def load_config(path: str | Path = DEFAULT_CONFIG) -> dict:
    config = json.loads(Path(path).read_text())
    arm = config["arm"]
    for key in ["lower_rad", "upper_rad", "kp", "kd"]:
        finite_array(arm[key], (14,), key)
    if np.any(np.asarray(arm["lower_rad"]) >= arm["upper_rad"]):
        raise ValueError("腕の関節下限と上限が逆です")
    for key in [
        "control_hz",
        "state_timeout_s",
        "camera_timeout_s",
        "command_timeout_s",
        "prediction_timeout_s",
        "ramp_s",
        "max_velocity_rad_s",
        "max_excursion_rad",
        "max_tracking_error_rad",
    ]:
        if not np.isfinite(arm[key]) or arm[key] <= 0:
            raise ValueError(f"{key}は有限の正数です")
    for key in ["action_hz", "execute_steps", "max_task_s", "video_history_s"]:
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"{key}は有限の正数です")
    if type(config["execute_steps"]) is not int or config["execute_steps"] > 40:
        raise ValueError("execute_stepsは1..40の整数です")
    if np.any(np.asarray(arm["kp"]) < 0) or np.any(np.asarray(arm["kd"]) < 0):
        raise ValueError("PDゲインは非負です")
    finite_array(config["fixed_hand_state"], (2, 7), "fixed_hand_state")
    return config
