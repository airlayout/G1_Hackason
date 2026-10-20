"""Isaacを起動する前に検証できる、モデルに依存しない平地の設定。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class FlatConfig:
    physics_dt: float = 0.005
    control_dt: float = 0.02
    episode_seconds: float = 20.0
    minimum_base_height: float = 0.35
    maximum_tilt_radians: float = 1.0
    friction: float = 1.0

    def __post_init__(self) -> None:
        values = [
            self.physics_dt,
            self.control_dt,
            self.episode_seconds,
            self.minimum_base_height,
            self.maximum_tilt_radians,
            self.friction,
        ]
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError("環境設定には有限の正数が必要です")
        ratio = self.control_dt / self.physics_dt
        if ratio < 1 or not np.isclose(ratio, round(ratio)):
            raise ValueError("control_dtはphysics_dtの整数倍である必要があります")

    @classmethod
    def load(cls, path: Path) -> FlatConfig:
        return cls(**yaml.safe_load(path.read_text()))
