"""Isaacをimportしない29関節の構成と関節名による対応表。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .contracts import FloatArray, vector


@dataclass(frozen=True)
class RobotSpec:
    """身体29関節の設定。35スロット・ハンドは含めない。"""

    joint_names: tuple[str, ...]
    default_position: FloatArray
    stiffness: FloatArray
    damping: FloatArray
    effort_limit: FloatArray
    velocity_limit: FloatArray
    spawn_height: float
    urdf: str

    @classmethod
    def load(cls, path: Path) -> RobotSpec:
        """名前・数値の不整合を起動前に検出する。"""
        data = yaml.safe_load(path.read_text())
        names = tuple(data["joint_names"])
        if len(names) != 29 or len(set(names)) != 29:
            raise ValueError("身体29関節の重複のないjoint_namesが必要です")
        fields = {
            name: vector(data[name], 29, name)
            for name in ["default_position", "stiffness", "damping", "effort_limit", "velocity_limit"]
        }
        if any(np.any(fields[name] < 0) for name in ["stiffness", "damping"]):
            raise ValueError("PDゲインは非負である必要があります")
        if any(np.any(fields[name] <= 0) for name in ["effort_limit", "velocity_limit"]):
            raise ValueError("effort/velocity limitは正数である必要があります")
        return cls(names, **fields, spawn_height=float(data["spawn_height"]), urdf=data["urdf"])

    def simulator_indices(self, simulator_names: list[str]) -> list[int]:
        """SDK順の各要素に対応するシミュレーター側のindexを返す。"""
        if len(simulator_names) != 29 or set(simulator_names) != set(self.joint_names):
            raise ValueError(f"29関節が一致しません: simulator={simulator_names}")
        return [simulator_names.index(name) for name in self.joint_names]
