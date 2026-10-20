"""機体への取り付けと収集周期。モデルのNN構造とは分離する。"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class MountConfig:
    body: str = "torso_link"
    position_m: tuple[float, float, float] = (0.12, 0.0, 0.34)
    rpy_deg: tuple[float, float, float] = (0.0, 15.0, 0.0)

    def __post_init__(self):
        if not self.body or len(self.position_m) != 3 or len(self.rpy_deg) != 3:
            raise ValueError("mountにはbody・XYZ・RPYが必要です")
        if not np.isfinite([*self.position_m, *self.rpy_deg]).all():
            raise ValueError("取り付け位置と回転は有限値です")


@dataclass(frozen=True)
class CameraConfig:
    width: int = 128
    height: int = 128
    period_s: float = 0.04
    horizontal_fov_deg: float = 80.0
    near_m: float = 0.05
    far_m: float = 30.0
    exposure_iso: float = 100.0
    exposure_time_s: float = 0.02
    exposure_f_stop: float = 5.0
    render_updates: int = 3
    mount: MountConfig = field(default_factory=MountConfig)

    def __post_init__(self):
        if min(self.width, self.height, self.render_updates) < 1:
            raise ValueError("画像サイズ・描画回数は正数です")
        if not 0 < self.horizontal_fov_deg < 180 or not 0 < self.near_m < self.far_m:
            raise ValueError("カメラの画角・距離範囲が不正です")
        positive = [self.period_s, self.exposure_iso, self.exposure_time_s, self.exposure_f_stop]
        if not np.isfinite(positive).all() or min(positive) <= 0:
            raise ValueError("カメラの周期・露出は有限の正数です")

    @property
    def intrinsics(self) -> np.ndarray:
        focal = self.width / (2 * np.tan(np.deg2rad(self.horizontal_fov_deg) / 2))
        return np.array(
            [[focal, 0, self.width / 2], [0, focal, self.height / 2], [0, 0, 1]], dtype=np.float32
        )


@dataclass(frozen=True)
class LidarConfig:
    period_s: float = 0.1
    channels: int = 32
    columns: int = 360
    vertical_fov_deg: tuple[float, float] = (-30.0, 60.0)
    near_m: float = 0.15
    far_m: float = 30.0
    mount: MountConfig = field(
        default_factory=lambda: MountConfig(position_m=(0.0, 0.0, 0.50), rpy_deg=(0.0, 0.0, 0.0))
    )

    def __post_init__(self):
        if min(self.channels, self.columns) < 1 or not 0 < self.near_m < self.far_m:
            raise ValueError("LiDARのbeam数・距離範囲が不正です")
        lo, hi = self.vertical_fov_deg
        if not -90 <= lo < hi <= 90 or not np.isfinite(self.period_s) or self.period_s <= 0:
            raise ValueError("LiDARの画角・周期が不正です")


@dataclass(frozen=True)
class SensorConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    landmarks: bool = True

    @classmethod
    def load(cls, path: Path):
        data = yaml.safe_load(path.read_text())
        for name, kind in [("camera", CameraConfig), ("lidar", LidarConfig)]:
            value = dict(data.pop(name, {}))
            if "mount" in value:
                value["mount"] = MountConfig(**value["mount"])
            data[name] = kind(**value)
        return cls(**data)

    def periods_in_steps(self, control_dt: float) -> tuple[int, int]:
        result = []
        for period in [self.camera.period_s, self.lidar.period_s]:
            ratio = period / control_dt
            if ratio < 1 or not np.isclose(ratio, round(ratio)):
                raise ValueError("センサー周期は制御周期以上の整数倍にしてください")
            result.append(round(ratio))
        return tuple(result)

    def to_dict(self):
        return asdict(self)
