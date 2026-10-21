"""機体への取り付けと収集周期。モデルのNN構造とは分離する。"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


@dataclass(frozen=True)
class MountConfig:
    body: str = "torso_link"
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    urdf_joint: str | None = None

    def __post_init__(self):
        if not self.body or len(self.position_m) != 3 or len(self.rpy_deg) != 3:
            raise ValueError("mountにはbody・XYZ・RPYが必要です")
        if not np.isfinite([*self.position_m, *self.rpy_deg]).all():
            raise ValueError("取り付け位置と回転は有限値です")

    @classmethod
    def from_urdf(cls, path: Path, joint_name: str):
        joint = next(
            (joint for joint in ET.parse(path).getroot().findall("joint") if joint.get("name") == joint_name),
            None,
        )
        if joint is None or joint.get("type") != "fixed":
            raise ValueError(f"センサー取り付けにはURDFのfixed jointが必要です: {joint_name}")
        origin, parent = joint.find("origin"), joint.find("parent")
        if parent is None:
            raise ValueError(f"URDF jointのparentがありません: {joint_name}")
        xyz = tuple(map(float, origin.get("xyz", "0 0 0").split())) if origin is not None else (0, 0, 0)
        rpy = tuple(map(float, origin.get("rpy", "0 0 0").split())) if origin is not None else (0, 0, 0)
        return cls(parent.get("link"), xyz, tuple(map(float, np.rad2deg(rpy))), joint_name)


@dataclass(frozen=True)
class PinholeCalibration:
    """仕様書の取得解像度とpixel単位のK。別解像度では全画面resizeとして変換する。"""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    source: str

    def __post_init__(self):
        if (
            min(self.width, self.height, self.fx, self.fy) <= 0
            or not np.isfinite([self.width, self.height, self.fx, self.fy, self.cx, self.cy]).all()
            or not 0 <= self.cx <= self.width
            or not 0 <= self.cy <= self.height
        ):
            raise ValueError("pinhole校正値の解像度・焦点距離・主点が不正です")

    def resized(self, width: int, height: int):
        sx, sy = width / self.width, height / self.height
        return np.array(
            [
                [self.fx * sx, 0, (self.cx + 0.5) * sx - 0.5],
                [0, self.fy * sy, (self.cy + 0.5) * sy - 0.5],
                [0, 0, 1],
            ],
            dtype=np.float32,
        )


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
    pinhole: PinholeCalibration | None = None

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
        if self.pinhole is not None:
            return self.pinhole.resized(self.width, self.height)
        focal = self.width / (2 * np.tan(np.deg2rad(self.horizontal_fov_deg) / 2))
        return np.array(
            [[focal, 0, (self.width - 1) / 2], [0, focal, (self.height - 1) / 2], [0, 0, 1]], dtype=np.float32
        )

    @property
    def fov_deg(self):
        k = self.intrinsics
        return tuple(
            map(
                float,
                np.rad2deg(
                    [
                        np.arctan((k[0, 2] + 0.5) / k[0, 0])
                        + np.arctan((self.width - 0.5 - k[0, 2]) / k[0, 0]),
                        np.arctan((k[1, 2] + 0.5) / k[1, 1])
                        + np.arctan((self.height - 0.5 - k[1, 2]) / k[1, 1]),
                    ]
                ),
            )
        )


@dataclass(frozen=True)
class LidarConfig:
    model: str = "grid"
    period_s: float = 0.1
    channels: int = 32
    columns: int = 360
    vertical_fov_deg: tuple[float, float] = (-7.0, 52.0)
    near_m: float = 0.15
    far_m: float = 30.0
    mount: MountConfig = field(default_factory=MountConfig)
    pattern_path: str = "assets/cache/sensors/mid360.csv"
    repeat_pattern: bool = False

    def __post_init__(self):
        if self.model not in {"grid", "livox_csv"}:
            raise ValueError("LiDAR modelはgrid / livox_csvです")
        if min(self.channels, self.columns) < 1 or not 0 < self.near_m < self.far_m:
            raise ValueError("LiDARのbeam数・距離範囲が不正です")
        lo, hi = self.vertical_fov_deg
        if not -90 <= lo < hi <= 90 or not np.isfinite(self.period_s) or self.period_s <= 0:
            raise ValueError("LiDARの画角・周期が不正です")


@dataclass(frozen=True)
class BodyImuConfig:
    primary: str = "pelvis"
    mounts: dict[str, MountConfig] = field(default_factory=dict)
    mapping_source: str = (
        "https://github.com/unitreerobotics/unitree_mujoco/tree/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d"
    )

    def __post_init__(self):
        if self.mounts and self.primary not in self.mounts:
            raise ValueError("primary IMUはmountsに含めてください")
        if any(not name.isidentifier() for name in self.mounts):
            raise ValueError("IMU名は識別子にしてください")


@dataclass(frozen=True)
class SensorConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    landmarks: bool = True
    body_imus: BodyImuConfig = field(default_factory=BodyImuConfig)

    @classmethod
    def load(cls, path: Path, *, robot_urdf: Path | None = None):
        data = yaml.safe_load(path.read_text())

        def resolve_mount(mount):
            if "urdf_joint" not in mount:
                return MountConfig(**mount)
            if robot_urdf is None:
                raise ValueError("URDF取り付けの解決にはrobot_urdfが必要です")
            if set(mount) != {"urdf_joint"}:
                raise ValueError("urdf_jointと手動の取り付け値は併用できません")
            return MountConfig.from_urdf(robot_urdf, mount["urdf_joint"])

        for name, kind in [("camera", CameraConfig), ("lidar", LidarConfig)]:
            value = dict(data.pop(name, {}))
            if "mount" in value:
                value["mount"] = resolve_mount(value["mount"])
            if name == "camera" and "pinhole" in value:
                if "horizontal_fov_deg" in value:
                    raise ValueError("pinhole校正と手動画角は併用できません")
                value["pinhole"] = PinholeCalibration(**value["pinhole"])
            data[name] = kind(**value)
        imus = dict(data.pop("body_imus", {}))
        imus["mounts"] = {name: resolve_mount(mount) for name, mount in imus.get("mounts", {}).items()}
        data["body_imus"] = BodyImuConfig(**imus)
        return cls(**data)

    def periods_in_steps(self, time_step_s: float) -> tuple[int, int]:
        result = []
        for period in [self.camera.period_s, self.lidar.period_s]:
            ratio = period / time_step_s
            if ratio < 1 or not np.isclose(ratio, round(ratio)):
                raise ValueError("センサー周期は取得基準の時間刻み以上の整数倍にしてください")
            result.append(round(ratio))
        return tuple(result)

    def to_dict(self):
        result = asdict(self)
        if self.camera.pinhole is not None:
            result["camera"].pop("horizontal_fov_deg")
        if self.lidar.model == "livox_csv":
            for key in ("channels", "columns"):
                result["lidar"].pop(key)
        return result
