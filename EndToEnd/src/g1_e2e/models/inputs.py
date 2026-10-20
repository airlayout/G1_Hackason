"""保存した生の観測だけを、RecurrentPerceiverのSensorTokensへ変換する。"""

from __future__ import annotations

import math

import numpy as np
import torch
import torch.nn.functional as F

from g1_e2e.data.geometry import transform_points

from .config import ModelConfig
from .tokens import SensorTokens, image_patches


class ModelInputAdapter:
    """未来のnext状態・今回のaction・希望速度は入力として参照しない。"""

    def __init__(
        self,
        config: ModelConfig,
        *,
        device: str = "cpu",
        control_dt_s: float = 0.02,
        image_size: int = 128,
        patch_size: int = 16,
        camera_max_age_s: float = 0.2,
        lidar_max_age_s: float = 0.3,
    ):
        self.config, self.device = config, torch.device(device)
        self.control_dt_s = control_dt_s
        self.image_size, self.patch_size = image_size, patch_size
        self.camera_max_age_s, self.lidar_max_age_s = camera_max_age_s, lidar_max_age_s
        if image_size < patch_size or image_size % patch_size:
            raise ValueError("入力画像サイズはpatch_sizeの正の整数倍です")
        if not all(math.isfinite(x) and x > 0 for x in [control_dt_s, camera_max_age_s, lidar_max_age_s]):
            raise ValueError("制御周期と許容ageは有限の正数です")
        expected = {
            "rgb": (3 * patch_size**2, (image_size // patch_size) ** 2, 2),
            "depth": (2 * patch_size**2, (image_size // patch_size) ** 2, 2),
            "lidar": (6, 1, 3),
            "joints": (3, 29, 0),
            "imu": (10, 1, 0),
            "previous_action": (1, 29, 0),
        }
        for name, stream in config.modalities.items():
            if name not in expected:
                raise ValueError(f"入力アダプター未対応のmodalityです: {name}")
            features, minimum_tokens, positions = expected[name]
            if (
                stream.feature_dim != features
                or stream.position_dim != positions
                or stream.max_tokens < minimum_tokens
            ):
                raise ValueError(f"モデルと入力アダプターのshapeが一致しません: {name}")

    def _tensor(self, value, dtype=torch.float32):
        return torch.as_tensor(np.asarray(value).copy(), dtype=dtype, device=self.device)

    def _tokens(self, features, *, valid=None, positions=None, age_s=0.0):
        array = np.asarray(features)
        features = self._tensor(array).unsqueeze(0)
        return SensorTokens(
            features,
            valid=self._tensor(
                np.isfinite(array).all(axis=-1) if valid is None else valid, torch.bool
            ).unsqueeze(0),
            positions=None if positions is None else self._tensor(positions).unsqueeze(0),
            age_s=self._tensor(np.broadcast_to(age_s, (features.shape[1],))).unsqueeze(0),
        )

    def _age(self, state, frame):
        age = float(state["time_s"] - frame["time_s"])
        if frame["episode_id"] != state["episode_id"] or age < -1e-9:
            raise ValueError("未来・別episodeのセンサー観測を入力できません")
        return max(age, 0.0)

    def _joints(self, sample: dict) -> SensorTokens:
        state = sample["state"]
        joints = np.stack(
            [state[key] for key in ["joint_position_rad", "joint_velocity_rad_s", "joint_torque_nm"]], axis=-1
        )
        return self._tokens(joints)

    def _imu(self, sample: dict) -> SensorTokens:
        if "imu" in sample:
            frame = sample["imu"]
            if frame is None:
                return self._tokens(np.zeros((1, 10), np.float32), valid=np.zeros(1, bool))
            age = self._age(sample["state"], frame)
            features = np.concatenate(
                [
                    frame[key]
                    for key in [
                        "orientation_xyzw",
                        "angular_velocity_sensor_rad_s",
                        "projected_gravity_sensor",
                    ]
                ]
            )
            return self._tokens(features[None], age_s=age)
        state = sample["state"]
        imu = np.concatenate(
            [
                state[key]
                for key in ["orientation_xyzw", "angular_velocity_body_rad_s", "projected_gravity_body"]
            ]
        )
        return self._tokens(imu[None])

    def _previous_action(self, sample: dict) -> SensorTokens:
        previous = sample["previous_action"]
        return self._tokens(
            previous["position_rad"][:, None],
            valid=np.full(29, bool(previous["valid"])),
            age_s=self.control_dt_s if previous["valid"] else 0.0,
        )

    def _camera(self, sample: dict) -> dict[str, SensorTokens]:
        state, camera = sample["state"], sample["camera"]
        camera_age = self._age(state, camera) if camera is not None else 0.0
        camera_valid = camera is not None and camera_age <= self.camera_max_age_s
        size = (self.image_size, self.image_size)
        observations = {}
        for name, channels in [("rgb", 3), ("depth", 2)]:
            if name not in self.config.modalities:
                continue
            if camera is None:
                image = torch.zeros(1, channels, *size, device=self.device)
            elif name == "rgb":
                image = self._tensor(camera["rgb"]).permute(2, 0, 1).unsqueeze(0) / 255.0
                image = F.interpolate(image, size=size, mode="bilinear", align_corners=False)
            else:
                valid = np.asarray(camera["depth_valid"], dtype=bool)
                depth_m = np.where(valid, camera["depth_m"], 0)
                image = self._tensor(np.stack((depth_m, valid), axis=0)).unsqueeze(0)
                image = F.interpolate(image, size=size, mode="nearest")
            patches = image_patches(image, self.patch_size, age_s=self._tensor([camera_age]))
            valid = torch.full(patches.features.shape[:2], camera_valid, dtype=torch.bool, device=self.device)
            if name == "depth":
                pixel_valid = F.unfold(image[:, 1:2], self.patch_size, stride=self.patch_size)
                valid = valid & pixel_valid.any(dim=1)
            observations[name] = SensorTokens(patches.features, valid, patches.positions, patches.age_s)
        return observations

    def _lidar(self, sample: dict) -> SensorTokens:
        state, lidar = sample["state"], sample["lidar"]
        n = self.config.modalities["lidar"].max_tokens
        features, valid, ages = np.zeros((n, 6), np.float32), np.zeros(n, bool), np.zeros(n, np.float32)
        if lidar is not None:
            age = self._age(state, lidar)
            calibration = sample["lidar_calibration"]
            offsets = np.asarray(calibration["point_time_offset_s"])
            if np.any(offsets > age + 1e-9):
                raise ValueError("未来時刻のLiDAR反射点です")
            candidates = np.flatnonzero(lidar["valid"] & np.isfinite(lidar["points_sensor_m"]).all(axis=-1))
            if len(candidates):
                # Beam順に全体へ分散させる決定的な間引き。保存した生の点群は削らない。
                count = min(n, len(candidates))
                chosen = candidates[np.linspace(0, len(candidates) - 1, count, dtype=np.int64)]
                if lidar.get("points_world_m") is not None:
                    xyz = transform_points(
                        lidar["points_world_m"][chosen], np.linalg.inv(state["base_pose_world"])
                    )
                else:
                    current_base_from_sensor = (
                        np.linalg.inv(state["base_pose_world"]) @ lidar["world_from_sensor"]
                    )
                    xyz = transform_points(lidar["points_sensor_m"][chosen], current_base_from_sensor)
                features[:count, :3] = xyz
                features[:count, 3] = lidar["intensity"][chosen]
                features[:count, 4] = calibration["ring"][chosen]
                for column, key in [(3, "intensity_valid"), (4, "ring_valid")]:
                    if lidar.get(key) is not None:
                        features[:count, column] *= lidar[key][chosen]
                features[:count, 5] = offsets[chosen]
                ages[:count] = age - offsets[chosen]
                valid[:count] = ages[:count] <= self.lidar_max_age_s
        return self._tokens(features, valid=valid, positions=features[:, :3], age_s=ages)

    def __call__(self, sample: dict) -> dict[str, SensorTokens]:
        """設定したmodalityだけ変換し、保存データや他のmodalityへ依存させない。"""
        builders = {
            "joints": self._joints,
            "imu": self._imu,
            "previous_action": self._previous_action,
            "lidar": self._lidar,
        }
        observations = self._camera(sample) if self.config.modalities.keys() & {"rgb", "depth"} else {}
        return {
            name: observations[name] if name in observations else builders[name](sample)
            for name in self.config.modalities
        }
