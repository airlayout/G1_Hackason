"""センサーを取得時に保存し、制御行にはその時点までの参照だけを固定する。"""

from dataclasses import dataclass, replace

from g1_e2e.data.geometry import base_pose


@dataclass
class CaptureQuality:
    rgb_mean: float = 0.0
    rgb_saturated: float = 0.0
    lidar_returns: int = 0

    def add_camera(self, frame):
        self.rgb_mean += float(frame.rgb.mean())
        self.rgb_saturated += float((frame.rgb >= 254).all(axis=-1).mean())

    def summary(self, counts):
        camera, lidar = counts["camera"], counts["lidar"]
        return {
            "camera_frames": camera,
            "lidar_frames": lidar,
            "imu_frames": {name[4:]: count for name, count in counts.items() if name.startswith("imu_")},
            "rgb_mean_u8": self.rgb_mean / camera if camera else None,
            "rgb_saturated_pixel_fraction": self.rgb_saturated / camera if camera else None,
            "lidar_valid_returns_mean": self.lidar_returns / lidar if lidar else None,
        }


class CollectionRecorder:
    def __init__(self, env, rig, writer, *, observer=None, observer_writer=None):
        if (observer is None) != (observer_writer is None):
            raise ValueError("外部カメラと保存先は組で指定してください")
        self.env, self.rig, self.writer = env, rig, writer
        self.observer, self.observer_writer = observer, observer_writer
        self.quality = CaptureQuality()
        self._prepared = None

    def start_episode(self, observation, remaining_steps):
        self._latest_camera = self._latest_lidar = None
        self.rig.reset(observation)
        self._stop_time = observation.simulation_time_s + remaining_steps * self.env.config.control_dt
        self._camera_index = self._lidar_index = -1
        self._imu_indices = {}
        self._previous = self._prepared = None
        self._capture(observation.episode_id, observation.simulation_time_s)

    def after_physics_step(self, episode_id, time_s):
        self.rig.after_physics_step(episode_id, time_s)
        self._capture(episode_id, time_s)

    def _capture(self, episode_id, time_s):
        for name, frame in self.rig.latest_imus.items():
            self._imu_indices[name] = self.writer.append_imu(name, frame)
        while (frame := self.rig.capture_lidar(episode_id, time_s)) is not None:
            self._latest_lidar = frame
            self._lidar_index = self.writer.append_lidar(frame)
            self.quality.lidar_returns += int(frame.valid.sum())
        if time_s < self._stop_time - 1e-9 and self.rig.camera_due(time_s):
            current = replace(self.env.observe(), episode_id=episode_id, simulation_time_s=time_s)
            frame = self.rig.capture_camera(current)
            self._latest_camera = frame
            self._camera_index = self.writer.append_camera(frame)
            self.quality.add_camera(frame)
            if self.observer is not None:
                self.observer_writer.append(
                    current, len(self.writer), self.observer.capture(current), camera_index=self._camera_index
                )

    def prepare(self, observation):
        """step中に新しいセンサーframeが保存されても、現在行の参照は更新しない。"""
        if self._prepared is not None:
            raise RuntimeError("前の制御行がまだ保存されていません")
        self._prepared = (
            (observation.episode_id, observation.step, observation.simulation_time_s),
            dict(
                camera_index=self._camera_index,
                lidar_index=self._lidar_index,
                imu_indices=self._imu_indices.copy(),
                specific_force_body_m_s2=self.rig.specific_force(observation).copy(),
            ),
        )

    def model_sample(self, observation):
        """保存と同じ取得済みframeだけを、オンライン方策へ渡す。actionや次状態は含めない。"""
        camera, lidar = self._latest_camera, self._latest_lidar
        return {
            "state": {
                "episode_id": observation.episode_id,
                "time_s": observation.simulation_time_s,
                "joint_position_rad": observation.joint_position_rad,
                "joint_velocity_rad_s": observation.joint_velocity_rad_s,
                "joint_torque_nm": observation.joint_torque_nm,
                "orientation_xyzw": observation.orientation_xyzw,
                "angular_velocity_body_rad_s": observation.angular_velocity_body_rad_s,
                "projected_gravity_body": observation.projected_gravity_body,
                "base_pose_world": base_pose(observation),
            },
            "camera": None if camera is None else camera.__dict__,
            "lidar": None if lidar is None else lidar.__dict__,
            "imus": {name: frame.__dict__ for name, frame in self.rig.latest_imus.items()},
            "lidar_calibration": {
                key: getattr(lidar, key)
                if lidar is not None and getattr(lidar, key) is not None
                else self.rig.lidar.calibration[key]
                for key in ("ray_directions_sensor", "ring", "point_time_offset_s")
            },
        }

    def append_step(self, observation, result, *, velocity_command=None):
        stamp = (observation.episode_id, observation.step, observation.simulation_time_s)
        if self._prepared is None or self._prepared[0] != stamp:
            raise RuntimeError("制御行の保存前に同じ観測のprepareが必要です")
        self.writer.append_step(
            observation,
            result,
            previous_command=self._previous,
            velocity_command=velocity_command,
            **self._prepared[1],
        )
        self._previous, self._prepared = result.applied_command, None

    @property
    def summary(self):
        return self.quality.summary(self.writer.frame_counts)
