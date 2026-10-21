"""GPUから受け取った取得順のbatchを、互換HDF5へ非同期保存する。"""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from dataclasses import dataclass
import os

import numpy as np

from g1_e2e.contracts import Observation, StepResult, VelocityCommand
from g1_e2e.data.dataset import DatasetWriter
from g1_e2e.data.frames import CameraFrame, LidarFrame
from g1_e2e.data.geometry import pose_matrix


@dataclass(frozen=True)
class BatchStep:
    observations: list[Observation]
    results: list[StepResult]
    velocities: list[VelocityCommand]


class BatchRecorder:
    def __init__(
        self,
        output,
        metadata,
        calibration,
        num_envs,
        *,
        schedule,
        imu_names,
        primary_imu,
        point_rate,
        compression=None,
    ):
        self.schedule, self.imu_names, self.point_rate = schedule, tuple(imu_names), point_rate
        self.primary_index = self.imu_names.index(primary_imu)
        self.writers = []
        with ExitStack() as resources:
            for i in range(num_envs):
                writer = DatasetWriter(
                    output / f"env_{i:03d}" / "dataset.h5",
                    {**metadata, "environment_index": i},
                    calibration,
                    flush_interval=50,
                    compression=compression,
                )
                resources.callback(writer.close)
                self.writers.append(writer)
            resources.pop_all()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="dataset-writer")
        self.pending = deque()
        self.previous = [None] * num_envs
        self._failed = self._closed = False

    def submit(self, function, *args):
        # メインloopから取得bufferの所有権を渡す。最大2件まで先行し、保存の遅延を隠さない。
        self.pending.append(self.executor.submit(function, *args))
        if len(self.pending) >= 2:
            self._wait_next()

    def _wait_next(self):
        try:
            self.pending.popleft().result()
        except BaseException:
            self._failed = True
            raise

    def cameras(self, values, extrinsics, time_s):
        self.submit(self._cameras, values, extrinsics, time_s)

    def _cameras(self, values, extrinsics, time_s):
        for i, writer in enumerate(self.writers):
            writer.append_camera(
                CameraFrame(0, time_s, **{key: value[i] for key, value in values.items()}, **extrinsics[i])
            )

    def block(self, imu, poses, scan, directions, start_tick, end_tick, rows):
        self.submit(self._block, imu, poses, scan, directions, start_tick, end_tick, rows)

    def _block(self, imu, poses, scan, directions, start_tick, end_tick, rows):
        dt = self.schedule.physics_dt_s
        first = 0 if start_tick == 0 else 1
        times = np.arange(start_tick + first, end_tick + 1, dtype=np.float64) * dt
        end_time = end_tick * dt
        start_index = round(start_tick * dt * self.point_rate)
        for i, writer in enumerate(self.writers):
            for sensor, name in enumerate(self.imu_names):
                data = imu[first:, i, sensor]
                writer.append_imu_batch(
                    name,
                    {
                        "episode_id": np.zeros(len(times), np.int64),
                        "time_s": times,
                        "orientation_xyzw": data[:, :4],
                        "angular_velocity_sensor_rad_s": data[:, 4:7],
                        "specific_force_sensor_m_s2": data[:, 7:10],
                        "projected_gravity_sensor": data[:, 10:13],
                        "world_from_sensor": data[:, 13:29].reshape(-1, 4, 4),
                        "base_from_sensor": data[:, 29:45].reshape(-1, 4, 4),
                        "acceleration_valid": times > 0,
                    },
                )
            data = scan[i]
            current = pose_matrix(poses[-1, i, 0, :3], poses[-1, i, 0, 3:])
            base = pose_matrix(poses[-1, i, 1, :3], poses[-1, i, 1, 3:])
            n = len(data)
            writer.append_lidar(
                LidarFrame(
                    0,
                    end_time,
                    directions * data[:, :1],
                    data[:, 0],
                    data[:, 1].astype(bool),
                    np.zeros(n, np.float32),
                    current,
                    np.linalg.inv(base) @ current,
                    ray_directions_sensor=directions,
                    point_time_offset_s=(start_index + np.arange(1, n + 1)) / self.point_rate - end_time,
                    ring=np.zeros(n, np.int32),
                    ring_valid=np.zeros(n, bool),
                    intensity_valid=np.zeros(n, bool),
                    points_world_m=data[:, 2:5],
                    measurement_positions_world_m=data[:, 5:8],
                    measurement_orientation_xyzw=data[:, 8:12],
                    scan_start_time_s=start_tick * dt,
                    profile_start_index=start_index,
                )
            )
            for row in rows:
                observation = row.observations[i]
                tick = round(observation.simulation_time_s / dt)
                data = imu[tick - start_tick, i, self.primary_index]
                specific_force = data[29:45].reshape(4, 4)[:3, :3] @ data[7:10]
                result = row.results[i]
                writer.append_step(
                    observation,
                    result,
                    previous_command=self.previous[i],
                    velocity_command=row.velocities[i],
                    camera_index=tick // self.schedule.camera_ticks,
                    lidar_index=tick // self.schedule.scan_ticks - 1,
                    imu_indices={name: tick for name in self.imu_names},
                    specific_force_body_m_s2=specific_force,
                )
                self.previous[i] = result.applied_command

    def drain(self):
        while self.pending:
            self._wait_next()

    def close(self, *, complete=False, accepted=None):
        if self._closed:
            return
        try:
            self.drain()
        finally:
            self.executor.shutdown(wait=True)
            self._closed = True
            # 一環境のcloseに失敗しても全ファイルを閉じる。writer失敗時は完了扱いにしない。
            with ExitStack() as resources:
                for i, writer in enumerate(self.writers):
                    resources.callback(
                        self._close_writer,
                        writer,
                        complete and not self._failed,
                        None if accepted is None else accepted[i],
                    )

    @staticmethod
    def _close_writer(writer, complete, accepted):
        path = writer.file.filename
        try:
            if accepted is not None:
                writer.file.attrs["patrol_training_accepted"] = accepted
        finally:
            writer.close(complete=complete)
        if complete:
            descriptor = os.open(path, os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
