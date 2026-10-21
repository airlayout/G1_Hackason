"""HDF5へ逐次保存する、因果的な観測・行動・次状態のデータセット。"""

from __future__ import annotations

from dataclasses import asdict, fields
import json
from pathlib import Path

import h5py
import numpy as np

from g1_e2e.contracts import JointCommand, Observation, StepResult, VelocityCommand

from .frames import CameraFrame, ImuFrame, LidarFrame
from .geometry import base_pose

SCHEMA_VERSION = 5


def _check_reference(frame_episode, frame_time, episode, time_s):
    if frame_episode != episode or not np.isfinite(frame_time) or frame_time > time_s + 1e-9:
        raise ValueError("別episodeまたは未来のセンサーframeを現在の観測へ結合できません")


class DatasetWriter:
    """保存済みのcontrol行数をcommitとして持つ。異常終了時も完了行を読み出せる。"""

    def __init__(
        self,
        path: Path,
        metadata: dict,
        calibration: dict[str, np.ndarray],
        *,
        flush_interval: int = 1,
        compression: str | None = "lzf",
    ):
        if flush_interval < 1 or compression not in {None, "lzf"}:
            raise ValueError("flush_intervalは正数、compressionはNoneまたはlzfです")
        self.flush_interval, self.compression = flush_interval, compression
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = h5py.File(path, "x")
        self.file.attrs.update(
            schema_version=SCHEMA_VERSION,
            complete=False,
            committed_steps=0,
            committed_camera_frames=0,
            committed_lidar_frames=0,
        )
        self.file.attrs["metadata_json"] = json.dumps(metadata, ensure_ascii=False)
        for key, value in calibration.items():
            self.file.create_dataset(f"calibration/{key}", data=value)
        self._sensor_stamps: dict[str, list[tuple[int, float]]] = {"camera": [], "lidar": []}
        self._frame_fields: dict[str, set[str]] = {}
        self._datasets: dict[str, h5py.Dataset] = {}
        self._dataset_specs = {}
        self._imu_buffers: dict[str, list[dict[str, np.ndarray]]] = {}
        self._control_buffer = []
        self._row_count = 0
        self._previous: tuple[int, int, float] | None = None
        self.file.flush()

    def _dataset(self, path: str, value) -> h5py.Dataset:
        value = np.asarray(value)
        if path not in self._datasets:
            self._datasets[path] = self.file.create_dataset(
                path,
                shape=(0, *value.shape),
                maxshape=(None, *value.shape),
                chunks=(64 if value.nbytes <= 4096 else 1, *value.shape),
                dtype=value.dtype,
                compression=self.compression,
            )
            self._dataset_specs[path] = value.shape, value.dtype
        dataset = self._datasets[path]
        if self._dataset_specs[path] != (value.shape, value.dtype):
            raise ValueError(f"収集中にshape/dtypeが変わりました: {path}")
        return dataset

    def _append(self, path: str, index: int, value) -> None:
        dataset = self._dataset(path, value)
        dataset.resize(index + 1, axis=0)
        dataset[index] = value

    def _flush_imus(self, name: str | None = None) -> None:
        names = self._imu_buffers if name is None else (name,)
        for sensor in names:
            pending = self._imu_buffers[sensor]
            if not pending:
                continue
            end = len(self._sensor_stamps[sensor])
            start = end - len(pending)
            for key, first in pending[0].items():
                dataset = self._dataset(f"sensors/{sensor}/{key}", first)
                dataset.resize(end, axis=0)
                dataset[start:end] = np.stack([values[key] for values in pending])
            self.file.attrs.modify(f"committed_{sensor}_frames", end)
            pending.clear()

    def _append_frame(self, name: str, frame, *, buffered: bool = False) -> int:
        if not np.isfinite(frame.time_s) or frame.time_s < 0:
            raise ValueError("frameの時刻は有限の非負値です")
        stamps = self._sensor_stamps.setdefault(name, [])
        if stamps and frame.episode_id == stamps[-1][0] and frame.time_s <= stamps[-1][1]:
            raise ValueError("同じepisode内のセンサー時刻は単調増加です")
        index = len(stamps)
        values = {
            field.name: getattr(frame, field.name)
            for field in fields(frame)
            if getattr(frame, field.name) is not None
        }
        if name in self._frame_fields and self._frame_fields[name] != values.keys():
            raise ValueError(f"収集中にframeのフィールドが変わりました: {name}")
        self._frame_fields[name] = set(values)
        if buffered:
            for key, value in values.items():
                self._dataset(f"sensors/{name}/{key}", value)
            pending = self._imu_buffers.setdefault(name, [])
            pending.append({key: np.asarray(value).copy() for key, value in values.items()})
        else:
            for key, value in values.items():
                self._append(f"sensors/{name}/{key}", index, value)
        stamps.append((frame.episode_id, frame.time_s))
        if buffered:
            if len(pending) >= 64:
                self._flush_imus(name)
        else:
            self.file.attrs.modify(f"committed_{name}_frames", index + 1)
        return index

    def append_camera(self, frame: CameraFrame) -> int:
        if frame.rgb.dtype != np.uint8 or frame.rgb.ndim != 3 or frame.rgb.shape[-1] != 3:
            raise ValueError("RGBはuint8[H,W,3]です")
        return self._append_frame("camera", frame)

    def append_lidar(self, frame: LidarFrame) -> int:
        n = len(frame.valid)
        if frame.points_sensor_m.shape != (n, 3) or frame.ranges_m.shape != (n,):
            raise ValueError("LiDARの点・range・validのbeam数が一致しません")
        if frame.valid.dtype != np.bool_ or frame.intensity.shape != (n,):
            raise ValueError("LiDARにはbeamごとのvalidとintensityが必要です")
        if not np.isfinite(frame.points_sensor_m[frame.valid]).all():
            raise ValueError("有効なLiDAR反射点は有限値です")
        if frame.point_time_offset_s is not None:
            if frame.point_time_offset_s.shape != (n,) or not np.isfinite(frame.point_time_offset_s).all():
                raise ValueError("点時刻offsetはbeamごとの有限値です")
            if np.any(frame.point_time_offset_s > 1e-9) or np.any(np.diff(frame.point_time_offset_s) < 0):
                raise ValueError("scan終了時刻に対する点時刻は非正・取得順です")
        return self._append_frame("lidar", frame)

    def append_imu(self, name: str, frame: ImuFrame) -> int:
        if not name.isidentifier():
            raise ValueError("IMU名は識別子にしてください")
        for key in [
            "angular_velocity_sensor_rad_s",
            "specific_force_sensor_m_s2",
            "projected_gravity_sensor",
        ]:
            value = getattr(frame, key)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError(f"IMUの{key}には有限の3成分が必要です")
        if frame.orientation_xyzw.shape != (4,) or not np.isclose(
            np.linalg.norm(frame.orientation_xyzw), 1, atol=1e-5
        ):
            raise ValueError("IMU姿勢は単位xyzw四元数です")
        return self._append_frame(f"imu_{name}", frame, buffered=True)

    def append_imu_batch(self, name: str, values: dict[str, np.ndarray]) -> None:
        """取得時刻を保ったIMU列を一括保存する。制御行のcommit前に呼ぶ。"""
        if not name.isidentifier() or set(values) != {field.name for field in fields(ImuFrame)}:
            raise ValueError("IMU batchにはImuFrameの全フィールドが必要です")
        times, episodes = np.asarray(values["time_s"]), np.asarray(values["episode_id"])
        if not len(times) or any(len(value) != len(times) for value in values.values()):
            raise ValueError("IMU batchの行数が一致しません")
        if (
            not np.isfinite(times).all()
            or np.any(times < 0)
            or np.any(np.diff(times) <= 0)
            or np.any(episodes != episodes[0])
        ):
            raise ValueError("IMU batchの時刻は同じepisode内で単調増加です")
        for key in (
            "angular_velocity_sensor_rad_s",
            "specific_force_sensor_m_s2",
            "projected_gravity_sensor",
        ):
            if values[key].shape != (len(times), 3) or not np.isfinite(values[key]).all():
                raise ValueError(f"IMU batchの{key}は有限の3成分です")
        if values["orientation_xyzw"].shape != (len(times), 4) or not np.allclose(
            np.linalg.norm(values["orientation_xyzw"], axis=1), 1, atol=1e-5
        ):
            raise ValueError("IMU batchの姿勢は単位xyzwです")
        sensor = f"imu_{name}"
        self._flush_imus()
        stamps = self._sensor_stamps.setdefault(sensor, [])
        if stamps and stamps[-1][0] == episodes[0] and times[0] <= stamps[-1][1]:
            raise ValueError("IMU batchの時刻が既存のframeと重複します")
        start = len(stamps)
        for key, value in values.items():
            dataset = self._dataset(f"sensors/{sensor}/{key}", value[0])
            dataset.resize(start + len(times), axis=0)
            dataset[start:] = value
        stamps.extend(zip(episodes.tolist(), times.tolist()))
        self.file.attrs.modify(f"committed_{sensor}_frames", len(stamps))

    def _flush_control(self):
        if not self._control_buffer:
            return
        start = self._row_count - len(self._control_buffer)
        for key, first in self._control_buffer[0].items():
            dataset = self._dataset(key, first)
            dataset.resize(self._row_count, axis=0)
            dataset[start:] = np.stack([row[key] for row in self._control_buffer])
        self.file.attrs.modify("committed_steps", self._row_count)
        self.file.flush()
        self._control_buffer.clear()

    def append_step(
        self,
        observation: Observation,
        result: StepResult,
        *,
        camera_index: int = -1,
        lidar_index: int = -1,
        previous_command: JointCommand | None = None,
        specific_force_body_m_s2: np.ndarray | None = None,
        imu_indices: dict[str, int] | None = None,
        velocity_command: VelocityCommand | None = None,
    ) -> None:
        stamp = (observation.episode_id, observation.step, observation.simulation_time_s)
        if not np.isfinite(stamp[2]) or min(stamp) < 0:
            raise ValueError("episode・step・時刻は有限の非負値です")
        if self._previous is not None and stamp <= self._previous:
            raise ValueError("control行のepisode・step・時刻は順序通りに保存してください")
        if self._previous is not None and stamp[0] == self._previous[0]:
            if stamp[1] != self._previous[1] + 1 or stamp[2] <= self._previous[2]:
                raise ValueError("同じepisode内では連続したstepと増加する時刻を保存してください")
        next_obs = result.observation
        if next_obs.episode_id != observation.episode_id or next_obs.step != observation.step + 1:
            raise ValueError("次状態が同じepisodeの次stepではありません")
        if next_obs.simulation_time_s <= observation.simulation_time_s:
            raise ValueError("次状態の時刻が現在より進んでいません")
        for name, index in [("camera", camera_index), ("lidar", lidar_index)]:
            if index >= 0:
                stamps = self._sensor_stamps[name]
                if index >= len(stamps):
                    raise ValueError("未保存のセンサーframeを参照しています")
                _check_reference(*stamps[index], observation.episode_id, observation.simulation_time_s)
        for name, index in (imu_indices or {}).items():
            stamps = self._sensor_stamps.get(f"imu_{name}", [])
            if index < 0 or index >= len(stamps):
                raise ValueError("未保存のIMUを参照しています")
            _check_reference(*stamps[index], observation.episode_id, observation.simulation_time_s)
        if observation.step == 0 and previous_command is not None:
            raise ValueError("episodeの最初には前回適用actionがありません")
        if velocity_command is not None:
            velocity_command = velocity_command.validated()
        row = {
            "velocity_command/requested": np.zeros(3, np.float32)
            if velocity_command is None
            else velocity_command.requested,
            "velocity_command/applied": np.zeros(3, np.float32)
            if velocity_command is None
            else velocity_command.applied,
            "velocity_command/valid": velocity_command is not None,
            **{f"state/imu_{name}_index": index for name, index in (imu_indices or {}).items()},
            "state/episode_id": observation.episode_id,
            "state/step": observation.step,
            "state/time_s": observation.simulation_time_s,
            "state/joint_position_rad": observation.joint_position_rad,
            "state/joint_velocity_rad_s": observation.joint_velocity_rad_s,
            "state/joint_torque_nm": observation.joint_torque_nm,
            "state/orientation_xyzw": observation.orientation_xyzw,
            "state/angular_velocity_body_rad_s": observation.angular_velocity_body_rad_s,
            "state/projected_gravity_body": observation.projected_gravity_body,
            "state/base_pose_world": base_pose(observation),
            "state/base_linear_velocity_body_m_s": observation.privileged["base_linear_velocity_body_m_s"],
            "state/specific_force_body_m_s2": np.zeros(3, np.float32)
            if specific_force_body_m_s2 is None
            else np.asarray(specific_force_body_m_s2, dtype=np.float32),
            "state/specific_force_valid": specific_force_body_m_s2 is not None and observation.step > 0,
            "state/camera_index": camera_index,
            "state/lidar_index": lidar_index,
            "previous_action/position_rad": np.zeros(29, np.float32)
            if previous_command is None
            else previous_command.position_rad,
            "previous_action/valid": previous_command is not None,
            "next/time_s": next_obs.simulation_time_s,
            "next/base_pose_world": base_pose(next_obs),
            "next/base_linear_velocity_body_m_s": next_obs.privileged["base_linear_velocity_body_m_s"],
            "next/joint_position_rad": next_obs.joint_position_rad,
            "next/joint_velocity_rad_s": next_obs.joint_velocity_rad_s,
            "next/terminated": result.terminated,
            "next/truncated": result.truncated,
        }
        for prefix, command in [
            ("requested_action", result.requested_command),
            ("applied_action", result.applied_command),
        ]:
            for key, value in asdict(command).items():
                row[f"{prefix}/{key}"] = value
        # 制御行が参照するIMUは、行のcommitより先に一括保存する。
        self._flush_imus()
        for key, value in row.items():
            self._dataset(key, value)
        self._control_buffer.append({key: np.asarray(value).copy() for key, value in row.items()})
        self._row_count += 1
        self._previous = stamp
        if self._row_count % self.flush_interval == 0:
            self._flush_control()

    def __len__(self):
        return self._row_count

    @property
    def frame_counts(self) -> dict[str, int]:
        return {name: len(stamps) for name, stamps in self._sensor_stamps.items()}

    def close(self, *, complete: bool = False):
        if self.file:
            self._flush_imus()
            self._flush_control()
            self.file.attrs.modify("complete", complete)
            self.file.flush()
            self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close(complete=exc_type is None)


class DatasetReader:
    """commit済みの行だけを読み、sensor参照をepisodeと時刻で再検証する。"""

    def __init__(self, path: Path):
        self.file = h5py.File(path, "r")
        if self.file.attrs.get("schema_version") not in {1, 2, 3, 4, SCHEMA_VERSION}:
            self.file.close()
            raise ValueError("未対応のdataset schemaです")
        self.metadata = json.loads(self.file.attrs["metadata_json"])
        self._sensor_timeline = {}

    def __len__(self):
        return int(self.file.attrs["committed_steps"])

    def _row(self, group: str, index: int):
        return {key: value[index] for key, value in self.file[group].items()}

    def sensor_frame(self, name: str, index: int) -> dict:
        if not 0 <= index < self.file.attrs.get(f"committed_{name}_frames", 0):
            raise ValueError(f"未commitのセンサーframeです: {name}")
        return self._row(f"sensors/{name}", index)

    def latest_sensor_frame(self, name: str, episode_id: int, time_s: float) -> dict | None:
        if name not in self._sensor_timeline:
            count = int(self.file.attrs.get(f"committed_{name}_frames", 0))
            self._sensor_timeline[name] = (
                None
                if not count
                else (
                    self.file[f"sensors/{name}/episode_id"][:count],
                    self.file[f"sensors/{name}/time_s"][:count],
                )
            )
        timeline = self._sensor_timeline[name]
        if timeline is None:
            return None
        episodes, times = timeline
        candidates = np.flatnonzero((episodes == episode_id) & (times <= time_s + 1e-9))
        return self.sensor_frame(name, int(candidates[-1])) if len(candidates) else None

    def sensor_frames_between(self, name: str, episode_id: int, start_s: float, end_s: float):
        """教師生成用の過去frame列。未commit、別episode、指定時刻後のframeは返さない。"""
        self.latest_sensor_frame(name, episode_id, end_s)
        timeline = self._sensor_timeline[name]
        if timeline is None:
            return
        episodes, times = timeline
        for index in np.flatnonzero(
            (episodes == episode_id) & (times >= start_s - 1e-9) & (times <= end_s + 1e-9)
        ):
            yield self.sensor_frame(name, int(index))

    def sample(self, index: int) -> dict:
        if not 0 <= index < len(self):
            raise IndexError(index)
        result = {
            name: self._row(name, index)
            for name in ["state", "previous_action", "requested_action", "applied_action", "next"]
        }
        if "velocity_command" in self.file:
            result["velocity_command"] = self._row("velocity_command", index)
        state = result["state"]
        for name in ["camera", "lidar"]:
            ref = int(state[f"{name}_index"])
            result[name] = None
            if ref < 0:
                continue
            frame = self.sensor_frame(name, ref)
            _check_reference(frame["episode_id"], frame["time_s"], state["episode_id"], state["time_s"])
            result[name] = frame
        if "primary_imu" in self.metadata:
            result["imus"] = {}
            for key, ref in state.items():
                if not key.startswith("imu_") or not key.endswith("_index"):
                    continue
                name = key[4:-6]
                frame = self.sensor_frame(f"imu_{name}", int(ref))
                _check_reference(frame["episode_id"], frame["time_s"], state["episode_id"], state["time_s"])
                result["imus"][name] = frame
            result["imu"] = result["imus"].get(self.metadata["primary_imu"])
        result["lidar_calibration"] = {
            key: result["lidar"][key]
            if result["lidar"] is not None and key in result["lidar"]
            else self.file[f"calibration/lidar/{key}"][:]
            for key in ["ray_directions_sensor", "ring", "point_time_offset_s"]
        }
        return result

    def close(self):
        self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
