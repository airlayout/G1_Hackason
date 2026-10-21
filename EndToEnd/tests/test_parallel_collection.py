"""並列収集の取得時計・環境分離・保存失敗を、Isaacを起動せず検証する。"""

from dataclasses import replace
from types import SimpleNamespace

import h5py
import numpy as np
import pytest

from g1_e2e.contracts import JointCommand, VelocityCommand
from g1_e2e.data.config import CameraConfig, SensorConfig
from g1_e2e.data.dataset import DatasetReader
from sim.batch.collection import ParallelCollector
from sim.batch.recording import BatchRecorder
from sim.batch.schedule import CaptureSchedule
from sim.envs.config import FlatConfig
from tests.test_batch_recording import calibration
from tests.test_data_collection import observation


class TestEnvironment:
    __test__ = False

    def __init__(self):
        self.config = FlatConfig(physics_dt=1 / 600)
        self.num_envs, self.ticks = 2, 0
        self.steps = np.zeros(2, np.int64)

    def observe(self):
        return [
            replace(
                observation(int(step)),
                privileged={
                    "base_position_world_m": np.array([i + step * 0.01, 0, 0.8]),
                    "base_linear_velocity_body_m_s": np.array([0.5, 0, 0], np.float32),
                },
            )
            for i, step in enumerate(self.steps)
        ]

    def set_positions(self, positions):
        return np.clip(positions, -1, 1)

    def physics_step(self):
        self.ticks += 1


class TestSensors:
    __test__ = False

    def __init__(self, env, schedule):
        self.env = env
        self.camera_stride, self.scan_ticks = schedule.camera_ticks, schedule.scan_ticks
        self.names = ("pelvis", "torso", "lidar")
        self.imu_values = np.zeros((self.scan_ticks + 1, 2, 3, 45), np.float32)
        self.pose_values = np.zeros((self.scan_ticks + 1, 2, 2, 7), np.float32)
        self.imu = SimpleNamespace(numpy=lambda: self.imu_values.copy())
        self.poses = SimpleNamespace(numpy=lambda: self.pose_values.copy())
        self.pattern = SimpleNamespace(rate=20, sample=lambda indices: np.tile([0, 0, -1], (len(indices), 1)))
        self.capture_inertial(0)

    def capture_inertial(self, tick):
        for i in range(2):
            matrix = np.eye(4)
            matrix[:3, 3] = [i + self.env.ticks / 1200, 0, 0.8]
            value = self.imu_values[tick, i]
            value[:, :4] = [0, 0, 0, 1]
            value[:, 7:10] = [0, 0, 9.81]
            value[:, 10:13] = [0, 0, -1]
            value[:, 13:29] = matrix.reshape(-1)
            value[:, 29:45] = np.eye(4).reshape(-1)
            self.pose_values[tick, i, :, :3] = matrix[:3, 3]
            self.pose_values[tick, i, :, 6] = 1

    def capture_camera(self):
        values = np.stack(
            [
                np.full((8, 8, 3), 1 + self.env.ticks // self.camera_stride + i * 10, np.uint8)
                for i in range(2)
            ]
        )
        return {"rgb": values}, [
            dict(world_from_optical=np.eye(4), base_from_optical=np.eye(4)) for _ in range(2)
        ]

    def capture_scan(self, start_index):
        scan = np.zeros((2, 2, 12), np.float32)
        scan[..., 0:2] = [0.8, 1]
        scan[..., 5] = np.arange(2)[:, None] + (start_index + np.arange(1, 3)) / 40
        scan[..., 7], scan[..., 11] = 0.8, 1
        scan[..., 2] = scan[..., 5]
        return scan

    def roll_scan_buffer(self):
        self.imu_values[0] = self.imu_values[-1]
        self.pose_values[0] = self.pose_values[-1]


class TestController:
    __test__ = False

    def set_velocity(self, command):
        self.velocity_command = VelocityCommand(command.copy(), command.copy())

    def act(self, current):
        return JointCommand(np.full(29, 9.0, np.float32))


def recorder(output, schedule):
    return BatchRecorder(
        output,
        {"control_dt_s": 0.02, "primary_imu": "pelvis"},
        calibration(),
        2,
        schedule=schedule,
        imu_names=("pelvis", "torso", "lidar"),
        primary_imu="pelvis",
        point_rate=20,
    )


def test_parallel_clock_records_every_frame_and_only_causal_references(tmp_path):
    env = TestEnvironment()
    sensors = SensorConfig(camera=CameraConfig(period_s=1 / 30))
    schedule = CaptureSchedule.from_configs(env.config, sensors)
    rig = TestSensors(env, schedule)
    teachers = [SimpleNamespace(command=lambda current: np.array([0.2, 0, 0], np.float32)) for _ in range(2)]
    output = recorder(tmp_path, schedule)
    try:
        collector = ParallelCollector(
            env, rig, output, teachers, [TestController() for _ in range(2)], schedule
        )
        collector.run(env.observe(), 10)
        output.close(complete=True, accepted=[True, True])
    finally:
        output.close()
    assert not collector.fallen.any()
    assert schedule.frame_counts(10) == {"camera": 6, "lidar": 2, "imu_each": 121}
    for i in range(2):
        with DatasetReader(tmp_path / f"env_{i:03d}" / "dataset.h5") as reader:
            assert reader.file.attrs["complete"]
            np.testing.assert_allclose(reader.file["sensors/camera/time_s"][:], np.arange(6) / 30)
            np.testing.assert_allclose(reader.file["sensors/lidar/time_s"][:], [0.1, 0.2])
            for name in rig.names:
                np.testing.assert_allclose(reader.file[f"sensors/imu_{name}/time_s"][:], np.arange(121) / 600)
            for index in range(10):
                sample = reader.sample(index)
                tick = index * 12
                assert int(sample["camera"]["rgb"][0, 0, 0]) == 1 + tick // 20 + i * 10
                assert (sample["lidar"] is None) == (index < 5)
                for frame in sample["imus"].values():
                    assert frame["time_s"] == pytest.approx(index * 0.02)
                    assert frame["world_from_sensor"][0, 3] == pytest.approx(i + index * 0.01)
                np.testing.assert_allclose(sample["requested_action"]["position_rad"], 9)
                np.testing.assert_allclose(sample["applied_action"]["position_rad"], 1)


def test_writer_failure_never_marks_a_partial_collection_complete(tmp_path):
    output = recorder(tmp_path, CaptureSchedule(1 / 600, 12, 20, 60))

    def fail():
        raise OSError("disk failure")

    output.submit(fail)
    with pytest.raises(OSError, match="disk failure"):
        output.close(complete=True, accepted=[True, True])
    output.close()  # 異常終了後のcleanupも冪等。
    for writer in output.writers:
        assert not writer.file.id.valid
    for path in tmp_path.glob("env_*/dataset.h5"):
        with h5py.File(path) as file:
            assert not file.attrs["complete"]


def test_partial_writer_initialization_closes_existing_files(tmp_path, monkeypatch):
    import sim.batch.recording as recording

    created = []
    original = recording.DatasetWriter

    def create(*args, **kwargs):
        if created:
            raise OSError("cannot open next file")
        writer = original(*args, **kwargs)
        created.append(writer)
        return writer

    monkeypatch.setattr(recording, "DatasetWriter", create)
    with pytest.raises(OSError, match="cannot open next file"):
        recorder(tmp_path, CaptureSchedule(1 / 600, 12, 20, 60))
    assert not created[0].file.id.valid


@pytest.mark.parametrize("steps", [0, 1, 4, 6])
def test_partial_lidar_blocks_are_rejected_before_collection(steps):
    with pytest.raises(ValueError, match="5の倍数"):
        CaptureSchedule(1 / 600, 12, 20, 60).validate_steps(steps)
