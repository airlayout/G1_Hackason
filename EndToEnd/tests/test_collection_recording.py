"""制御中の非同期frame、episode境界、動画の独立した時刻を検証する。"""

from collections import deque
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from g1_e2e.data.config import LidarConfig
from g1_e2e.data.dataset import DatasetReader
from g1_e2e.data.observer import ObserverWriter, synchronized_frames
from sim.collection import CollectionRecorder
from sim.sensors.lidar import GridLidar
from tests.test_data_collection import frames, observation, transition, writer
from tests.test_imu import measurement


class FakeRig:
    def __init__(self, env):
        self.env = env
        self.pending_lidar = deque()

    def reset(self, current):
        self.pending_lidar.clear()
        self.after_physics_step(current.episode_id, current.simulation_time_s)

    def after_physics_step(self, episode_id, time_s):
        self.latest_imus = {
            "pelvis": measurement(episode_id=episode_id, time_s=time_s, acceleration_valid=time_s > 0)
        }
        self.pending_lidar.append(frames(episode_id, time_s)[1])

    def capture_lidar(self, episode_id, time_s):
        return self.pending_lidar.popleft() if self.pending_lidar else None

    def camera_due(self, time_s):
        return round(time_s / 0.01) % 3 == 0

    def capture_camera(self, current):
        return frames(current.episode_id, current.simulation_time_s)[0]

    def specific_force(self, current):
        frame = self.latest_imus["pelvis"]
        assert frame.time_s == pytest.approx(current.simulation_time_s)
        return frame.specific_force_sensor_m_s2


def collect_episode(recorder, env, episode_id, remaining_steps):
    current = observation(0, episode_id)
    env.current = current
    recorder.start_episode(current, remaining_steps)
    for step in range(2):
        recorder.prepare(current)
        for offset in [0.01, 0.02]:
            time_s = step * 0.02 + offset
            env.current = replace(current, simulation_time_s=time_s)
            recorder.after_physics_step(episode_id, time_s)
        result = transition(step, episode_id)
        recorder.append_step(current, result)
        current = env.current = result.observation


def test_capture_during_step_never_changes_prepared_control_references(tmp_path):
    path = tmp_path / "capture.h5"
    env = SimpleNamespace(config=SimpleNamespace(control_dt=0.02))
    env.observe = lambda: env.current
    with writer(path, primary_imu="pelvis") as output:
        recorder = CollectionRecorder(env, FakeRig(env), output)
        collect_episode(recorder, env, 0, 2)
        assert recorder.summary["camera_frames"] == 2
        assert recorder.summary["lidar_frames"] == 5
        assert recorder.summary["imu_frames"] == {"pelvis": 5}
    with DatasetReader(path) as source:
        assert source.sample(0)["lidar"]["time_s"] == 0
        assert source.sample(1)["lidar"]["time_s"] == pytest.approx(0.02)
        assert source.sample(1)["imu"]["time_s"] == pytest.approx(0.02)
        assert source.sample(1)["camera"]["time_s"] == 0
        # 最終stepで完成したframeも生データとして保存される。
        assert source.latest_sensor_frame("lidar", 0, 0.04)["time_s"] == pytest.approx(0.04)
        assert source.latest_sensor_frame("camera", 0, 0.04)["time_s"] == pytest.approx(0.03)


def test_reset_preserves_completed_frames_and_clears_previous_actions(tmp_path):
    path = tmp_path / "reset.h5"
    env = SimpleNamespace(config=SimpleNamespace(control_dt=0.02))
    env.observe = lambda: env.current
    with writer(path, primary_imu="pelvis") as output:
        recorder = CollectionRecorder(env, FakeRig(env), output)
        collect_episode(recorder, env, 0, 4)
        collect_episode(recorder, env, 1, 2)
        assert recorder.summary["camera_frames"] == 4
        assert recorder.summary["lidar_frames"] == 10
        assert recorder.summary["imu_frames"] == {"pelvis": 10}
    with DatasetReader(path) as source:
        first = source.sample(2)
        assert first["state"]["episode_id"] == first["lidar"]["episode_id"] == first["imu"]["episode_id"] == 1
        assert first["lidar"]["time_s"] == 0
        assert not first["previous_action"]["valid"]
        assert source.latest_sensor_frame("lidar", 2, 1) is None


def test_grid_lidar_keeps_multiple_scans_between_control_ticks():
    class Raycaster:
        def cast(self, origins, directions, *, normals):
            return np.ones(len(origins), np.float32), np.ones(len(origins), bool), -directions

    env = SimpleNamespace(config=SimpleNamespace(physics_dt=0.01))
    config = LidarConfig(period_s=0.01, channels=1, columns=2)
    lidar = GridLidar(
        env, config, Raycaster(), lambda: np.eye(4, dtype=np.float32), lambda: np.eye(4, dtype=np.float32)
    )
    lidar.reset(0)
    assert lidar.pop(0, 0).time_s == 0
    lidar.advance(0, 0.01)
    lidar.advance(0, 0.02)
    assert lidar.pop(0, 0) is None
    assert [lidar.pop(0, 0.02).time_s, lidar.pop(0, 0.02).time_s] == [0.01, 0.02]
    assert lidar.pop(0, 0.02) is None
    lidar.reset(1)
    with pytest.raises(RuntimeError, match="別episode"):
        lidar.pop(0, 0.02)
    assert lidar.pop(1, 0).episode_id == 1


def test_frame_schema_cannot_silently_gain_fabricated_earlier_values(tmp_path):
    with writer(tmp_path / "schema.h5") as output:
        _, lidar = frames()
        output.append_lidar(lidar)
        with pytest.raises(ValueError, match="フィールド"):
            output.append_lidar(replace(lidar, time_s=0.1, ring_valid=np.zeros(3, bool)))
        assert output.frame_counts["lidar"] == 1


def test_visualization_uses_camera_time_without_a_control_row_at_that_time(tmp_path):
    with writer(tmp_path / "dataset.h5") as output:
        first, lidar = frames()
        ci = output.append_camera(first)
        li = output.append_lidar(lidar)
        output.append_step(observation(), transition(), camera_index=ci, lidar_index=li)
        later = replace(first, time_s=0.03, world_from_optical=np.diag([1, 1, 1, 1]).astype(np.float32))
        later_index = output.append_camera(later)
    external = ObserverWriter(tmp_path / "observer.h5", (2, 2))
    external.append(
        replace(observation(), simulation_time_s=0.03),
        1,
        np.zeros((2, 2, 3), np.uint8),
        camera_index=later_index,
    )
    external.close(complete=True)
    capture = list(synchronized_frames(tmp_path))[0]
    assert capture.time_s == pytest.approx(0.03)
    assert capture.lidar["time_s"] == 0
    with DatasetReader(tmp_path / "dataset.h5") as source:
        assert source.sample(0)["state"]["time_s"] == 0
