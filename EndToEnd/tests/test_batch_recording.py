"""一括IMUと制御行bufferでも、既存Readerの因果性と中断時の完了行を保つ。"""

from dataclasses import fields, replace

import numpy as np
import pytest

from g1_e2e.data.dataset import DatasetReader, DatasetWriter
from tests.test_data_collection import observation, transition
from tests.test_imu import measurement


def calibration():
    return {
        "lidar/ray_directions_sensor": np.empty((0, 3), np.float32),
        "lidar/ring": np.empty(0, np.int32),
        "lidar/point_time_offset_s": np.empty(0, np.float64),
    }


def test_bulk_imus_and_buffered_control_preserve_causal_references(tmp_path):
    path = tmp_path / "batch.h5"
    frames = [measurement(time_s=i * 0.001) for i in range(41)]
    values = {
        field.name: np.stack([getattr(frame, field.name) for frame in frames]) for field in fields(frames[0])
    }
    with DatasetWriter(
        path,
        {"control_dt_s": 0.02, "primary_imu": "pelvis"},
        calibration(),
        flush_interval=50,
        compression=None,
    ) as writer:
        writer.append_imu_batch("pelvis", values)
        for step in range(2):
            writer.append_step(observation(step), transition(step), imu_indices={"pelvis": step * 20})
        assert len(writer) == 2
        assert writer.file.attrs["committed_steps"] == 0
    with DatasetReader(path) as reader:
        assert len(reader) == 2
        assert reader.sample(0)["imu"]["time_s"] == 0
        assert reader.sample(1)["imu"]["time_s"] == pytest.approx(0.02)
        assert reader.file["sensors/imu_pelvis/time_s"].shape == (41,)


def test_failed_buffered_row_keeps_only_the_valid_prefix(tmp_path):
    path = tmp_path / "failed.h5"
    with pytest.raises(ValueError):
        with DatasetWriter(path, {}, calibration(), flush_interval=50) as writer:
            writer.append_step(observation(), transition())
            invalid = replace(observation(1), joint_position_rad=np.zeros(30, np.float32))
            writer.append_step(invalid, transition(1))
    with DatasetReader(path) as reader:
        assert len(reader) == 1
        assert not reader.file.attrs["complete"]
        assert reader.sample(0)["state"]["step"] == 0
