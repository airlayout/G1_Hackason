"""取り付け位置の加速度、センサー軸、因果的な保存、SDK変換を検証する。"""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from g1_e2e.data.dataset import DatasetReader, DatasetWriter
from g1_e2e.data.geometry import mount_matrix
from g1_e2e.data.imu import measure_imu
from g1_e2e.models import ModelConfig, ModelInputAdapter
from tests.test_data_collection import observation, transition


def measurement(**changes):
    values = dict(
        episode_id=0,
        time_s=0.0,
        world_from_body=np.eye(4),
        body_from_sensor=np.eye(4),
        com_position_world_m=np.zeros(3),
        com_acceleration_world_m_s2=np.zeros(3),
        angular_velocity_world_rad_s=np.zeros(3),
        angular_acceleration_world_rad_s2=np.zeros(3),
        gravity_world_m_s2=np.array([0, 0, -9.81]),
        world_from_base=np.eye(4),
    )
    return measure_imu(**(values | changes))


def test_stationary_imu_reports_specific_force_in_sensor_axes():
    frame = measurement(body_from_sensor=mount_matrix((0.2, 0, 0.1), (90, 0, 0)))
    np.testing.assert_allclose(frame.specific_force_sensor_m_s2, [0, 9.81, 0], atol=1e-6)
    np.testing.assert_allclose(frame.projected_gravity_sensor, [0, -1, 0], atol=1e-6)


def test_offset_imu_includes_tangential_centripetal_and_sensor_rotation():
    frame = measurement(
        body_from_sensor=mount_matrix((1, 0, 0), (0, 0, 90)),
        com_acceleration_world_m_s2=np.array([1, 2, 0]),
        angular_velocity_world_rad_s=np.array([0, 0, 2]),
        angular_acceleration_world_rad_s2=np.array([0, 0, 3]),
    )
    # 世界座標では加速度(-3,5,0)、sensor軸では(5,3,0)。
    np.testing.assert_allclose(frame.specific_force_sensor_m_s2, [5, 3, 9.81], atol=1e-6)
    np.testing.assert_allclose(frame.angular_velocity_sensor_rad_s, [0, 0, 2], atol=1e-6)


def test_lever_arm_is_measured_from_com_and_torso_follows_its_parent():
    frame = measurement(
        world_from_body=mount_matrix((2, 0, 1), (0, 0, 90)),
        body_from_sensor=mount_matrix((1, 0, 0), (0, 0, 0)),
        com_position_world_m=np.array([2, 0.5, 1]),
        angular_velocity_world_rad_s=np.array([0, 0, 2]),
    )
    np.testing.assert_allclose(frame.world_from_sensor[:3, 3], [2, 1, 1], atol=1e-6)
    np.testing.assert_allclose(frame.specific_force_sensor_m_s2, [-2, 0, 9.81], atol=1e-6)
    np.testing.assert_allclose(frame.orientation_xyzw, [0, 0, np.sqrt(0.5), np.sqrt(0.5)], atol=1e-6)


def test_imu_roundtrip_selects_primary_sensor_and_rejects_future_or_other_episode(tmp_path):
    path = tmp_path / "imu.h5"
    calibration = {
        f"lidar/{key}": np.empty(shape, dtype)
        for key, shape, dtype in [
            ("ray_directions_sensor", (0, 3), np.float32),
            ("ring", (0,), np.int32),
            ("point_time_offset_s", (0,), np.float64),
        ]
    }
    frame = measurement(time_s=0.01, body_from_sensor=mount_matrix((1, 0, 0), (0, 0, 90)))
    with DatasetWriter(path, {"primary_imu": "torso", "control_dt_s": 0.02}, calibration) as writer:
        index = writer.append_imu("torso", frame)
        with pytest.raises(ValueError, match="未来"):
            writer.append_step(observation(), transition(), imu_indices={"torso": index})
        with pytest.raises(ValueError, match="別episode"):
            writer.append_step(
                observation(1, episode=1), transition(1, episode=1), imu_indices={"torso": index}
            )
        writer.append_step(observation(1), transition(1), imu_indices={"torso": index})
    root = Path(__file__).resolve().parents[1]
    config = ModelConfig.load(root / "configs/model/recurrent_perceiver.yaml")
    config = replace(config, modalities={"imu_torso": config.modalities["imu_torso"]})
    with DatasetReader(path) as reader:
        sample = reader.sample(0)
        tokens = ModelInputAdapter(config)(sample)["imu_torso"]
        np.testing.assert_allclose(tokens.features[0, 0, :4].numpy(), frame.orientation_xyzw)
        assert tokens.age_s[0, 0].item() == pytest.approx(0.01)
        np.testing.assert_allclose(tokens.features[0, 0, 7:10].numpy(), frame.specific_force_sensor_m_s2)
        np.testing.assert_allclose(tokens.features[0, 0, 10:13].numpy(), frame.projected_gravity_sensor)
        np.testing.assert_allclose(sample["imus"]["torso"]["world_from_sensor"], frame.world_from_sensor)


def test_all_imu_components_and_each_sensors_own_timestamp_reach_the_model(tmp_path):
    from tests.test_data_collection import writer

    path = tmp_path / "all_imus.h5"
    frames = {
        name: measurement(
            time_s=0.005 * index,
            angular_velocity_world_rad_s=np.array([index + 1, 2, 3]),
            com_acceleration_world_m_s2=np.array([index, 1, 2]),
            acceleration_valid=name != "lidar",
        )
        for index, name in enumerate(["pelvis", "torso", "lidar"])
    }
    with writer(path, primary_imu="pelvis") as output:
        indices = {name: output.append_imu(name, frame) for name, frame in frames.items()}
        output.append_step(observation(1), transition(1), imu_indices=indices)
    root = Path(__file__).resolve().parents[1]
    config = ModelConfig.load(root / "configs/model/recurrent_perceiver.yaml")
    config = replace(
        config, modalities={name: cfg for name, cfg in config.modalities.items() if name.startswith("imu_")}
    )
    with DatasetReader(path) as reader:
        sample = reader.sample(0)
    inputs = ModelInputAdapter(config)(sample)
    for name, frame in frames.items():
        tokens = inputs[f"imu_{name}"]
        assert tokens.valid.all()
        assert tokens.features.shape == (1, 1, 14)
        np.testing.assert_allclose(tokens.features[0, 0, :4], frame.orientation_xyzw)
        np.testing.assert_allclose(tokens.features[0, 0, 4:7], frame.angular_velocity_sensor_rad_s)
        expected_acceleration = frame.specific_force_sensor_m_s2 if frame.acceleration_valid else np.zeros(3)
        np.testing.assert_allclose(tokens.features[0, 0, 7:10], expected_acceleration)
        np.testing.assert_allclose(tokens.features[0, 0, 10:13], frame.projected_gravity_sensor)
        assert tokens.features[0, 0, 13] == float(frame.acceleration_valid)
        assert tokens.age_s.item() == pytest.approx(0.02 - frame.time_s)
    sample["imus"].pop("torso")
    assert not ModelInputAdapter(config)(sample)["imu_torso"].valid.any()
    sample["imus"]["lidar"]["time_s"] = 0.03
    with pytest.raises(ValueError, match="未来"):
        ModelInputAdapter(config)(sample)


def test_imu_batches_preserve_values_and_flush_before_committing_a_control_row(tmp_path):
    from tests.test_data_collection import writer

    path = tmp_path / "batched-imu.h5"
    with writer(path, primary_imu="pelvis") as output:
        for index in range(81):
            frame = measurement(time_s=index / 1000)
            frame.specific_force_sensor_m_s2[:] = index
            imu_index = output.append_imu("pelvis", frame)
            # 呼び出し元のbufferが再利用されても、取得時の値を保存する。
            frame.specific_force_sensor_m_s2[:] = -999
        output.append_step(observation(5), transition(5), imu_indices={"pelvis": imu_index})
        with DatasetReader(path) as reader:
            assert len(reader) == 1
            np.testing.assert_allclose(reader.sample(0)["imu"]["specific_force_sensor_m_s2"], 80)
            np.testing.assert_allclose(reader.file["sensors/imu_pelvis/time_s"][:], np.arange(81) / 1000)
            np.testing.assert_allclose(
                reader.file["sensors/imu_pelvis/specific_force_sensor_m_s2"][:, 0], np.arange(81)
            )
        # 最後の制御行の後に取得したIMUも、正常なcloseで保存する。
        output.append_imu("pelvis", measurement(time_s=0.101))
    with DatasetReader(path) as reader:
        assert reader.file.attrs["committed_imu_pelvis_frames"] == 82
        assert reader.latest_sensor_frame("imu_pelvis", 0, 0.101)["time_s"] == pytest.approx(0.101)
