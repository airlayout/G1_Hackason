"""欠測・同期・座標変換・episode境界を、Isaacなしで検証する。"""

from dataclasses import replace
import importlib
from pathlib import Path
import sys

import h5py
import numpy as np
import pytest
import torch

from g1_e2e.contracts import JointCommand, Observation, StepResult
from g1_e2e.data.config import CameraConfig, SensorConfig
from g1_e2e.data.dataset import CameraFrame, DatasetReader, DatasetWriter, LidarFrame
from g1_e2e.data.geometry import MOUNT_FROM_OPTICAL, lidar_pattern, pose_matrix, transform_points
from g1_e2e.models import ModelConfig, ModelInputAdapter, QueryRequest, RecurrentPerceiver

ROOT = Path(__file__).resolve().parents[1]


def observation(step=0, episode=0):
    return Observation(
        episode_id=episode,
        step=step,
        simulation_time_s=step * 0.02,
        joint_position_rad=np.full(29, step * 0.01, np.float32),
        joint_velocity_rad_s=np.full(29, 0.5, np.float32),
        joint_torque_nm=np.arange(29, dtype=np.float32),
        orientation_xyzw=np.array([0, 0, 0, 1], np.float32),
        angular_velocity_body_rad_s=np.array([0, 0, 0.1], np.float32),
        projected_gravity_body=np.array([0, 0, -1], np.float32),
        privileged={
            "base_position_world_m": np.array([step * 0.01, 0, 0.8], np.float32),
            "base_linear_velocity_body_m_s": np.array([0.5, 0, 0], np.float32),
        },
    )


def transition(step=0, episode=0):
    requested = JointCommand(np.full(29, 9.0, np.float32))
    applied = JointCommand(np.full(29, 0.25, np.float32))
    return StepResult(observation(step + 1, episode), requested, applied, False, False, {})


def frames(episode=0, time_s=0.0):
    transform = np.eye(4, dtype=np.float32)
    rgb = np.zeros((128, 128, 3), np.uint8)
    rgb[..., 0] = 255
    camera = CameraFrame(
        episode,
        time_s,
        rgb,
        np.full((128, 128), 2.0, np.float32),
        np.ones((128, 128), bool),
        transform.copy(),
        transform.copy(),
    )
    lidar = LidarFrame(
        episode,
        time_s,
        np.array([[1, 0, 0], [2, 0, 0], [0, 0, 0]], np.float32),
        np.array([1, 2, 0], np.float32),
        np.array([True, True, False]),
        np.array([1, 0.5, 0], np.float32),
        transform.copy(),
        transform.copy(),
    )
    return camera, lidar


def writer(path, **metadata):
    return DatasetWriter(
        path,
        {"control_dt_s": 0.02, **metadata},
        {
            "camera/intrinsics": CameraConfig().intrinsics,
            "lidar/ray_directions_sensor": np.tile([1, 0, 0], (3, 1)).astype(np.float32),
            "lidar/ring": np.arange(3, dtype=np.int32),
            "lidar/point_time_offset_s": np.zeros(3, np.float32),
        },
    )


@pytest.fixture
def dataset(tmp_path):
    path = tmp_path / "dataset.h5"
    with writer(path) as output:
        camera, lidar = frames()
        ci, li = output.append_camera(camera), output.append_lidar(lidar)
        output.append_step(observation(), transition(), camera_index=ci, lidar_index=li)
        output.append_step(
            observation(1),
            transition(1),
            camera_index=ci,
            lidar_index=li,
            previous_command=transition().applied_command,
        )
    return path


def model_config():
    return ModelConfig.load(ROOT / "configs/model/recurrent_perceiver.yaml")


def test_collection_import_does_not_launch_isaac():
    before = set(sys.modules)
    importlib.import_module("sim.collect")
    importlib.import_module("sim.sensors.rig")
    assert not any(name.startswith(("isaaclab", "isaacsim", "omni.")) for name in set(sys.modules) - before)


def test_model_shapes_and_real_forward(dataset):
    torch.set_num_threads(1)
    config = model_config()
    with DatasetReader(dataset) as source:
        inputs = ModelInputAdapter(config)(source.sample(0))
    expected = {
        "rgb": (1, 64, 768),
        "depth": (1, 64, 512),
        "lidar": (1, 128, 6),
        "joints": (1, 29, 3),
        "imu": (1, 1, 10),
        "previous_action": (1, 29, 1),
    }
    assert {key: tuple(value.features.shape) for key, value in inputs.items()} == expected
    assert inputs["rgb"].features.max() == 1
    assert inputs["lidar"].valid.sum() == 2
    assert not inputs["previous_action"].valid.any()
    with torch.inference_mode():
        model = RecurrentPerceiver(config).eval()
        result = model(inputs, queries={"velocity": QueryRequest()})
    assert result.predictions["velocity"].shape == (1, 1, 3)
    assert torch.isfinite(result.predictions["velocity"]).all()


def test_previous_action_is_applied_previous_not_requested_or_current(dataset):
    with DatasetReader(dataset) as source:
        sample = source.sample(1)
        inputs = ModelInputAdapter(model_config())(sample)
    np.testing.assert_allclose(inputs["previous_action"].features.numpy(), 0.25)
    assert inputs["previous_action"].valid.all()
    assert inputs["previous_action"].age_s[0, 0].item() == pytest.approx(0.02)
    assert sample["requested_action"]["position_rad"][0] == 9


def test_next_state_and_actions_never_affect_inputs(dataset):
    adapter = ModelInputAdapter(model_config())
    with DatasetReader(dataset) as source:
        sample = source.sample(1)
        expected = adapter(sample)
        sample["next"] = {"poison": float("nan")}
        sample["requested_action"] = sample["applied_action"] = {"poison": float("nan")}
        actual = adapter(sample)
    for key in actual:
        torch.testing.assert_close(actual[key].features, expected[key].features)


def test_lidar_ego_motion_compensates_to_current_base(dataset):
    with DatasetReader(dataset) as source:
        sample = source.sample(1)
    sample["state"]["base_pose_world"] = pose_matrix(np.array([0.5, 0, 0]), np.array([0, 0, 0, 1]))
    inputs = ModelInputAdapter(model_config())(sample)
    np.testing.assert_allclose(inputs["lidar"].features[0, :2, :3], [[0.5, 0, 0], [1.5, 0, 0]])
    np.testing.assert_allclose(inputs["lidar"].age_s[0, :2], 0.02)


@pytest.mark.parametrize("sensor", ["camera", "lidar"])
def test_adapter_rejects_future_or_other_episode(dataset, sensor):
    with DatasetReader(dataset) as source:
        sample = source.sample(0)
    sample[sensor]["time_s"] = 0.001
    with pytest.raises(ValueError, match="未来"):
        ModelInputAdapter(model_config())(sample)
    sample[sensor]["time_s"], sample[sensor]["episode_id"] = 0, 1
    with pytest.raises(ValueError, match="別episode"):
        ModelInputAdapter(model_config())(sample)


def test_missing_stale_and_invalid_depth_are_masked(dataset):
    adapter = ModelInputAdapter(model_config())
    with DatasetReader(dataset) as source:
        sample = source.sample(0)
    sample["camera"]["depth_valid"][:] = False
    sample["camera"]["depth_m"][:] = np.inf
    inputs = adapter(sample)
    assert not inputs["depth"].valid.any()
    assert torch.isfinite(inputs["depth"].features).all()
    sample["camera"] = sample["lidar"] = None
    inputs = adapter(sample)
    for key in ["rgb", "depth", "lidar"]:
        assert not inputs[key].valid.any()
        assert torch.isfinite(inputs[key].features).all()
    camera, lidar = frames()
    sample["state"]["time_s"] = 0.5
    sample["camera"], sample["lidar"] = camera.__dict__, lidar.__dict__
    for key in ["rgb", "depth", "lidar"]:
        assert not adapter(sample)[key].valid.any()


def test_reader_rechecks_corrupt_future_reference(dataset):
    with h5py.File(dataset, "r+") as data:
        data["sensors/camera/time_s"][0] = 1.0
    with DatasetReader(dataset) as source, pytest.raises(ValueError, match="未来"):
        source.sample(0)


def test_writer_rejects_future_and_episode_leak(tmp_path):
    with writer(tmp_path / "data.h5") as output:
        c, _ = frames(time_s=0.04)
        index = output.append_camera(c)
        with pytest.raises(ValueError, match="未来"):
            output.append_step(observation(), transition(), camera_index=index)
        with pytest.raises(ValueError, match="別episode"):
            output.append_step(observation(0, 1), transition(0, 1), camera_index=index)


def test_episode_reset_has_no_previous_action_or_old_sensor(tmp_path):
    path = tmp_path / "data.h5"
    with writer(path) as output:
        output.append_step(observation(), transition())
        output.append_step(observation(0, 1), transition(0, 1))
    with DatasetReader(path) as source:
        sample = source.sample(1)
    inputs = ModelInputAdapter(model_config())(sample)
    assert not inputs["previous_action"].valid.any()
    assert sample["camera"] is None and sample["lidar"] is None


def test_interrupted_file_keeps_committed_rows_and_refuses_overwrite(tmp_path):
    path = tmp_path / "partial.h5"
    with pytest.raises(RuntimeError):
        with writer(path) as output:
            output.append_step(observation(), transition())
            raise RuntimeError("interruption")
    with DatasetReader(path) as source:
        assert len(source) == 1
        assert not source.file.attrs["complete"]
        assert source.sample(0)["state"]["step"] == 0
    with pytest.raises(FileExistsError):
        writer(path)


def test_failed_append_does_not_expose_partial_control_row(tmp_path):
    path = tmp_path / "partial-row.h5"
    with pytest.raises(ValueError):
        with writer(path) as output:
            output.append_step(observation(), transition())
            invalid = replace(observation(1), joint_position_rad=np.zeros(30, np.float32))
            output.append_step(invalid, transition(1))
    with DatasetReader(path) as source:
        assert len(source) == 1
        assert source.sample(0)["state"]["joint_position_rad"].shape == (29,)
        with pytest.raises(IndexError):
            source.sample(1)


def test_point_timestamps_cannot_leak_future_measurements(dataset):
    with DatasetReader(dataset) as source:
        sample = source.sample(0)
    sample["lidar_calibration"]["point_time_offset_s"][0] = 0.001
    with pytest.raises(ValueError, match="未来時刻"):
        ModelInputAdapter(model_config())(sample)


def test_changed_model_contract_fails_explicitly():
    config = model_config()
    modalities = dict(config.modalities)
    modalities["imu"] = replace(modalities["imu"], feature_dim=11)
    with pytest.raises(ValueError, match="imu"):
        ModelInputAdapter(replace(config, modalities=modalities))


@pytest.mark.parametrize("modality", ["rgb", "depth", "lidar", "joints", "imu", "previous_action"])
def test_modality_subset_does_not_require_unselected_sensor_data(dataset, modality):
    config = model_config()
    with DatasetReader(dataset) as source:
        sample = source.sample(1)
    expected = ModelInputAdapter(config)(sample)[modality]
    minimal = {"state": sample["state"]}
    if modality in {"rgb", "depth"}:
        fields = ["rgb"] if modality == "rgb" else ["depth_m", "depth_valid"]
        minimal["camera"] = {name: sample["camera"][name] for name in ["episode_id", "time_s", *fields]}
    elif modality == "lidar":
        minimal.update(lidar=sample["lidar"], lidar_calibration=sample["lidar_calibration"])
    elif modality == "previous_action":
        minimal = {"previous_action": sample["previous_action"]}
    subset = replace(config, modalities={modality: config.modalities[modality]})
    actual = ModelInputAdapter(subset)(minimal)
    assert list(actual) == [modality]
    for name in ["features", "valid", "positions", "age_s"]:
        torch.testing.assert_close(getattr(actual[modality], name), getattr(expected, name))


def test_angular_scan_and_optical_axes():
    directions, ring = lidar_pattern(3, 4, (-30, 30))
    np.testing.assert_allclose(np.linalg.norm(directions, axis=-1), 1)
    np.testing.assert_array_equal(ring, np.repeat(np.arange(3), 4))
    np.testing.assert_allclose(
        transform_points(np.array([[0, 0, 1], [1, 0, 0]]), MOUNT_FROM_OPTICAL), [[1, 0, 0], [0, -1, 0]]
    )


def test_sensor_periods_must_align_with_control_clock():
    config = SensorConfig()
    assert config.periods_in_steps(0.02) == (2, 5)
    with pytest.raises(ValueError):
        replace(config, camera=replace(config.camera, period_s=0.033)).periods_in_steps(0.02)
