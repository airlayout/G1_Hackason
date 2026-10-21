"""速度指令を実速度と区別し、未知・episode境界・未来情報の混入を検証する。"""

from dataclasses import replace

import numpy as np
import pytest

from g1_e2e.contracts import VelocityCommand
from g1_e2e.data.dataset import DatasetReader
from g1_e2e.data.targets import RayEvidence, SupervisionReader
from g1_e2e.models import ModelInputAdapter
from tests.test_data_collection import frames, model_config, observation, transition, writer


def test_recorded_policy_teacher_is_applied_velocity_command_and_never_a_model_input(tmp_path):
    path = tmp_path / "commands.h5"
    command = VelocityCommand(np.array([1.5, 0.1, 0.1], np.float32), np.array([1, 0.1, 0.1], np.float32))
    with writer(path) as output:
        output.append_step(observation(), transition(), velocity_command=command)
    with DatasetReader(path) as reader:
        sample = reader.sample(0)
        np.testing.assert_allclose(sample["velocity_command"]["requested"], command.requested)
        target, valid = SupervisionReader(reader).velocity(0)
        assert valid
        np.testing.assert_allclose(target, command.applied)
        assert not np.allclose(target, sample["state"]["base_linear_velocity_body_m_s"])
        adapter = ModelInputAdapter(model_config())
        expected = adapter(sample)
        sample["velocity_command"] = {"poison": float("nan")}
        actual = adapter(sample)
        for name in actual:
            np.testing.assert_allclose(actual[name].features, expected[name].features)


def test_missing_velocity_teacher_is_masked_instead_of_using_real_velocity(tmp_path):
    path = tmp_path / "missing.h5"
    with writer(path) as output:
        output.append_step(observation(), transition())
    with DatasetReader(path) as reader:
        _, valid = SupervisionReader(reader).velocity(0)
        assert not valid


def test_stationary_or_unsafe_patrol_is_not_accepted_as_a_velocity_teacher(tmp_path):
    from g1_e2e.data.quality import patrol_quality
    from types import SimpleNamespace

    config = SimpleNamespace(cruise_speed_m_s=0.3, robot_radius_m=0.4)
    summary = {
        "falls": 0,
        "simulated_seconds": 30,
        "patrol_episodes": [{"distance_m": 0.06, "minimum_map_clearance_m": 1.3}],
    }
    assert not patrol_quality(summary, config)["accepted"]
    summary["patrol_episodes"][0]["distance_m"] = 8.0
    assert patrol_quality(summary, config)["accepted"]
    summary["patrol_episodes"][0]["minimum_map_clearance_m"] = 0.3
    assert not patrol_quality(summary, config)["accepted"]
    path = tmp_path / "rejected.h5"
    with writer(path) as output:
        command = VelocityCommand(np.ones(3, np.float32), np.ones(3, np.float32))
        output.append_step(observation(), transition(), velocity_command=command)
        output.file.attrs["patrol_training_accepted"] = False
    with DatasetReader(path) as reader:
        assert not SupervisionReader(reader).velocity(0)[1]


def test_future_poses_are_relative_to_anchor_and_masked_after_episode_end(tmp_path):
    path = tmp_path / "trajectory.h5"
    with writer(path) as output:
        for episode in [0, 1]:
            for step in range(3):
                output.append_step(observation(step, episode), transition(step, episode))
    with DatasetReader(path) as reader:
        target = SupervisionReader(reader).trajectory(1, [0, 0.01, 0.04, 0.06, 1.0])
        assert target.valid.tolist() == [True, True, True, False, False]
        np.testing.assert_allclose(target.poses[:3, 0], [0, 0.005, 0.02], atol=1e-7)
        np.testing.assert_allclose(target.poses[:, 3:], np.tile([0, 0, 0, 1], (5, 1)))
        with pytest.raises(ValueError):
            SupervisionReader(reader).trajectory(0, [-0.1])


def test_ray_evidence_leaves_unseen_and_behind_surface_voxels_unknown():
    evidence = RayEvidence(bounds=((0, 4), (0, 1), (0, 1)), voxel_size_m=0.25)
    evidence.add(np.array([[0.1, 0.5, 0.5]]), np.array([[2, 0.5, 0.5]]))
    assert evidence.known[4, 2, 2] and not evidence.occupied[4, 2, 2]
    assert evidence.occupied[8, 2, 2]
    assert not evidence.known[12, 2, 2]
    sampled = evidence.sample(count=90, seed=2)
    assert sampled.coordinates.shape == (90, 3)
    assert sampled.occupied.sum() == 30
    assert sampled.observed.sum() == 60
    assert not sampled.occupancy_valid[sampled.observed == 0].any()


def test_field_targets_only_use_causal_valid_rays_from_the_same_episode(tmp_path):
    path = tmp_path / "field.h5"
    with writer(path) as output:
        _, lidar = frames()
        li = output.append_lidar(lidar)
        output.append_step(observation(), transition(), lidar_index=li)
        future = replace(lidar, time_s=0.1, points_sensor_m=np.full((3, 3), 100, np.float32))
        output.append_lidar(future)
    with DatasetReader(path) as reader:
        target = SupervisionReader(reader).field(0, count=60)
        assert target.occupied.any()
        assert target.observed.any() and (target.observed == 0).any()
        assert len(list(reader.sensor_frames_between("lidar", 0, 0, 0))) == 1
        assert not list(reader.sensor_frames_between("lidar", 1, 0, 1))


def test_failed_input_audit_masks_velocity_teacher(tmp_path):
    path = tmp_path / "bad-inputs.h5"
    from g1_e2e.contracts import VelocityCommand

    with writer(path) as output:
        command = VelocityCommand(np.ones(3, np.float32), np.ones(3, np.float32))
        output.append_step(observation(), transition(), velocity_command=command)
        output.file.attrs["input_integrity_accepted"] = False
    with DatasetReader(path) as reader:
        assert not SupervisionReader(reader).velocity(0)[1]
