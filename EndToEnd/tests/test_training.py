"""因果的な教師cache・未知mask・TBPTT・保存重みを小さいモデルで統合検証する。"""

from dataclasses import asdict, replace
import json

import numpy as np
import pytest
import torch
import yaml

from g1_e2e.contracts import VelocityCommand
from g1_e2e.models import ModelConfig, RecurrentPerceiver
from g1_e2e.data.dataset import DatasetReader
from g1_e2e.inference.export import export_checkpoint, sha256
from g1_e2e.inference.memory_policy import MemoryVelocityPolicy
from g1_e2e.training.cache import prepare, TrainingEpisode
from g1_e2e.training.field import RollingField
from g1_e2e.training.runner import Trainer, TrainingConfig
from tests.test_data_collection import frames, model_config, observation, transition, writer


def test_rolling_field_expires_old_evidence_and_never_marks_unknown_free():
    _, lidar = frames()
    field = RollingField(ray_stride=1)
    field.add(lidar.__dict__)
    current = field.sample(np.eye(4), 0, count=60)
    assert current.occupied.any()
    assert current.observed.any() and (current.observed == 0).any()
    expired = field.sample(np.eye(4), 41, count=60)
    assert not expired.observed.any() and not expired.occupied.any()
    with pytest.raises(ValueError, match="未来"):
        field.sample(np.eye(4), -0.1, count=60)


def test_cache_and_three_head_training_save_reloadable_weights(tmp_path):
    torch.set_num_threads(1)
    config = replace(model_config(), width=16, heads=2, layers=1, ff_multiple_of=16)
    config_path = tmp_path / "model.yaml"
    config_path.write_text(yaml.safe_dump(asdict(config)))
    manifest = {"model_config": str(config_path), "splits": {}}
    for name, split in (
        ("train-a", "train"),
        ("train-b", "train"),
        ("validation", "validation"),
        ("test", "test"),
    ):
        source, cache = tmp_path / f"{name}.h5", tmp_path / f"{name}-cache.h5"
        with writer(source, patrol_teacher={"kind": "local"}) as out:
            camera, lidar = frames()
            ci, li = out.append_camera(camera), out.append_lidar(lidar)
            for index in range(6):
                command = VelocityCommand(
                    np.array([0.2, 0, 0], np.float32), np.array([0.2, 0, 0], np.float32)
                )
                out.append_step(
                    observation(index),
                    transition(index),
                    camera_index=ci,
                    lidar_index=li,
                    velocity_command=command,
                )
            out.file.attrs["patrol_training_accepted"] = True
            out.file.attrs["input_integrity_accepted"] = True
        record = prepare(source, cache, config, aux_stride=1, field_queries=12, ray_stride=1)
        manifest["splits"].setdefault(split, []).append(record)
        episode = TrainingEpisode(cache, "cpu")
        assert len(episode) == 6
        assert set(episode.inputs) == set(config.modalities)
        assert not episode.target(5)["pose_valid"].any()
        episode.close()
    output = tmp_path / "training"
    output.mkdir()
    trainer = Trainer(manifest, output, TrainingConfig(unroll_steps=2, accumulation=1), device="cpu")
    before = {name: parameter.detach().clone() for name, parameter in trainer.model.named_parameters()}
    report = trainer.run()
    assert report["complete"] and report["updates"] == 3
    saved = torch.load(output / "best.pt", map_location="cpu", weights_only=False)
    model = RecurrentPerceiver(ModelConfig.load(config_path))
    model.load_state_dict(saved["model"])
    for head in ("velocity", "bev", "trajectory"):
        prefix = f"decoder.heads.{head}."
        assert any(
            not torch.equal(before[name], value)
            for name, value in model.named_parameters()
            if name.startswith(prefix)
        )
    assert json.loads((output / "summary.json").read_text())["complete"]
    policy = MemoryVelocityPolicy(output / "best.pt", "cpu")
    with DatasetReader(source) as reader:
        velocity = policy.command(reader.sample(0))
    assert velocity.shape == (3,) and np.isfinite(velocity).all()
    assert policy.state.tokens.shape[1] == 1
    policy.reset()
    assert policy.state.tokens.shape[1] == 0
    bundle = tmp_path / "inference"
    exported = export_checkpoint(output / "best.pt", bundle, code_commit="a" * 40, code_dirty=False)
    compact = torch.load(bundle / "model.pt", map_location="cpu", weights_only=True)
    assert set(compact) == {"model", "model_config"}
    assert compact["model_config"] == saved["model_config"]
    assert all(torch.equal(value, compact["model"][name]) for name, value in saved["model"].items())
    assert exported["weights"]["sha256"] == sha256(bundle / "model.pt")
    assert (bundle / "model.pt").stat().st_size < (output / "best.pt").stat().st_size
    portable = MemoryVelocityPolicy(bundle / "model.pt", "cpu")
    with DatasetReader(source) as reader:
        np.testing.assert_array_equal(velocity, portable.command(reader.sample(0)))
    assert exported["export_code_commit"] == "a" * 40 and not exported["export_code_dirty"]
    with pytest.raises(FileExistsError):
        export_checkpoint(output / "best.pt", bundle)
    resumed = Trainer(
        manifest,
        output,
        TrainingConfig(epochs=2, unroll_steps=2, accumulation=1),
        device="cpu",
        resume=output / "last.pt",
    ).run()
    assert resumed["updates"] == 6 and len(resumed["epochs"]) == 2
    assert resumed["initial_validation"] == report["initial_validation"]
    with pytest.raises(ValueError, match="batch_size"):
        Trainer(
            manifest,
            output,
            TrainingConfig(epochs=3, batch_size=1, unroll_steps=2, accumulation=1),
            device="cpu",
            resume=output / "last.pt",
        )
