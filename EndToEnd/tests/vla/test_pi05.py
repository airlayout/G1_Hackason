"""公開G1 checkpointの実際のメタデータで、観測・関節順・正規化を検証する。"""

import base64
import io
import json
from pathlib import Path
import shutil
import struct

import numpy as np
from PIL import Image
import pytest

from g1_e2e.vla.config import load_config
from g1_e2e.vla.backends.pi05 import (
    ACTION_NAMES,
    IMAGE_KEY,
    STATE_NAMES,
    Pi05ObservationAdapter,
    Pi05Policy,
    arm_chunk,
    link_cached_file,
    validate_checkpoint,
)
from g1_e2e.vla.policy import create_adapter, create_policy


FIXTURE = Path(__file__).parent / "fixtures/pi05_g1"


def observation(color=(255, 0, 0), size=(640, 480)):
    buffer = io.BytesIO()
    Image.new("RGB", size, color).save(buffer, format="JPEG")
    return {"q": (np.arange(29) / 10).tolist(), "jpeg": base64.b64encode(buffer.getvalue()).decode()}


def test_default_selects_real_g1_finetune_and_not_base_model():
    config = load_config()
    assert config["model_id"] == "xiaopeng-wu/pi05_unitree_g1"
    assert isinstance(create_adapter(config, []), Pi05ObservationAdapter)
    assert isinstance(create_policy(config), Pi05Policy)
    raw = validate_checkpoint(FIXTURE)
    assert raw["pretrained_path"] == "lerobot/pi05_base"
    assert raw["action_feature_names"] == list(ACTION_NAMES)


def test_pi05_observation_uses_current_rgb_and_measured_29_joints_without_fk():
    adapter = Pi05ObservationAdapter(load_config())
    adapter.record(observation((0, 0, 255)), now=1)
    current = observation((255, 0, 0))
    inputs = adapter.build(current, "Move the blue box", now=2)
    assert set(inputs) == {IMAGE_KEY, "observation.state", "task"}
    assert inputs[IMAGE_KEY].shape == (3, 480, 640)
    assert inputs[IMAGE_KEY].dtype == np.float32
    assert inputs[IMAGE_KEY][0].mean() > 0.99
    assert inputs[IMAGE_KEY][2].mean() < 0.01
    np.testing.assert_allclose(inputs["observation.state"], current["q"])
    assert inputs["task"] == "Move the blue box"


def test_wrong_camera_resolution_is_rejected_instead_of_warping():
    with pytest.raises(ValueError, match="640"):
        Pi05ObservationAdapter(load_config()).build(observation(size=(848, 480)), "Move")


def test_absolute_arm_angles_are_extracted_after_unnormalization_without_remote():
    actions = np.zeros((1, 50, 18), np.float32)
    actions[0, :, :14] = np.arange(14) / 10
    actions[0, :, 14:] = 1
    result = arm_chunk(actions, 8)
    assert result.shape == (8, 14)
    np.testing.assert_allclose(result[0], np.arange(14) / 10)
    actions[:] = 0
    assert result[0, 13] == pytest.approx(1.3)


@pytest.mark.parametrize("shape", [(1, 50, 14), (1, 50, 26), (50, 18), (2, 50, 18), (1, 3, 18)])
def test_incompatible_action_heads_fail(shape):
    with pytest.raises(ValueError):
        arm_chunk(np.zeros(shape), 8)


@pytest.mark.parametrize("mutation", ["camera", "state_order", "action_order", "relative", "stats"])
def test_incompatible_finetune_is_rejected(tmp_path, mutation):
    directory = tmp_path / "checkpoint"
    shutil.copytree(FIXTURE, directory)
    config = json.loads((directory / "config.json").read_text())
    dataset = json.loads((directory / "dataset_info.json").read_text())
    if mutation == "camera":
        config["input_features"]["observation.images.left_wrist"] = config["input_features"][IMAGE_KEY]
    elif mutation == "state_order":
        names = dataset["features"]["observation.state"]["names"]
        names[15], names[22] = names[22], names[15]
    elif mutation == "action_order":
        config["action_feature_names"][0] = "kRightShoulderPitch.q"
    elif mutation == "relative":
        config["use_relative_actions"] = True
    else:
        (directory / "policy_postprocessor_step_0_unnormalizer_processor.safetensors").unlink()
    (directory / "config.json").write_text(json.dumps(config))
    (directory / "dataset_info.json").write_text(json.dumps(dataset))
    with pytest.raises(ValueError):
        validate_checkpoint(directory)


def test_missing_finetuned_weights_cannot_fall_back_to_random_model():
    policy = Pi05Policy(load_config(), str(FIXTURE))
    with pytest.raises(ValueError, match="model.safetensors"):
        policy.load()
    assert policy.policy is None


def test_model_download_reuses_same_revision_and_refuses_stale_or_owned_files(tmp_path):
    old = tmp_path / "old.safetensors"
    new = tmp_path / "new.safetensors"
    old.write_bytes(b"old")
    new.write_bytes(b"new")
    target = tmp_path / "model.safetensors"
    link_cached_file(old, target)
    link_cached_file(old, target)
    with pytest.raises(ValueError, match="model_directory"):
        link_cached_file(new, target)
    assert target.read_bytes() == b"old"
    with pytest.raises(ValueError):
        link_cached_file(new, old)
    assert old.read_bytes() == b"old"


def tensor_from_fixture(filename, key):
    content = (FIXTURE / filename).read_bytes()
    length = struct.unpack("<Q", content[:8])[0]
    header = json.loads(content[8 : 8 + length])
    item = header[key]
    assert item["dtype"] == "F32"
    start, end = item["data_offsets"]
    return np.frombuffer(content[8 + length + start : 8 + length + end], dtype="<f4").reshape(item["shape"])


def test_saved_quantile_statistics_have_real_joint_units_and_expected_shapes():
    q01 = tensor_from_fixture("policy_postprocessor_step_0_unnormalizer_processor.safetensors", "action.q01")
    q99 = tensor_from_fixture("policy_postprocessor_step_0_unnormalizer_processor.safetensors", "action.q99")
    assert q01.shape == q99.shape == (18,)
    assert np.isfinite(q01).all() and np.isfinite(q99).all()
    assert len(STATE_NAMES) == 29 and ACTION_NAMES[:14] == STATE_NAMES[15:]
    assert np.any(np.abs(q01[:14]) > 0.5)


def test_pinned_lerobot_architecture_matches_all_published_weight_tensors():
    torch = pytest.importorskip("torch")
    pytest.importorskip("lerobot")
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy

    config = PreTrainedConfig.from_pretrained(str(FIXTURE))
    config.device = "meta"
    config.gradient_checkpointing = config.compile_model = False
    with torch.device("meta"):
        policy = PI05Policy(config)
    header = json.loads((FIXTURE / "weights_header.json").read_text())
    actual = policy.state_dict()
    assert set(actual) == set(header) - {"__metadata__"}
    assert len(actual) == 813
    assert not [name for name, tensor in actual.items() if list(tensor.shape) != header[name]["shape"]]


def test_real_lerobot_saved_normalization_pipeline_and_arm_extraction():
    torch = pytest.importorskip("torch")
    pytest.importorskip("lerobot")
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.pi05 import processor_pi05  # noqa: F401
    from tokenizers import Tokenizer
    from tokenizers.models import WordLevel
    from transformers import PreTrainedTokenizerFast

    # tokenizerの重み・利用許可に依存せず、実際のcheckpoint処理をCPUで検証する。
    tokenizer = PreTrainedTokenizerFast(
        tokenizer_object=Tokenizer(WordLevel({"[UNK]": 0, "[PAD]": 1}, unk_token="[UNK]")),
        unk_token="[UNK]",
        pad_token="[PAD]",
    )
    config = PreTrainedConfig.from_pretrained(str(FIXTURE))
    config.device = "cpu"
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=str(FIXTURE),
        preprocessor_overrides={
            "device_processor": {"device": "cpu"},
            "tokenizer_processor": {"tokenizer": tokenizer},
        },
    )
    q01 = tensor_from_fixture(
        "policy_preprocessor_step_2_normalizer_processor.safetensors", "observation.state.q01"
    )
    batch = preprocessor(
        {
            "observation.state": torch.from_numpy(q01.copy()),
            IMAGE_KEY: torch.zeros(3, 480, 640),
            "task": "Move the blue box.",
        }
    )
    torch.testing.assert_close(batch["observation.state"], -torch.ones(1, 29))
    assert batch["observation.language.tokens"].shape == (1, 200)
    assert IMAGE_KEY in batch

    class NormalizedPolicy:
        def predict_action_chunk(self, _batch):
            return torch.zeros(1, 50, 18)

    policy = Pi05Policy(load_config(), str(FIXTURE))
    policy.policy, policy.preprocessor, policy.postprocessor = NormalizedPolicy(), preprocessor, postprocessor
    inputs = Pi05ObservationAdapter(load_config()).build(observation(), "Move the blue box.")
    physical = policy.predict(inputs)
    low = tensor_from_fixture("policy_postprocessor_step_0_unnormalizer_processor.safetensors", "action.q01")
    high = tensor_from_fixture("policy_postprocessor_step_0_unnormalizer_processor.safetensors", "action.q99")
    np.testing.assert_allclose(physical[0], ((low + high) / 2)[:14], atol=1e-6)
    assert physical.shape == (8, 14)
