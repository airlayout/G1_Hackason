"""G1の単眼・身体29関節でFTされたπ0.5。正規化統計もcheckpointから読む。"""

from __future__ import annotations

import base64
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from ..config import finite_array


STATE_NAMES = (
    "kLeftHipPitch.q",
    "kLeftHipRoll.q",
    "kLeftHipYaw.q",
    "kLeftKnee.q",
    "kLeftAnklePitch.q",
    "kLeftAnkleRoll.q",
    "kRightHipPitch.q",
    "kRightHipRoll.q",
    "kRightHipYaw.q",
    "kRightKnee.q",
    "kRightAnklePitch.q",
    "kRightAnkleRoll.q",
    "kWaistYaw.q",
    "kWaistRoll.q",
    "kWaistPitch.q",
    "kLeftShoulderPitch.q",
    "kLeftShoulderRoll.q",
    "kLeftShoulderYaw.q",
    "kLeftElbow.q",
    "kLeftWristRoll.q",
    "kLeftWristPitch.q",
    "kLeftWristyaw.q",
    "kRightShoulderPitch.q",
    "kRightShoulderRoll.q",
    "kRightShoulderYaw.q",
    "kRightElbow.q",
    "kRightWristRoll.q",
    "kRightWristPitch.q",
    "kRightWristYaw.q",
)
ACTION_NAMES = STATE_NAMES[15:] + ("remote.lx", "remote.ly", "remote.rx", "remote.ry")
IMAGE_KEY = "observation.images.global_view"


def read_json(directory: Path, name: str) -> dict:
    path = directory / name
    if not path.is_file():
        raise ValueError(f"checkpointに{name}がありません。arm-setup.sh --modelで準備してください")
    return json.loads(path.read_text())


def validate_checkpoint(directory: Path) -> dict:
    """次元だけでなく関節の順序と、学習時の前後処理を照合する。"""
    config = read_json(directory, "config.json")
    dataset = read_json(directory, "dataset_info.json")
    inputs = config.get("input_features", {})
    if config.get("type") != "pi05" or config.get("n_obs_steps") != 1:
        raise ValueError("単時刻観測のπ0.5 checkpointが必要です")
    if set(inputs) != {"observation.state", IMAGE_KEY}:
        raise ValueError("このアダプタはglobal_view単眼のG1モデルを使います")
    if inputs["observation.state"]["shape"] != [29] or inputs[IMAGE_KEY]["shape"] != [3, 480, 640]:
        raise ValueError("入力はSDK順29関節とRGB 640×480です")
    if config.get("output_features", {}).get("action", {}).get("shape") != [18]:
        raise ValueError("出力は腕14関節とremote 4要素の18次元が必要です")
    if tuple(config.get("action_feature_names", ())) != ACTION_NAMES:
        raise ValueError("checkpointのaction関節順がG1アダプタと異なります")
    if config.get("use_relative_actions", False):
        raise ValueError("このアダプタは絶対関節角でFTされたcheckpointを使います")
    features = dataset.get("features", {})
    if tuple(features.get("observation.state", {}).get("names", ())) != STATE_NAMES:
        raise ValueError("学習データの身体29関節の順序がSDKと一致しません")
    if tuple(features.get("action", {}).get("names", ())) != ACTION_NAMES:
        raise ValueError("学習データのaction順序が一致しません")
    if dataset.get("fps") != 30:
        raise ValueError("30Hzで収集されたG1データが必要です")
    for filename, registry, feature in [
        ("policy_preprocessor.json", "normalizer_processor", "observation.state"),
        ("policy_postprocessor.json", "unnormalizer_processor", "action"),
    ]:
        pipeline = read_json(directory, filename)
        steps = [step for step in pipeline["steps"] if step.get("registry_name") == registry]
        if len(steps) != 1:
            raise ValueError(f"{filename}の学習済み正規化がありません")
        step = steps[0]
        normalization = step["config"]["norm_map"]
        if normalization.get("STATE" if feature == "observation.state" else "ACTION") != "QUANTILES":
            raise ValueError("学習時のQUANTILES正規化が必要です")
        state_file = step.get("state_file", "")
        if Path(state_file).name != state_file or not (directory / state_file).is_file():
            raise ValueError(f"{filename}の学習済み統計がありません")
    return config


class Pi05ObservationAdapter:
    def __init__(self, config: dict):
        self.config = config

    def record(self, observation: dict, *, now: float | None = None) -> None:
        # π0.5は履歴を使わない。buildで同じ観測の画像と関節角を対にする。
        pass

    def build(self, observation: dict, instruction: str, *, now: float | None = None) -> dict:
        q = finite_array(observation["q"], (29,), "q").astype(np.float32)
        jpeg = base64.b64decode(observation["jpeg"], validate=True)
        with Image.open(io.BytesIO(jpeg)) as image:
            if image.size != (640, 480):
                raise ValueError("π0.5の頭部RGB画像は640×480が必要です")
            rgb = np.asarray(image.convert("RGB"), dtype=np.float32).copy()
        # LeRobotの画像入力はCHW・0..1。resize/padは公式policyに任せる。
        return {"observation.state": q, IMAGE_KEY: rgb.transpose(2, 0, 1) / 255.0, "task": instruction}


def arm_chunk(actions, steps: int) -> np.ndarray:
    array = np.asarray(actions, dtype=np.float32)
    if array.ndim != 3 or array.shape[0] != 1 or array.shape[2] != 18 or array.shape[1] < steps:
        raise ValueError("π0.5の出力はbatch=1・18次元のaction chunkです")
    if not np.isfinite(array).all():
        raise ValueError("π0.5の予測にNaN/Infがあります")
    # postprocessorで物理単位に戻した後に腕だけ取り出す。remoteは送らない。
    return array[0, :steps, :14].copy()


def validate_runtime_files(directory: Path) -> None:
    validate_checkpoint(directory)
    for name in ["model.safetensors", "tokenizer/tokenizer_config.json", "tokenizer/tokenizer.json"]:
        if not (directory / name).is_file():
            raise ValueError(f"{name}がありません。arm-setup.sh --modelを実行してください")


class Pi05Policy:
    def __init__(self, config: dict, model_path: str):
        self.config, self.model_path = config, Path(model_path)
        self.policy = self.preprocessor = self.postprocessor = None

    def load(self) -> None:
        if self.policy is not None:
            return
        validate_runtime_files(self.model_path)
        weights = self.model_path / "model.safetensors"
        tokenizer = self.model_path / "tokenizer"
        import torch
        from safetensors.torch import load_model
        from lerobot.configs import PreTrainedConfig
        from lerobot.policies.pi05.modeling_pi05 import PI05Policy
        from lerobot.policies.pi05 import processor_pi05  # noqa: F401
        from lerobot.policies.factory import make_pre_post_processors

        device = self.config["device"]
        if not torch.cuda.is_available() or torch.cuda.get_device_capability(device) < (8, 0):
            raise RuntimeError("bf16対応のCUDA GPUとCUDA 12.8版PyTorchが必要です")
        free, total = torch.cuda.mem_get_info(device)
        if total < 15 * 1024**3 or free < 11 * 1024**3:
            raise RuntimeError("16GBのGPUを指定し、他のGPUアプリを終了してください")
        policy_config = PreTrainedConfig.from_pretrained(str(self.model_path))
        policy_config.device = "cpu"
        policy_config.dtype = "bfloat16"
        policy_config.compile_model = False
        policy_config.gradient_checkpointing = False
        policy_config.num_inference_steps = self.config["num_inference_steps"]
        # 上流from_pretrainedは読込失敗を警告だけで返す場合があるため使わない。
        # strict=Trueで重み・共有embeddingまで復元し、失敗時は必ず実行を止める。
        policy = PI05Policy(policy_config)
        load_model(policy, str(weights), strict=True, device="cpu")
        policy.requires_grad_(False).eval()
        policy.to(device)
        policy.config.device = device
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy_config,
            pretrained_path=str(self.model_path),
            preprocessor_overrides={
                "device_processor": {"device": device},
                "tokenizer_processor": {"tokenizer_name": str(tokenizer)},
            },
        )
        self.preprocessor, self.postprocessor, self.policy = preprocessor, postprocessor, policy

    def predict(self, observation: dict) -> np.ndarray:
        self.load()
        import torch

        batch = {
            key: torch.from_numpy(value) if isinstance(value, np.ndarray) else value
            for key, value in observation.items()
        }
        with torch.inference_mode():
            normalized = self.policy.predict_action_chunk(self.preprocessor(batch))
            physical = self.postprocessor(normalized)
        return arm_chunk(physical.detach().float().cpu().numpy(), self.config["execute_steps"])

    def metrics(self) -> dict:
        if self.policy is None:
            return {"loaded": False}
        import torch

        device = self.config["device"]
        return {
            "loaded": True,
            "gpu": torch.cuda.get_device_name(device),
            "vram_gb": round(torch.cuda.memory_allocated(device) / 1024**3, 2),
            "peak_vram_gb": round(torch.cuda.max_memory_allocated(device) / 1024**3, 2),
        }


def download_model(config: dict) -> None:
    from huggingface_hub import snapshot_download, hf_hub_download
    from huggingface_hub.errors import GatedRepoError
    from ..policy import model_directory

    target = model_directory(config)
    manifest = {
        "backend": "pi05",
        "model": config["model_id"],
        "revision": config["model_revision"],
        "dataset": config["dataset_id"],
        "dataset_revision": config["dataset_revision"],
        "tokenizer": config["tokenizer_id"],
        "tokenizer_revision": config["tokenizer_revision"],
    }
    manifest_path = target / "g1_bundle.json"
    if manifest_path.exists() and read_json(target, manifest_path.name) != manifest:
        raise ValueError("取得先に別版のモデルがあります。model_directoryを別の場所に指定してください")

    # tokenizerだけを取得する。PaliGemma本体の重みはFT済みモデルに含まれる。
    try:
        tokenizer = Path(
            snapshot_download(
                config["tokenizer_id"],
                revision=config["tokenizer_revision"],
                allow_patterns=["tokenizer*", "added_tokens.json", "special_tokens_map.json"],
            )
        )
    except GatedRepoError as error:
        raise RuntimeError(
            f"https://huggingface.co/{config['tokenizer_id']} の利用許可とhf auth loginが必要です"
        ) from error
    source = Path(
        snapshot_download(
            config["model_id"],
            revision=config["model_revision"],
            allow_patterns=["*.json", "*.safetensors", "README.md"],
        )
    )
    dataset_info = hf_hub_download(
        config["dataset_id"], "meta/info.json", repo_type="dataset", revision=config["dataset_revision"]
    )
    target.mkdir(parents=True, exist_ok=True)
    for path in source.iterdir():
        if path.is_file():
            link_cached_file(path, target / path.name)
    link_cached_file(tokenizer, target / "tokenizer")
    (target / "dataset_info.json").write_bytes(Path(dataset_info).read_bytes())
    validate_runtime_files(target)
    manifest_path.write_text(json.dumps(manifest, indent=2))


def link_cached_file(source: Path, destination: Path) -> None:
    """既存の別版や利用者のファイルを上書きせず、同じキャッシュなら再利用する。"""
    if destination.is_symlink() and destination.resolve() == source.resolve():
        return
    if destination.exists() or destination.is_symlink():
        raise ValueError(f"{destination}に別のファイルがあります。model_directoryを変更してください")
    destination.symlink_to(source, target_is_directory=source.is_dir())
