"""GR00Tの公開Policy APIと、G1の観測・腕14関節との境界。"""

from __future__ import annotations

import base64
from collections import deque
import io
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from ..config import ROOT, finite_array
from ..kinematics import WristKinematics


class ObservationAdapter:
    def __init__(self, config: dict, joint_names: list[str]):
        self.config = config
        urdf = Path(config["urdf"])
        self.kinematics = WristKinematics(
            urdf if urdf.is_absolute() else ROOT / urdf,
            joint_names,
            config["eef_base_link"],
            config["eef_links"],
        )
        self.history = deque(maxlen=120)

    def record(self, observation: dict, *, now: float | None = None) -> None:
        timestamp = time.monotonic() if now is None else now
        with Image.open(io.BytesIO(base64.b64decode(observation["jpeg"], validate=True))) as image:
            if image.width > 1920 or image.height > 1080:
                raise ValueError("カメラ画像が大きすぎます")
            rgb = np.asarray(image.convert("RGB").resize((640, 480)), dtype=np.uint8).copy()
        self.history.append((timestamp, rgb))

    def build(self, observation: dict, instruction: str, *, now: float | None = None) -> dict:
        timestamp = time.monotonic() if now is None else now
        q = finite_array(observation["q"], (29,), "q").astype(np.float32)
        if not self.history:
            self.record(observation, now=timestamp)
        cutoff = timestamp - self.config["video_history_s"]
        past = min(self.history, key=lambda item: abs(item[0] - cutoff))[1]
        current = self.history[-1][1]
        wrists = self.kinematics.poses(q)
        left_hand, right_hand = np.asarray(self.config["fixed_hand_state"], dtype=np.float32)
        states = {
            "left_wrist_eef_9d": wrists[0],
            "right_wrist_eef_9d": wrists[1],
            "left_arm": q[15:22],
            "right_arm": q[22:29],
            "waist": q[12:15],
            "left_hand": left_hand,
            "right_hand": right_hand,
        }
        return {
            "video": {"ego_view": np.stack([past, current])[None]},
            "state": {name: value[None, None].copy() for name, value in states.items()},
            "language": {"annotation.human.task_description": [[instruction]]},
        }


def arm_chunk(actions: dict, steps: int) -> np.ndarray:
    arrays = [np.asarray(actions[name], dtype=np.float32) for name in ["left_arm", "right_arm"]]
    if any(a.ndim != 3 or a.shape[0] != 1 or a.shape[2] != 7 or a.shape[1] < steps for a in arrays):
        raise ValueError("GR00Tの出力はbatch=1、各腕7関節の時系列である必要があります")
    chunk = np.concatenate([a[0, :steps] for a in arrays], axis=1)
    if not np.isfinite(chunk).all():
        raise ValueError("GR00Tの予測にNaN/Infがあります")
    # Policy APIが相対予測を絶対radへ復元済み。現在角度を二重に加算しない。
    return chunk


class GrootPolicy:
    def __init__(self, config: dict, model_path: str | None = None):
        self.config = config
        self.model_path = model_path or str(ROOT / "assets/cache/vla/model")
        self.policy = None

    def load(self) -> None:
        if self.policy is not None:
            return
        import torch
        from gr00t.policy import Gr00tPolicy

        if not torch.cuda.is_available():
            raise RuntimeError("CUDAが使えません。CUDA 12.8版PyTorchでセットアップしてください")
        device = self.config["device"]
        if torch.cuda.get_device_capability(device) < (8, 0):
            raise RuntimeError("bf16対応GPUを指定してください。5060 Tiは通常cuda:0です")
        free, total = torch.cuda.mem_get_info(device)
        if total < 15 * 1024**3 or free < 10 * 1024**3:
            raise RuntimeError("VRAMが不足しています。16GBのGPUを選び、他のGPUアプリを終了してください")
        if not (Path(self.model_path) / "processor_config.json").is_file():
            raise RuntimeError("重みがありません。bash EndToEnd/arm-setup.sh --model を実行してください")
        self.policy = Gr00tPolicy(
            embodiment_tag=self.config["embodiment"], model_path=self.model_path, device=device, strict=True
        )
        modalities = self.policy.get_modality_config()
        expected = {
            "left_wrist_eef_9d",
            "right_wrist_eef_9d",
            "left_arm",
            "right_arm",
            "waist",
            "left_hand",
            "right_hand",
        }
        if (
            modalities["video"].modality_keys != ["ego_view"]
            or modalities["video"].delta_indices != [-20, 0]
            or set(modalities["state"].modality_keys) != expected
            or modalities["state"].delta_indices != [0]
        ):
            self.policy = None
            raise RuntimeError("このcheckpointの観測形式はREAL_G1アダプタと一致しません")

    def predict(self, observation: dict) -> np.ndarray:
        if self.policy is None:
            self.load()
        actions, _info = self.policy.get_action(observation)
        return arm_chunk(actions, self.config["execute_steps"])

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
    """HFキャッシュは変更せず、固定したbackboneを参照する実行用bundleを作る。"""
    from huggingface_hub import hf_hub_download, snapshot_download
    from huggingface_hub.errors import GatedRepoError

    # 先にbackboneの利用許可を確認し、本体の大きな重みを無駄に取得しない。
    try:
        hf_hub_download(config["backbone_id"], "config.json", revision=config["backbone_revision"])
    except GatedRepoError as error:
        raise RuntimeError(
            f"https://huggingface.co/{config['backbone_id']} でアクセス許可を取得し、hf auth loginを実行してください"
        ) from error

    target = ROOT / "assets/cache/vla/model"
    source = Path(
        snapshot_download(
            config["model_id"], revision=config["model_revision"], allow_patterns=["*.json", "*.safetensors"]
        )
    )
    backbone = snapshot_download(
        config["backbone_id"],
        revision=config["backbone_revision"],
        allow_patterns=["*.json", "*.safetensors", "*.txt", "*.jinja", "*.model"],
    )
    target.mkdir(parents=True, exist_ok=True)
    for path in source.iterdir():
        destination = target / path.name
        if path.name in {"config.json", "processor_config.json"}:
            data = json.loads(path.read_text())
            container = data.get("processor_kwargs", data)
            container["model_name"] = backbone
            # 5060 Ti(sm_120)ではFlashAttention2を使わない。公式SDPA経路へ進む。
            if path.name == "config.json":
                container["use_flash_attention"] = False
            destination.write_text(json.dumps(data, indent=2))
        elif path.is_file() and not destination.exists():
            destination.symlink_to(path)
    (target / "g1_bundle.json").write_text(
        json.dumps(
            {
                "model": config["model_id"],
                "revision": config["model_revision"],
                "backbone": config["backbone_id"],
                "backbone_revision": config["backbone_revision"],
            },
            indent=2,
        )
    )
