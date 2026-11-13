"""EmbeddingGemma 2 に音声波形を直接渡してベクトルを作る（文字起こしは行わない）。"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np

from Voice.audio_input import AudioClip, SAMPLE_RATE

DEFAULT_MODEL = "google/embeddinggemma-2"
EMBEDDING_DIM = 768


class EmbeddingGemma:
    def __init__(self, model_name: str = DEFAULT_MODEL, device: str = "auto", *, local_files_only: bool = False) -> None:
        # 判定・実行側のテストや --list-tasks ではモデルを読み込まない。
        import torch
        from transformers import AutoConfig, AutoModel, AutoProcessor

        self.torch = torch
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu") if device == "auto" else torch.device(device)
        dtype = torch.float32
        if self.device.type == "cuda":
            if not torch.cuda.is_available():
                raise ValueError("CUDA が使えません。--device cpu を指定してください")
            with torch.cuda.device(self.device):
                if torch.cuda.is_bf16_supported():
                    dtype = torch.bfloat16
        config = AutoConfig.from_pretrained(model_name, local_files_only=local_files_only)
        if config.model_type != "embedding_gemma2" or config.audio_config is None:
            raise ValueError("音声エンコーダ付きの EmbeddingGemma 2 を指定してください")
        # 音声とテキストだけを使うので、画像エンコーダはメモリに載せない。
        config.vision_config = None
        self.processor = AutoProcessor.from_pretrained(model_name, local_files_only=local_files_only)
        self.model = AutoModel.from_pretrained(model_name, config=config, dtype=dtype, local_files_only=local_files_only).to(self.device).eval()
        self.model_name = model_name
        self.revision = getattr(config, "_commit_hash", None)

    def encode_audio(self, clip: AudioClip) -> np.ndarray:
        """生の波形を入力し、正規化済みの 768 次元ベクトルを返す。"""
        if clip.sample_rate != SAMPLE_RATE:
            raise ValueError("音声を 16 kHz に整えてから入力してください")
        inputs = self.processor(
            audio=[clip.samples], sampling_rate=clip.sample_rate,
            padding=True, return_tensors="pt",
        )
        return self._embed(inputs)[0]

    def encode_documents(self, descriptions: Sequence[str]) -> np.ndarray:
        """登録済みタスクの説明を、音声と比較できる同じ空間に埋め込む。"""
        inputs = self.processor(
            text=[f"title: none | text: {text}" for text in descriptions],
            padding=True, return_tensors="pt",
        )
        return self._embed(inputs)

    def _embed(self, inputs: Any) -> np.ndarray:
        torch = self.torch
        inputs = inputs.to(self.device)
        if inputs["input_ids"].shape[1] > 8192:
            raise ValueError("モデルのコンテキスト長を超えています")
        with torch.inference_mode():
            hidden = self.model(**inputs).last_hidden_state.float()
            # 公式実装と同じ、padding を除く平均プーリング → L2 正規化。
            mask = inputs["attention_mask"].unsqueeze(-1).float()
            pooled = (hidden * mask).sum(dim=1) / mask.sum(dim=1).clamp(min=1e-9)
            norms = pooled.norm(dim=-1, keepdim=True)
            if not torch.isfinite(pooled).all() or not torch.isfinite(norms).all() or (norms <= 1e-9).any():
                raise RuntimeError("モデルが有効なベクトルを返しませんでした")
            embeddings = (pooled / norms).cpu().numpy()
        if embeddings.ndim != 2 or embeddings.shape[1] != EMBEDDING_DIM:
            raise RuntimeError(f"想定外のベクトル形状です: {embeddings.shape}")
        return embeddings
