from __future__ import annotations

import importlib.util
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from Voice.audio_input import AudioClip
from Voice.embedding_gemma import EMBEDDING_DIM, EmbeddingGemma


@unittest.skipUnless(importlib.util.find_spec("torch"), "torch 未導入。モデル境界のテストは推論用環境で実行")
class EmbeddingGemmaTests(unittest.TestCase):
    def setUp(self) -> None:
        import torch

        class Batch(dict):
            def to(self, device: object) -> Batch:
                return self

        self.encoder = object.__new__(EmbeddingGemma)
        self.encoder.torch = torch
        self.encoder.device = torch.device("cpu")
        self.encoder.processor = Mock(return_value=Batch({
            "input_ids": torch.tensor([[1, 2, 0]]),
            "attention_mask": torch.tensor([[1, 1, 0]]),
        }))
        hidden = torch.zeros((1, 3, EMBEDDING_DIM))
        hidden[0, 0, 0] = 2
        hidden[0, 1, 1] = 2
        hidden[0, 2, :] = 1000  # padding は平均に含めない。
        self.encoder.model = Mock(return_value=SimpleNamespace(last_hidden_state=hidden))

    def test_waveform_goes_directly_to_audio_processor(self) -> None:
        samples = np.ones(4_000, dtype=np.float32) * 0.1
        vector = self.encoder.encode_audio(AudioClip(samples))
        kwargs = self.encoder.processor.call_args.kwargs
        self.assertIs(kwargs["audio"][0], samples)
        self.assertNotIn("text", kwargs)
        self.assertEqual(kwargs["sampling_rate"], 16_000)
        self.assertEqual(vector.shape, (EMBEDDING_DIM,))
        np.testing.assert_allclose(vector[:2], [2 ** -0.5, 2 ** -0.5], atol=1e-6)
        np.testing.assert_allclose(np.linalg.norm(vector), 1, atol=1e-6)

    def test_document_prompt_is_separate_from_audio(self) -> None:
        self.encoder.encode_documents(["ボトルを押す"])
        self.assertEqual(self.encoder.processor.call_args.kwargs["text"], ["title: none | text: ボトルを押す"])
        self.assertNotIn("audio", self.encoder.processor.call_args.kwargs)

    def test_nonfinite_model_output_is_rejected(self) -> None:
        self.encoder.model.return_value.last_hidden_state.fill_(float("nan"))
        with self.assertRaises(RuntimeError):
            self.encoder.encode_audio(AudioClip(np.ones(4_000, dtype=np.float32)))

    def test_overflowed_norm_is_rejected(self) -> None:
        self.encoder.model.return_value.last_hidden_state.fill_(1e38)
        with self.assertRaises(RuntimeError):
            self.encoder.encode_audio(AudioClip(np.ones(4_000, dtype=np.float32)))
