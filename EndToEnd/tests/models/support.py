"""NN検証で共通の小さい構成・観測・履歴。"""

from pathlib import Path

import pytest

torch = pytest.importorskip("torch", reason="NNの検証にはEndToEnd[model]が必要です")

from g1_e2e.models import MemoryState, ModalityConfig, ModelConfig, QueryConfig, SensorTokens  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


def config(**overrides):
    return ModelConfig(
        **(
            {
                "modalities": {"points": ModalityConfig(3, 8, position_dim=3), "imu": ModalityConfig(4, 1)},
                "queries": {
                    "velocity": QueryConfig(3),
                    "bev": QueryConfig(5, kind="coordinate", coordinate_dim=3),
                    "trajectory": QueryConfig(
                        7, kind="coordinate", coordinate_dim=1, output_transform="pose3d"
                    ),
                },
                "width": 16,
                "memory_tokens": 1,
                "layers": 1,
                "heads": 2,
                "ff_multiplier": 2,
                "ff_multiple_of": 8,
                "recurrence_period": 1,
                "context_window": 16,
            }
            | overrides
        )
    )


def observations(batch_size=1, *, requires_grad=False):
    features = torch.randn(batch_size, 5, 3, requires_grad=requires_grad)
    return {"points": SensorTokens(features, positions=torch.randn(batch_size, 5, 3))}


def history_state(model, frames=3, batch_size=1):
    n = frames * model.config.memory_tokens
    return MemoryState(
        tokens=torch.randn(batch_size, n, model.config.width),
        valid=torch.ones(batch_size, n, dtype=torch.bool),
        token_steps=torch.arange(frames)
        .repeat_interleave(model.config.memory_tokens)[None]
        .expand(batch_size, -1),
        step=torch.full((batch_size,), frames, dtype=torch.long),
    )
