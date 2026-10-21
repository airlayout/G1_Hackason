"""入力の並び順ではなく、共通時間軸上の取得時刻でQ/Kを回転する。"""

import torch
from torch import Tensor, nn


class RotaryTimeEmbedding(nn.Module):
    """時刻は制御周期単位。長さに依存する学習パラメーターは持たない。"""

    def __init__(self, head_width: int, base: float) -> None:
        super().__init__()
        frequencies = base ** (-torch.arange(0, head_width, 2).float() / head_width)
        self.register_buffer("frequencies", frequencies)

    def forward(self, value: Tensor, times: Tensor) -> Tensor:
        """value=[B,heads,N,D]、times=[B,N]。位相と回転はfloat32で評価する。"""
        phase = times.float()[:, None, :, None] * self.frequencies.float()
        pairs = value.float().unflatten(-1, (-1, 2))
        x, y = pairs.unbind(-1)
        cosine, sine = phase.cos(), phase.sin()
        return torch.stack((x * cosine - y * sine, x * sine + y * cosine), dim=-1).flatten(-2).to(value.dtype)
