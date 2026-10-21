"""今回のMとqueryを結合する、taskごとに独立した点単位の出力MLP。"""

from __future__ import annotations

import torch
from torch import Tensor, nn
import torch.nn.functional as F

from .config import QueryConfig
from .feedforward import SwiGLU


class QueryHead(nn.Module):
    """履歴・観測・他headの出力を受け取らず、固定形状の今回のMだけで条件付けする。"""

    def __init__(
        self, config: QueryConfig, width: int, memory_tokens: int, hidden_dim: int, dropout: float
    ) -> None:
        super().__init__()
        self.config = config
        self.memory_norm = nn.LayerNorm(memory_tokens * width)
        self.condition = nn.Linear((memory_tokens + 1) * width, width)
        self.ff_norm = nn.LayerNorm(width)
        self.ff = SwiGLU(width, hidden_dim, dropout)
        self.projection = nn.Sequential(nn.LayerNorm(width), nn.Linear(width, config.output_dim))
        self.register_buffer("output_min", torch.tensor(config.output_min))
        self.register_buffer("output_max", torch.tensor(config.output_max))
        if config.output_transform == "pose3d":
            with torch.no_grad():
                self.projection[-1].bias[3:6].zero_()
                self.projection[-1].bias[6] = 1.0

    def forward(self, memory: Tensor, query: Tensor, *, raw: bool = False) -> Tensor:
        """[B,Q,C]。raw=Trueはsigmoid等の変換前で、場のBCEWithLogits学習にも使える。"""
        current = self.memory_norm(memory.flatten(1))[:, None].expand(-1, query.shape[1], -1)
        features = self.condition(torch.cat((current, query), dim=-1))
        features = features + self.ff(self.ff_norm(features))
        result = self.projection(features)
        if raw:
            return result
        if self.config.output_transform == "occupancy":
            # 各成分は独立な確率。[0]=占有、[1]=観測済み。未知を空き空間と混同しない。
            return result.sigmoid()
        if self.config.output_transform == "bounded":
            low, high = self.output_min.to(result.dtype), self.output_max.to(result.dtype)
            return low + (high - low) * (result.tanh() + 1) / 2
        if self.config.output_transform == "pose3d":
            quaternion = result[..., 3:].float()
            norm = torch.linalg.vector_norm(quaternion, dim=-1, keepdim=True)
            identity = quaternion.new_tensor([0.0, 0.0, 0.0, 1.0])
            quaternion = torch.where(norm > 1e-6, F.normalize(quaternion, dim=-1, eps=1e-6), identity)
            return torch.cat((result[..., :3], quaternion.to(result.dtype)), dim=-1)
        return result
