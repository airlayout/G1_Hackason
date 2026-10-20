"""固定query・座標queryを共通メモリから読み出す。観測や再帰状態は変更しない。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

import torch
from torch import Tensor, nn
import torch.nn.functional as F

from .attention import CrossRead
from .config import ModelConfig, QueryConfig


@dataclass(frozen=True)
class QueryRequest:
    """coordinates=[B,Q,C]、context=[B,C]または[B,Q,C]。単位はquery設定に従う。"""

    coordinates: Tensor | None = None
    context: Tensor | None = None


class QuerySource(nn.Module):
    """学習したqueryか、Fourier座標特徴からtask固有のqueryを生成する。"""

    def __init__(self, config: QueryConfig, width: int) -> None:
        super().__init__()
        self.config = config
        self.task = nn.Parameter(torch.randn(1, 1, width) * 0.02)
        if config.kind == "learned":
            self.learned = nn.Parameter(torch.randn(1, config.num_queries, width) * 0.02)
            self.coordinate_projection = None
        else:
            self.register_parameter("learned", None)
            encoded_dim = config.coordinate_dim * (1 + 2 * config.fourier_bands)
            self.coordinate_projection = nn.Linear(encoded_dim, width)
        self.register_buffer("scale", torch.tensor(config.coordinate_scale or [1.0] * config.coordinate_dim))
        self.register_buffer("frequencies", math.pi * 2.0 ** torch.arange(config.fourier_bands))
        self.context_projection = nn.Linear(config.context_dim, width) if config.context_dim else None

    def validate(self, request: QueryRequest, batch_size: int, device: torch.device) -> int:
        """query数を返す。座標の意味・可視範囲はタスク側が管理する。"""
        coords = request.coordinates
        if self.config.kind == "coordinate":
            if (
                coords is None
                or coords.ndim != 3
                or coords.shape[0] != batch_size
                or coords.shape[2] != self.config.coordinate_dim
                or not coords.is_floating_point()
                or coords.device != device
            ):
                raise ValueError("coordinatesはメモリと同じdeviceの浮動小数点[B,Q,coordinate_dim]です")
            count = coords.shape[1]
        else:
            if coords is not None:
                raise ValueError("learned queryにはcoordinatesを渡しません")
            count = self.config.num_queries
        context = request.context
        if self.context_projection is None:
            if context is not None:
                raise ValueError("contextを使うqueryにはcontext_dimを設定してください")
        elif (
            context is None
            or context.shape
            not in {(batch_size, self.config.context_dim), (batch_size, count, self.config.context_dim)}
            or not context.is_floating_point()
            or context.device != device
        ):
            raise ValueError("contextはメモリと同じdeviceの浮動小数点[B,C]または[B,Q,C]です")
        return count

    def forward(self, request: QueryRequest, batch_size: int, start: int, end: int) -> Tensor:
        """座標埋め込みも分割生成し、密な場の推論時メモリを抑える。"""
        if self.learned is not None:
            result = self.learned[:, start:end].expand(batch_size, -1, -1)
        else:
            coords = request.coordinates[:, start:end]
            # Fourier特徴は低精度の大きな位相誤差を避けてfloat32で計算する。
            coords = coords.float() * self.scale.float()
            phase = coords[..., None] * self.frequencies.float()
            encoded = torch.cat((coords, phase.sin().flatten(2), phase.cos().flatten(2)), dim=-1)
            result = self.coordinate_projection(encoded.to(self.task.dtype))
        result = result + self.task
        if self.context_projection is not None:
            context = request.context
            context = context[:, None] if context.ndim == 2 else context[:, start:end]
            result = result + self.context_projection(context.to(self.task.dtype))
        return result


class QueryHead(nn.Module):
    """task別の出力投影。姿勢はxyzwの単位quaternionへ写す。"""

    def __init__(self, config: QueryConfig, width: int) -> None:
        super().__init__()
        self.config = config
        self.projection = nn.Sequential(nn.LayerNorm(width), nn.Linear(width, config.output_dim))
        if config.output_transform == "pose3d":
            with torch.no_grad():
                self.projection[-1].bias[3:6].zero_()
                self.projection[-1].bias[6] = 1.0

    def forward(self, features: Tensor) -> Tensor:
        result = self.projection(features)
        if self.config.output_transform == "pose3d":
            # normはfloat32で評価し、ゼロ近傍は恒等回転へ置換する。
            quaternion = result[..., 3:].float()
            norm = torch.linalg.vector_norm(quaternion, dim=-1, keepdim=True)
            identity = quaternion.new_tensor([0.0, 0.0, 0.0, 1.0])
            quaternion = torch.where(norm > 1e-6, F.normalize(quaternion, dim=-1, eps=1e-6), identity)
            result = torch.cat((result[..., :3], quaternion.to(result.dtype)), dim=-1)
        return result


class QueryDecoder(nn.Module):
    """task間でcross-attentionを共有し、query sourceとheadを差し替える。"""

    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        self.chunk_size = config.query_chunk_size
        self.sources = nn.ModuleDict(
            {name: QuerySource(cfg, config.width) for name, cfg in config.queries.items()}
        )
        self.heads = nn.ModuleDict(
            {name: QueryHead(cfg, config.width) for name, cfg in config.queries.items()}
        )
        self.read = CrossRead(config.width, config.heads, config.ff_multiplier, config.dropout)

    def forward(
        self,
        latents: Tensor,
        requests: Mapping[str, QueryRequest] | None = None,
        *,
        chunk_size: int | None = None,
    ) -> dict[str, Tensor]:
        """出力[B,Q,C]。eval時はquery集合・順序・分割サイズに依存しない。"""
        if requests is None:
            requests = {
                name: QueryRequest()
                for name, source in self.sources.items()
                if source.config.kind == "learned"
            }
        unknown = set(requests) - set(self.sources)
        if unknown:
            raise ValueError(f"未定義のquery: {sorted(unknown)}")
        chunk_size = self.chunk_size if chunk_size is None else chunk_size
        if chunk_size < 1:
            raise ValueError("chunk_sizeは正数です")
        outputs = {}
        for name, request in requests.items():
            source, head = self.sources[name], self.heads[name]
            count = source.validate(request, latents.shape[0], latents.device)
            if count == 0:
                outputs[name] = latents.new_empty(latents.shape[0], 0, head.config.output_dim)
                continue
            chunks = []
            for start in range(0, count, chunk_size):
                query = source(request, latents.shape[0], start, min(start + chunk_size, count))
                chunks.append(head(self.read(query, latents)))
            outputs[name] = torch.cat(chunks, dim=1)
        return outputs
