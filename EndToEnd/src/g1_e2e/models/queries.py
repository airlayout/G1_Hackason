"""固定query・座標queryを今回のMだけから読み出す。再帰状態は変更しない。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

import torch
from torch import Tensor, nn

from .config import ModelConfig, QueryConfig
from .heads import QueryHead


@dataclass(frozen=True)
class QueryRequest:
    """問い合わせ座標だけを渡す。XYZは現在のbase座標m、未来時刻は現在からの秒。"""

    coordinates: Tensor | None = None


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

    def validate(self, request: QueryRequest, batch_size: int, device: torch.device) -> int:
        """query数を返す。座標の意味・可視範囲はタスク側が管理する。"""
        if not isinstance(request, QueryRequest):
            raise TypeError("queryにはQueryRequestを渡してください")
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
            if not torch.isfinite(coords).all():
                raise ValueError("coordinatesは有限値です")
            if self.config.output_transform == "pose3d" and (coords < 0).any():
                raise ValueError("未来時刻は非負の秒です")
            count = coords.shape[1]
        else:
            if coords is not None:
                raise ValueError("learned queryにはcoordinatesを渡しません")
            count = self.config.num_queries
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
        return result + self.task


class QueryDecoder(nn.Module):
    """同じ今回のMから各headへ並列に分岐する。他headの予測・履歴を参照しない。"""

    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        self.chunk_size = config.query_chunk_size
        self.memory_shape = (config.memory_tokens, config.width)
        self.sources = nn.ModuleDict(
            {name: QuerySource(cfg, config.width) for name, cfg in config.queries.items()}
        )
        self.heads = nn.ModuleDict(
            {
                name: QueryHead(cfg, config.width, config.memory_tokens, config.ff_hidden_dim, config.dropout)
                for name, cfg in config.queries.items()
            }
        )

    def forward(
        self,
        memory: Tensor,
        requests: Mapping[str, QueryRequest] | None = None,
        *,
        chunk_size: int | None = None,
        raw: bool = False,
    ) -> dict[str, Tensor]:
        """出力[B,Q,C]。raw=Trueは出力変換前。eval時はquery集合・順序・分割に依存しない。"""
        if memory.ndim != 3 or memory.shape[0] < 1 or memory.shape[1:] != self.memory_shape:
            raise ValueError("decoderが読むのは固定形状の今回のMだけです")
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
        counts = {
            name: self.sources[name].validate(request, memory.shape[0], memory.device)
            for name, request in requests.items()
        }
        outputs = {}
        for name, request in requests.items():
            source, head = self.sources[name], self.heads[name]
            count = counts[name]
            if count == 0:
                outputs[name] = memory.new_empty(memory.shape[0], 0, head.config.output_dim)
                continue
            chunks = []
            for start in range(0, count, chunk_size):
                query = source(request, memory.shape[0], start, min(start + chunk_size, count))
                chunks.append(head(memory, query, raw=raw))
            outputs[name] = torch.cat(chunks, dim=1)
        return outputs
