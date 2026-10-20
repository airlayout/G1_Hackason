"""再帰メモリ・観測encoder・queryを構成する、Isaacから独立した設定。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import yaml


@dataclass(frozen=True)
class ModalityConfig:
    """1種類の観測をトークン列へ写すための形状。"""

    feature_dim: int
    max_tokens: int
    position_dim: int = 0
    learned_positions: bool = False

    def __post_init__(self) -> None:
        if self.feature_dim < 1 or self.max_tokens < 1 or self.position_dim < 0:
            raise ValueError("modalityの次元・トークン上限が不正です")


@dataclass(frozen=True)
class QueryConfig:
    """固定の学習可能query、または任意座標から生成するquery。"""

    output_dim: int
    kind: Literal["learned", "coordinate"] = "learned"
    num_queries: int = 1
    coordinate_dim: int = 0
    coordinate_scale: tuple[float, ...] = ()
    fourier_bands: int = 6
    context_dim: int = 0
    output_transform: Literal["identity", "pose3d"] = "identity"

    def __post_init__(self) -> None:
        if self.output_dim < 1 or self.num_queries < 1 or self.context_dim < 0 or self.fourier_bands < 0:
            raise ValueError("queryの次元が不正です")
        if self.kind not in {"learned", "coordinate"}:
            raise ValueError(f"未知のquery kind: {self.kind}")
        if self.kind == "coordinate" and self.coordinate_dim < 1:
            raise ValueError("coordinate queryにはcoordinate_dimが必要です")
        if self.kind == "learned" and (self.coordinate_dim or self.coordinate_scale):
            raise ValueError("learned queryには座標を設定しません")
        if self.coordinate_scale and (
            len(self.coordinate_scale) != self.coordinate_dim
            or any(not 0 < value < float("inf") for value in self.coordinate_scale)
        ):
            raise ValueError("coordinate_scaleは座標ごとの有限の正数です")
        if self.output_transform not in {"identity", "pose3d"}:
            raise ValueError("未知のoutput_transformです")
        if self.output_transform == "pose3d" and self.output_dim != 7:
            raise ValueError("pose3dはXYZ + quaternion xyzwの7次元です")


@dataclass(frozen=True)
class ModelConfig:
    """時刻間で共有するモデル定義。周期カウンターはMemoryStateとして外へ返す。"""

    modalities: dict[str, ModalityConfig]
    queries: dict[str, QueryConfig]
    width: int = 256
    memory_tokens: int = 64
    layers: int = 4
    heads: int = 8
    ff_multiplier: int = 4
    dropout: float = 0.0
    recurrence_period: int = 1
    query_chunk_size: int = 1024

    def __post_init__(self) -> None:
        if (
            min(
                self.width,
                self.memory_tokens,
                self.layers,
                self.heads,
                self.ff_multiplier,
                self.recurrence_period,
                self.query_chunk_size,
            )
            < 1
        ):
            raise ValueError("モデルのサイズ・周期は正数です")
        if self.width % self.heads:
            raise ValueError("widthはheadsの整数倍です")
        if not 0 <= self.dropout < 1:
            raise ValueError("dropoutは0以上1未満です")
        if not self.modalities or not self.queries:
            raise ValueError("観測とqueryを1種類以上定義してください")
        if any(not name.isidentifier() for name in (*self.modalities, *self.queries)):
            raise ValueError("modality/query名はPythonの識別子にしてください")

    @classmethod
    def load(cls, path: Path) -> ModelConfig:
        """YAMLを読み、未知のキーもコンストラクターで拒否する。"""
        data = yaml.safe_load(path.read_text())
        data["modalities"] = {name: ModalityConfig(**value) for name, value in data["modalities"].items()}
        queries = {}
        for name, value in data["queries"].items():
            value = dict(value)
            if "coordinate_scale" in value:
                value["coordinate_scale"] = tuple(value["coordinate_scale"])
            queries[name] = QueryConfig(**value)
        data["queries"] = queries
        return cls(**data)
