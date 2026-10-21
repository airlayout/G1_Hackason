"""再帰メモリ・観測encoder・queryを構成する、Isaacから独立した設定。"""

from __future__ import annotations

from dataclasses import dataclass
import math
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
    """Mと結合するquery、およびtask固有の出力の意味・範囲。"""

    output_dim: int
    kind: Literal["learned", "coordinate"] = "learned"
    num_queries: int = 1
    coordinate_dim: int = 0
    coordinate_scale: tuple[float, ...] = ()
    fourier_bands: int = 6
    output_transform: Literal["identity", "occupancy", "bounded", "pose3d"] = "identity"
    output_min: tuple[float, ...] = ()
    output_max: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        if self.output_dim < 1 or self.num_queries < 1 or self.fourier_bands < 0:
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
        if self.output_transform not in {"identity", "occupancy", "bounded", "pose3d"}:
            raise ValueError("未知のoutput_transformです")
        if self.output_transform == "occupancy" and (
            self.kind != "coordinate" or self.coordinate_dim != 3 or self.output_dim != 2
        ):
            raise ValueError("occupancyはXYZ queryから占有・観測済みの2成分を返します")
        if self.output_transform == "pose3d" and (
            self.output_dim != 7 or self.kind != "coordinate" or self.coordinate_dim != 1
        ):
            raise ValueError("pose3dは未来時刻queryからXYZ + quaternion xyzwの7次元を返します")
        if self.output_transform == "bounded":
            if (
                len(self.output_min) != self.output_dim
                or len(self.output_max) != self.output_dim
                or any(
                    not math.isfinite(low) or not math.isfinite(high) or low >= high
                    for low, high in zip(self.output_min, self.output_max)
                )
            ):
                raise ValueError("boundedには出力次元ごとの有限なoutput_min < output_maxが必要です")
        elif self.output_min or self.output_max:
            raise ValueError("出力範囲を設定できるのはboundedだけです")


@dataclass(frozen=True)
class ModelConfig:
    """context_windowはM履歴のトークン上限。現在の観測トークンは別枠で取り込む。"""

    modalities: dict[str, ModalityConfig]
    queries: dict[str, QueryConfig]
    width: int = 256
    memory_tokens: int = 1
    layers: int = 36
    heads: int = 8
    ff_multiplier: int = 4
    ff_multiple_of: int = 256
    dropout: float = 0.0
    recurrence_period: int = 2
    context_window: int = 1024
    query_chunk_size: int = 1024
    control_dt_s: float = 0.02
    rope_base: float = 10000.0

    def __post_init__(self) -> None:
        if (
            min(
                self.width,
                self.memory_tokens,
                self.layers,
                self.heads,
                self.ff_multiplier,
                self.ff_multiple_of,
                self.recurrence_period,
                self.context_window,
                self.query_chunk_size,
            )
            < 1
        ):
            raise ValueError("モデルのサイズ・周期は正数です")
        if self.width % self.heads:
            raise ValueError("widthはheadsの整数倍です")
        if (self.width // self.heads) % 2:
            raise ValueError("時間RoPEのhead次元は偶数です")
        if not math.isfinite(self.control_dt_s) or self.control_dt_s <= 0:
            raise ValueError("control_dt_sは有限の正数です")
        if not math.isfinite(self.rope_base) or self.rope_base <= 1:
            raise ValueError("rope_baseは1より大きい有限値です")
        if self.context_window < self.memory_tokens:
            raise ValueError("context_windowは1周期のmemory_tokens以上にしてください")
        if not 0 <= self.dropout < 1:
            raise ValueError("dropoutは0以上1未満です")
        if not self.modalities or not self.queries:
            raise ValueError("観測とqueryを1種類以上定義してください")
        if any(not name.isidentifier() for name in (*self.modalities, *self.queries)):
            raise ValueError("modality/query名はPythonの識別子にしてください")

    @property
    def ff_hidden_dim(self) -> int:
        """通常FFNの幅を2/3にし、指定倍数へ切り上げる。既定は8D/3→256単位。"""
        hidden_dim = 2 * self.width * self.ff_multiplier // 3
        return self.ff_multiple_of * ((hidden_dim + self.ff_multiple_of - 1) // self.ff_multiple_of)

    @classmethod
    def load(cls, path: Path) -> ModelConfig:
        """YAMLを読み、未知のキーもコンストラクターで拒否する。"""
        return cls.from_dict(yaml.safe_load(path.read_text()))

    @classmethod
    def from_dict(cls, values: dict) -> ModelConfig:
        """YAMLとcheckpointで同じ構成検証を使う。渡されたdictは書き換えない。"""
        data = dict(values)
        data["modalities"] = {name: ModalityConfig(**value) for name, value in data["modalities"].items()}
        queries = {}
        for name, value in data["queries"].items():
            value = dict(value)
            for key in ["coordinate_scale", "output_min", "output_max"]:
                if key in value:
                    value[key] = tuple(value[key])
            queries[name] = QueryConfig(**value)
        data["queries"] = queries
        return cls(**data)
