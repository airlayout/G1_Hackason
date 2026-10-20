"""センサー別トークンと、パッチ・空間queryを作る純粋なtensor処理。"""

from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Mapping

import torch
from torch import Tensor, nn
import torch.nn.functional as F

from .config import ModalityConfig


@dataclass(frozen=True)
class SensorTokens:
    """features=[B,N,C]。valid=Trueが有効。ageは秒、positionsはセンサー固有座標。"""

    features: Tensor
    valid: Tensor | None = None
    positions: Tensor | None = None
    age_s: Tensor | None = None  # [B]（フレーム単位）または[B,N]


def image_patches(image: Tensor, patch_size: int = 16, *, age_s: Tensor | None = None) -> SensorTokens:
    """[B,C,H,W]の画像を、行優先のパッチと[-1,1]のXY位置へ変換する。"""
    if image.ndim != 4 or not image.is_floating_point() or patch_size < 1:
        raise ValueError("imageには浮動小数点の[B,C,H,W]が必要です")
    b, _, h, w = image.shape
    if h % patch_size or w % patch_size or min(h, w) < patch_size:
        raise ValueError("画像の高さ・幅はpatch_sizeの正の整数倍にしてください")
    features = F.unfold(image, kernel_size=patch_size, stride=patch_size).transpose(1, 2)
    ny, nx = h // patch_size, w // patch_size
    ys = (torch.arange(ny, device=image.device, dtype=image.dtype) + 0.5) * 2 / ny - 1
    xs = (torch.arange(nx, device=image.device, dtype=image.dtype) + 0.5) * 2 / nx - 1
    y, x = torch.meshgrid(ys, xs, indexing="ij")
    positions = torch.stack((x, y), dim=-1).reshape(1, ny * nx, 2).expand(b, -1, -1)
    return SensorTokens(features, positions=positions, age_s=age_s)


def grid_queries(bounds_xyz: Tensor, shape_zyx: tuple[int, int, int], batch_size: int = 1) -> Tensor:
    """格子のセル中心をXYZ queryへ変換する。出力[B,Z*Y*X,3]、Xが最速軸。"""
    if bounds_xyz.shape != (3, 2) or not bounds_xyz.is_floating_point():
        raise ValueError("bounds_xyzは浮動小数点[3,2]の[min,max]です")
    if len(shape_zyx) != 3 or min(shape_zyx) < 1 or batch_size < 1:
        raise ValueError("格子サイズとbatch_sizeは正数です")
    if not torch.isfinite(bounds_xyz).all() or not (bounds_xyz[:, 1] > bounds_xyz[:, 0]).all():
        raise ValueError("格子境界には有限なmin < maxが必要です")
    axes = []
    for bound, count in zip(bounds_xyz, reversed(shape_zyx)):
        centers = (torch.arange(count, device=bound.device, dtype=bound.dtype) + 0.5) / count
        axes.append(bound[0] + centers * (bound[1] - bound[0]))
    z, y, x = torch.meshgrid(axes[2], axes[1], axes[0], indexing="ij")
    return torch.stack((x, y, z), dim=-1).reshape(1, -1, 3).expand(batch_size, -1, -1)


class ModalityEncoder(nn.Module):
    """内容・位置・age・modalityを共通幅へ写す。センサー列間のself-attentionは行わない。"""

    def __init__(self, config: ModalityConfig, width: int) -> None:
        super().__init__()
        self.config = config
        self.features = nn.Linear(config.feature_dim, width)
        self.coordinates = nn.Linear(config.position_dim, width) if config.position_dim else None
        self.age = nn.Sequential(nn.Linear(1, width), nn.SiLU(), nn.Linear(width, width))
        self.modality = nn.Parameter(torch.randn(1, 1, width) * 0.02)
        self.slot_positions = (
            nn.Parameter(torch.randn(1, config.max_tokens, width) * 0.02)
            if config.learned_positions
            else None
        )
        self.norm = nn.LayerNorm(width)

    def forward(self, stream: SensorTokens) -> tuple[Tensor, Tensor]:
        """無効トークンのNaNを投影前に取り除き、attention用valid maskを返す。"""
        x = stream.features
        if x.ndim != 3 or x.shape[-1] != self.config.feature_dim or not x.is_floating_point():
            raise ValueError("featuresの[B,N,C]・浮動小数点型を確認してください")
        b, n, _ = x.shape
        if n > self.config.max_tokens:
            raise ValueError(f"観測トークン数{n}が上限{self.config.max_tokens}を超えています")
        valid = (
            stream.valid
            if stream.valid is not None
            else torch.ones((b, n), device=x.device, dtype=torch.bool)
        )
        if valid.shape != (b, n) or valid.dtype != torch.bool or valid.device != x.device:
            raise ValueError("validはfeaturesと同じdeviceのbool[B,N]です")
        result = self.features(torch.where(valid[..., None], x, 0)) + self.modality
        if self.coordinates is not None:
            pos = stream.positions
            if pos is None or pos.shape != (b, n, self.config.position_dim):
                raise ValueError("このmodalityにはpositions=[B,N,position_dim]が必要です")
            result = result + self.coordinates(torch.where(valid[..., None], pos, 0))
        if self.slot_positions is not None:
            result = result + self.slot_positions[:, :n]
        age = stream.age_s
        if age is None:
            age = x.new_zeros(b, n)
        elif age.shape == (b,):
            age = age[:, None].expand(-1, n)
        if age.shape != (b, n):
            raise ValueError("age_sは[B]または[B,N]です")
        age = torch.where(valid, age, 0).clamp_min(0).log1p()
        return self.norm(result + self.age(age[..., None])), valid


class ObservationEncoder(nn.Module):
    """存在するセンサーだけを取り込み、全欠損にも対応するnull tokenを添える。"""

    def __init__(self, configs: dict[str, ModalityConfig], width: int) -> None:
        super().__init__()
        self.modalities = nn.ModuleDict({name: ModalityEncoder(cfg, width) for name, cfg in configs.items()})
        self.null = nn.Parameter(torch.randn(1, 1, width) * 0.02)

    def forward(self, observations: Mapping[str, SensorTokens], batch_size: int) -> tuple[Tensor, Tensor]:
        """モデルの設定順で連結し、attentionから参照可能なトークンを返す。"""
        unknown = set(observations) - set(self.modalities)
        if unknown:
            raise ValueError(f"未定義のmodality: {sorted(unknown)}")
        tokens = [self.null.expand(batch_size, -1, -1)]
        masks = [torch.ones((batch_size, 1), dtype=torch.bool, device=self.null.device)]
        for name, encoder in self.modalities.items():
            if name in observations:
                value, valid = encoder(observations[name])
                if value.shape[0] != batch_size or value.device != self.null.device:
                    raise ValueError("全modalityのbatch・deviceをモデルと揃えてください")
                tokens.append(value)
                masks.append(valid)
        return torch.cat(tokens, dim=1), torch.cat(masks, dim=1)
