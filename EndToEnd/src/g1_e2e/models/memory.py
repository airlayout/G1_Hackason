"""生成したMを変更せず蓄積する履歴と、参照時の相対時刻符号。"""

from __future__ import annotations

from dataclasses import dataclass
import math

import torch
from torch import Tensor, nn


@dataclass(frozen=True)
class MemoryState:
    """tokens=[B,H,D]、valid/token_steps=[B,H]、step=[B]。Hは履歴の長さ。"""

    tokens: Tensor
    valid: Tensor
    token_steps: Tensor
    step: Tensor  # 次に処理する観測・制御周期。Mの追加回数とは別。episode開始時を0とする。

    def validate(self, reference: Tensor, context_window: int) -> int:
        """モデルと履歴の形状・device・dtypeを照合し、batch数を返す。"""
        if self.tokens.ndim != 3:
            raise ValueError("state.tokensは[B,H,D]です")
        b, h, d = self.tokens.shape
        if (
            b < 1
            or d != reference.shape[-1]
            or h > context_window
            or self.tokens.device != reference.device
            or self.tokens.dtype != reference.dtype
        ):
            raise ValueError("履歴の幅・context_window・device・dtypeをモデルと揃えてください")
        for name, shape, dtype in [
            ("valid", (b, h), torch.bool),
            ("token_steps", (b, h), torch.long),
            ("step", (b,), torch.long),
        ]:
            value = getattr(self, name)
            if value.shape != shape or value.dtype != dtype or value.device != reference.device:
                raise ValueError(f"state.{name}のshape・dtype・deviceを確認してください")
        return b

    def readout(
        self, *, reference_step: Tensor | None = None, enabled: Tensor | None = None
    ) -> tuple[Tensor, Tensor, Tensor]:
        """内容・valid・経過周期を返す。既定の参照時刻は最後に処理した制御周期。"""
        if reference_step is None:
            if self.tokens.shape[1] == 0:
                raise ValueError("decodeの前にencode_stepでMを生成してください")
            reference_step = self.step - 1
        valid = self.valid if enabled is None else self.valid & enabled[:, None]
        return self.tokens, valid, reference_step[:, None] - self.token_steps

    def detach(self) -> MemoryState:
        """truncated BPTTの境界を学習側で指定する。履歴の内容は保つ。"""
        return MemoryState(self.tokens.detach(), self.valid, self.token_steps, self.step)

    def reset(self, mask: Tensor) -> MemoryState:
        """選択したepisodeの履歴と周期を無効化する。他のepisodeは維持する。"""
        return MemoryState(
            torch.where(mask[:, None, None], 0, self.tokens),
            self.valid & ~mask[:, None],
            torch.where(mask[:, None], 0, self.token_steps),
            torch.where(mask, 0, self.step),
        )

    def append(self, current: Tensor, context_window: int, write_mask: Tensor) -> MemoryState:
        """再帰周期のepisodeだけMを追加する。全episodeの制御周期は進める。"""
        if not write_mask.any():
            return MemoryState(self.tokens, self.valid, self.token_steps, self.step + 1)
        b, n, _ = current.shape
        new_valid = torch.ones((b, n), device=current.device, dtype=torch.bool)
        new_steps = self.step[:, None].expand(-1, n)
        # 有効な履歴は各行で右詰め。休止する行の最古Mを押し出さない。
        written = torch.cat((self.tokens, current), dim=1)[:, -context_window:]
        held = torch.cat((torch.zeros_like(current), self.tokens), dim=1)[:, -context_window:]
        written_valid = torch.cat((self.valid, new_valid), dim=1)[:, -context_window:]
        held_valid = torch.cat((torch.zeros_like(new_valid), self.valid), dim=1)[:, -context_window:]
        written_steps = torch.cat((self.token_steps, new_steps), dim=1)[:, -context_window:]
        held_steps = torch.cat((torch.zeros_like(new_steps), self.token_steps), dim=1)[:, -context_window:]
        return MemoryState(
            torch.where(write_mask[:, None, None], written, held),
            torch.where(write_mask[:, None], written_valid, held_valid),
            torch.where(write_mask[:, None], written_steps, held_steps),
            self.step + 1,
        )


@dataclass(frozen=True)
class MemoryUpdate:
    """今回のMまたはUと、次周期へ渡すM履歴。Uは読み出すときだけ使う。"""

    latents: Tensor  # [B,memory_tokens,D]
    state: MemoryState
    recurrent: Tensor  # bool[B]。今回Mを履歴へ追加したか。

    def validate(self, reference: Tensor, context_window: int) -> int:
        batch_size = self.state.validate(reference, context_window)
        if (
            self.latents.shape != (batch_size, *reference.shape[1:])
            or self.latents.device != reference.device
            or self.latents.dtype != reference.dtype
        ):
            raise ValueError("latentsはモデルと同じdevice・dtypeの[B,memory_tokens,D]です")
        if (
            self.recurrent.shape != (batch_size,)
            or self.recurrent.dtype != torch.bool
            or self.recurrent.device != reference.device
        ):
            raise ValueError("recurrentはモデルと同じdeviceのbool[B]です")
        return batch_size

    def readout(self) -> tuple[Tensor, Tensor, Tensor]:
        """全M履歴と今回のUを返す。履歴にある今回のMは二重に数えない。"""
        tokens, valid, ages = self.state.readout()
        current_valid = (~self.recurrent)[:, None].expand(self.latents.shape[:2])
        return (
            torch.cat((tokens, self.latents), dim=1),
            torch.cat((valid, current_valid), dim=1),
            torch.cat((ages, torch.zeros_like(current_valid, dtype=torch.long)), dim=1),
        )


class HistoryEncoding(nn.Module):
    """履歴を参照するときだけ時間符号を加える。保存されたMは変更しない。"""

    def __init__(self, width: int) -> None:
        super().__init__()
        self.width = width
        self.modality = nn.Parameter(torch.randn(1, 1, width) * 0.02)
        frequencies = torch.exp(-math.log(10000.0) * torch.arange(0, width, 2).float() / width)
        self.register_buffer("frequencies", frequencies)

    def forward(self, tokens: Tensor, valid: Tensor, ages: Tensor) -> tuple[Tensor, Tensor]:
        """MとUに共通の時間符号を加える。無効な内容・時刻は参照させない。"""
        age = torch.where(valid, ages, 0).float()
        phase = age[..., None] * self.frequencies.float()
        temporal = torch.stack((phase.sin(), phase.cos()), dim=-1).flatten(2)[..., : self.width]
        content = torch.where(valid[..., None], tokens, 0)
        context = content + temporal.to(content.dtype) + self.modality
        return torch.where(valid[..., None], context, 0), valid
