"""観測→再帰潜在メモリ→任意query。Isaac・G1の関節制御から独立したNN。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import torch
from torch import Tensor, nn

from .attention import MemoryBlock
from .config import ModelConfig
from .queries import QueryDecoder, QueryRequest
from .tokens import ObservationEncoder, SensorTokens


@dataclass(frozen=True)
class MemoryState:
    """時刻間で受け渡す潜在[B,M,D]と、環境ごとの処理済み周期数[B]。"""

    tokens: Tensor
    step: Tensor

    def detach(self) -> MemoryState:
        """truncated BPTTの区切りを学習側で指定する。通常forwardでは切断しない。"""
        return MemoryState(self.tokens.detach(), self.step.detach())


@dataclass(frozen=True)
class MemoryUpdate:
    """latentsは今回の読出し元、stateは次回へ渡す長期メモリ。"""

    latents: Tensor
    state: MemoryState
    recurrent: Tensor  # bool[B]。今回、前回メモリを入力に使ったか。


@dataclass(frozen=True)
class ModelOutput:
    """query別予測と、明示的な再帰状態。"""

    predictions: dict[str, Tensor]
    memory: MemoryUpdate


class RecurrentPerceiver(nn.Module):
    """小さなメモリをqueryとして観測を取り込み、固定・空間・時間queryで読み出す。"""

    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        self.config = config
        self.seed = nn.Parameter(torch.randn(1, config.memory_tokens, config.width) * 0.02)
        self.observations = ObservationEncoder(config.modalities, config.width)
        self.blocks = nn.ModuleList(
            [
                MemoryBlock(config.width, config.heads, config.ff_multiplier, config.dropout)
                for _ in range(config.layers)
            ]
        )
        self.decoder = QueryDecoder(config)

    def initial_state(self, batch_size: int) -> MemoryState:
        """学習可能な初期メモリ。device/dtypeはモデルに合わせる。"""
        if batch_size < 1:
            raise ValueError("batch_sizeは正数です")
        tokens = self.seed.expand(batch_size, -1, -1).clone()
        return MemoryState(tokens, torch.zeros(batch_size, dtype=torch.long, device=self.seed.device))

    def _validate_state(self, state: MemoryState) -> int:
        if state.tokens.ndim != 3:
            raise ValueError("state.tokensは[B,M,D]です")
        batch_size = state.tokens.shape[0]
        expected = (batch_size, self.config.memory_tokens, self.config.width)
        if (
            batch_size < 1
            or state.tokens.shape != expected
            or state.tokens.device != self.seed.device
            or state.tokens.dtype != self.seed.dtype
        ):
            raise ValueError("state.tokensの[B,M,D]・device・dtypeをモデルと揃えてください")
        if (
            state.step.shape != (batch_size,)
            or state.step.dtype != torch.long
            or state.step.device != self.seed.device
        ):
            raise ValueError("state.stepはモデルと同じdeviceのlong[B]です")
        return batch_size

    def encode_step(
        self,
        observations: Mapping[str, SensorTokens],
        state: MemoryState | None = None,
        *,
        reset_mask: Tensor | None = None,
    ) -> MemoryUpdate:
        """メモリを1周期進める。センサー周期合わせ・正規化は入力アダプターの責務。"""
        if state is None:
            if not observations:
                raise ValueError("観測が空ならbatch_sizeを指定したinitial_stateを渡してください")
            first = next(iter(observations.values())).features
            if first.ndim != 3:
                raise ValueError("featuresは[B,N,C]です")
            state = self.initial_state(first.shape[0])
        batch_size = self._validate_state(state)
        old, step = state.tokens, state.step
        seed = self.seed.expand(batch_size, -1, -1)
        if reset_mask is not None:
            if (
                reset_mask.shape != (batch_size,)
                or reset_mask.dtype != torch.bool
                or reset_mask.device != self.seed.device
            ):
                raise ValueError("reset_maskはモデルと同じdeviceのbool[B]です")
            old = torch.where(reset_mask[:, None, None], seed, old)
            step = torch.where(reset_mask, 0, step)
        tokens, valid = self.observations(observations, batch_size)
        recurrent = step.remainder(self.config.recurrence_period) == 0
        # 観測のみの周期では、前回メモリをqueryにもresidualにも流さない。
        current = torch.where(recurrent[:, None, None], old, seed)
        for block in self.blocks:
            current = block(current, tokens, valid)
        # autocastで生じるdtype差を再帰状態に持ち越さない。
        current = current.to(self.seed.dtype)
        persistent = torch.where(recurrent[:, None, None], current, old)
        return MemoryUpdate(current, MemoryState(persistent, step + 1), recurrent)

    def decode(
        self,
        latents: Tensor,
        queries: Mapping[str, QueryRequest] | None = None,
        *,
        chunk_size: int | None = None,
    ) -> dict[str, Tensor]:
        """同じ時刻のメモリから、必要なqueryだけ読み出す。再帰周期は進めない。"""
        if (
            latents.ndim != 3
            or latents.shape[1:] != (self.config.memory_tokens, self.config.width)
            or latents.device != self.seed.device
            or latents.dtype != self.seed.dtype
        ):
            raise ValueError("latentsの[B,M,D]・device・dtypeをモデルと揃えてください")
        return self.decoder(latents, queries, chunk_size=chunk_size)

    def forward(
        self,
        observations: Mapping[str, SensorTokens],
        state: MemoryState | None = None,
        queries: Mapping[str, QueryRequest] | None = None,
        *,
        reset_mask: Tensor | None = None,
    ) -> ModelOutput:
        memory = self.encode_step(observations, state, reset_mask=reset_mask)
        return ModelOutput(self.decode(memory.latents, queries), memory)
