"""観測とM履歴から表現を生成し、M履歴と今回の表現をqueryで読むNN。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import torch
from torch import Tensor, nn

from .attention import MemoryBlock
from .config import ModelConfig
from .memory import HistoryEncoding, MemoryState, MemoryUpdate
from .queries import QueryDecoder, QueryRequest
from .tokens import ObservationEncoder, SensorTokens


@dataclass(frozen=True)
class ModelOutput:
    """M履歴と今回の表現から読んだquery別予測と、次周期へ渡す状態。"""

    predictions: dict[str, Tensor]
    memory: MemoryUpdate


class RecurrentPerceiver(nn.Module):
    """H_t=[M_0,...,M_t]を保持し、更新時と読出し時の両方から参照する。"""

    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        self.config = config
        self.seed = nn.Parameter(torch.randn(1, config.memory_tokens, config.width) * 0.02)
        self.observations = ObservationEncoder(config.modalities, config.width)
        self.history = HistoryEncoding(config.width)
        self.blocks = nn.ModuleList(
            [
                MemoryBlock(config.width, config.heads, config.ff_multiplier, config.dropout)
                for _ in range(config.layers)
            ]
        )
        self.decoder = QueryDecoder(config)

    def initial_state(self, batch_size: int) -> MemoryState:
        """履歴は空から開始する。seedは出力queryであり、履歴へは数えない。"""
        if batch_size < 1:
            raise ValueError("batch_sizeは正数です")
        return MemoryState(
            tokens=self.seed.new_empty(batch_size, 0, self.config.width),
            valid=torch.empty((batch_size, 0), dtype=torch.bool, device=self.seed.device),
            token_steps=torch.empty((batch_size, 0), dtype=torch.long, device=self.seed.device),
            step=torch.zeros(batch_size, dtype=torch.long, device=self.seed.device),
        )

    def encode_step(
        self,
        observations: Mapping[str, SensorTokens],
        state: MemoryState | None = None,
        *,
        reset_mask: Tensor | None = None,
    ) -> MemoryUpdate:
        """再帰周期だけMを追加する。観測のみの周期は一時表現を作り履歴を保持する。"""
        if state is None:
            if not observations:
                raise ValueError("観測が空ならbatch_sizeを指定したinitial_stateを渡してください")
            first = next(iter(observations.values())).features
            if first.ndim != 3:
                raise ValueError("featuresは[B,N,C]です")
            state = self.initial_state(first.shape[0])
        if not isinstance(state, MemoryState):
            raise TypeError("履歴を含むMemoryStateを渡してください")
        batch_size = state.validate(self.seed, self.config.context_window)
        if reset_mask is not None:
            if (
                reset_mask.shape != (batch_size,)
                or reset_mask.dtype != torch.bool
                or reset_mask.device != self.seed.device
            ):
                raise ValueError("reset_maskはモデルと同じdeviceのbool[B]です")
            state = state.reset(reset_mask)
        observation_tokens, observation_valid = self.observations(observations, batch_size)
        recurrent = state.step.remainder(self.config.recurrence_period) == 0
        history_tokens, history_valid = self.history(
            *state.readout(reference_step=state.step, enabled=recurrent)
        )
        context = torch.cat((history_tokens, observation_tokens), dim=1)
        valid = torch.cat((history_valid, observation_valid), dim=1)
        # 過去Mをquery/residualへコピーせず、履歴はK/Vからだけ参照する。
        current = self.seed.expand(batch_size, -1, -1)
        for block in self.blocks:
            current = block(current, context, valid)
        current = current.to(self.seed.dtype)
        return MemoryUpdate(current, state.append(current, self.config.context_window, recurrent), recurrent)

    def decode(
        self,
        memory: MemoryUpdate | MemoryState,
        queries: Mapping[str, QueryRequest] | None = None,
        *,
        chunk_size: int | None = None,
    ) -> dict[str, Tensor]:
        """MemoryUpdateではM履歴と今回のUを、MemoryStateではM履歴だけを読む。"""
        if not isinstance(memory, (MemoryState, MemoryUpdate)):
            raise TypeError("MemoryUpdateまたはMemoryStateを渡してください")
        memory.validate(self.seed, self.config.context_window)
        context, valid = self.history(*memory.readout())
        return self.decoder(context, queries, valid=valid, chunk_size=chunk_size)

    def forward(
        self,
        observations: Mapping[str, SensorTokens],
        state: MemoryState | None = None,
        queries: Mapping[str, QueryRequest] | None = None,
        *,
        reset_mask: Tensor | None = None,
    ) -> ModelOutput:
        memory = self.encode_step(observations, state, reset_mask=reset_mask)
        return ModelOutput(self.decode(memory, queries), memory)
