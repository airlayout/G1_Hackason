"""観測・M履歴の単一系列をself-attentionし、最新M位置から今回のMを生成する。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import torch
from torch import Tensor, nn

from .attention import TransformerBlock
from .config import ModelConfig
from .memory import MemoryState, MemoryUpdate
from .queries import QueryDecoder, QueryRequest
from .tokens import ObservationEncoder, SensorTokens


@dataclass(frozen=True)
class ModelOutput:
    """今回のM、そのMだけから読んだquery別予測、次周期へ渡す状態。"""

    predictions: dict[str, Tensor]
    memory: MemoryUpdate

    @property
    def latents(self) -> Tensor:
        return self.memory.latents


class RecurrentPerceiver(nn.Module):
    """全周期で履歴と観測を融合する。履歴追加だけをrecurrence_periodで間引く。"""

    def __init__(self, config: ModelConfig) -> None:
        super().__init__()
        self.config = config
        self.seed = nn.Parameter(torch.randn(1, config.memory_tokens, config.width) * 0.02)
        self.memory_type = nn.Parameter(torch.randn(1, 1, config.width) * 0.02)
        self.observations = ObservationEncoder(config.modalities, config.width)
        self.blocks = nn.ModuleList(
            [
                TransformerBlock(
                    config.width, config.heads, config.ff_hidden_dim, config.dropout, config.rope_base
                )
                for _ in range(config.layers)
            ]
        )
        self.output_norm = nn.LayerNorm(config.width)
        self.decoder = QueryDecoder(config)
        self.gradient_checkpointing = False

    def initial_state(self, batch_size: int) -> MemoryState:
        """履歴は空から開始する。seedは履歴が空のepisodeだけで初期Mとして使う。"""
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
        """全系列を再計算し、最新の保存M位置を取り出す。保存周期以外も今回のMを返す。"""
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
        observation_tokens, observation_valid, observation_ages = self.observations(observations, batch_size)
        recurrent = state.step.remainder(self.config.recurrence_period) == 0
        # seedと保存Mは同じ系列に置く。履歴があるepisodeのseedは無効にする。
        seed_valid = (~state.valid.any(dim=1))[:, None].expand(batch_size, self.config.memory_tokens)
        memory_valid = torch.cat((seed_valid, state.valid), dim=1)
        memory_tokens = torch.cat((self.seed.expand(batch_size, -1, -1), state.tokens), dim=1)
        memory_tokens = torch.where(memory_valid[..., None], memory_tokens + self.memory_type, 0)
        memory_times = torch.cat(
            (
                self.seed.new_zeros(batch_size, self.config.memory_tokens),
                (state.token_steps - state.step[:, None]).float(),
            ),
            dim=1,
        )
        memory_times = torch.where(memory_valid, memory_times, 0)
        # 現在を原点にした共通の制御周期単位。点ごとの非同期取得時刻も保持する。
        times = torch.cat((memory_times, -observation_ages / self.config.control_dt_s), dim=1)
        sequence = torch.cat((memory_tokens, observation_tokens), dim=1)
        valid = torch.cat((memory_valid, observation_valid), dim=1)
        for block in self.blocks:
            if self.training and self.gradient_checkpointing:
                from torch.utils.checkpoint import checkpoint

                sequence = checkpoint(block, sequence, valid, times, use_reentrant=False)
            else:
                sequence = block(sequence, valid, times)
        # 過去Mの各出力で保存済み履歴を書き換えず、最新M位置の出力だけを採用する。
        indices = torch.arange(memory_valid.shape[1], device=self.seed.device).expand(batch_size, -1)
        latest = torch.where(memory_valid, indices, -1).topk(self.config.memory_tokens, dim=1).values.flip(1)
        current = sequence.gather(1, latest[..., None].expand(-1, -1, self.config.width))
        current = self.output_norm(current).to(self.seed.dtype)
        return MemoryUpdate(current, state.append(current, self.config.context_window, recurrent), recurrent)

    def decode(
        self,
        memory: MemoryUpdate | Tensor,
        queries: Mapping[str, QueryRequest] | None = None,
        *,
        chunk_size: int | None = None,
        raw: bool = False,
    ) -> dict[str, Tensor]:
        """今回のMだけを読む。raw=Trueなら各headの出力変換前（場はlogits）を返す。"""
        if isinstance(memory, MemoryUpdate):
            memory.validate(self.seed, self.config.context_window)
            current = memory.latents
        elif isinstance(memory, Tensor):
            current = memory
            if (
                current.ndim != 3
                or current.shape[0] < 1
                or current.shape[1:] != self.seed.shape[1:]
                or current.device != self.seed.device
                or current.dtype != self.seed.dtype
            ):
                raise ValueError("今回のMはモデルと同じdevice・dtypeの[B,memory_tokens,D]です")
        else:
            raise TypeError("decodeには今回のMemoryUpdateまたはMのTensorを渡してください")
        return self.decoder(current, queries, chunk_size=chunk_size, raw=raw)

    def forward(
        self,
        observations: Mapping[str, SensorTokens],
        state: MemoryState | None = None,
        queries: Mapping[str, QueryRequest] | None = None,
        *,
        reset_mask: Tensor | None = None,
        raw: bool = False,
    ) -> ModelOutput:
        memory = self.encode_step(observations, state, reset_mask=reset_mask)
        return ModelOutput(self.decode(memory, queries, raw=raw), memory)
