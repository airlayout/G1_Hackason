"""観測・過去Mへのcross-attentionと、今回生成する小さいM内のself-attention。"""

from __future__ import annotations

from torch import Tensor, nn
import torch.nn.functional as F


class Attention(nn.Module):
    """SDPAを使用するattention。valid=Trueのkeyだけを参照する。"""

    def __init__(self, width: int, heads: int, dropout: float) -> None:
        super().__init__()
        self.heads, self.head_width, self.dropout = heads, width // heads, dropout
        self.q = nn.Linear(width, width)
        self.k = nn.Linear(width, width)
        self.v = nn.Linear(width, width)
        self.out = nn.Linear(width, width)

    def forward(self, queries: Tensor, context: Tensor, valid: Tensor | None = None) -> Tensor:
        """query同士を混ぜず、各queryからcontextへattentionする。"""

        def split(value: Tensor) -> Tensor:
            return value.reshape(value.shape[0], value.shape[1], self.heads, self.head_width).transpose(1, 2)

        q, k, v = split(self.q(queries)), split(self.k(context)), split(self.v(context))
        mask = valid[:, None, None, :] if valid is not None else None
        result = F.scaled_dot_product_attention(
            q, k, v, attn_mask=mask, dropout_p=self.dropout if self.training else 0.0
        )
        result = result.transpose(1, 2).contiguous().flatten(2)
        return self.out(result)


class CrossRead(nn.Module):
    """PreNorm cross-attention + tokenごとのFFN。空間query間のself-attentionは持たない。"""

    def __init__(self, width: int, heads: int, multiplier: int, dropout: float) -> None:
        super().__init__()
        self.query_norm, self.context_norm = nn.LayerNorm(width), nn.LayerNorm(width)
        self.attention = Attention(width, heads, dropout)
        self.ff_norm = nn.LayerNorm(width)
        self.ff = nn.Sequential(
            nn.Linear(width, width * multiplier),
            nn.GELU(),
            nn.Linear(width * multiplier, width),
            nn.Dropout(dropout),
        )

    def forward(self, queries: Tensor, context: Tensor, valid: Tensor | None = None) -> Tensor:
        """query単位で独立な読出しを行い、Neural Fieldの分割評価を可能にする。"""
        result = queries + self.attention(self.query_norm(queries), self.context_norm(context), valid)
        return result + self.ff(self.ff_norm(result))


class MemoryBlock(nn.Module):
    """観測と履歴から情報を取り込み、今回の出力トークン内で情報を交換する1層。"""

    def __init__(self, width: int, heads: int, multiplier: int, dropout: float) -> None:
        super().__init__()
        self.context_read = CrossRead(width, heads, multiplier, dropout)
        self.self_norm = nn.LayerNorm(width)
        self.self_attention = Attention(width, heads, dropout)
        self.ff_norm = nn.LayerNorm(width)
        self.ff = nn.Sequential(
            nn.Linear(width, width * multiplier),
            nn.GELU(),
            nn.Linear(width * multiplier, width),
            nn.Dropout(dropout),
        )

    def forward(self, memory: Tensor, context: Tensor, valid: Tensor) -> Tensor:
        """観測・履歴全体のN×N self-attentionは計算しない。"""
        memory = self.context_read(memory, context, valid)
        normalized = self.self_norm(memory)
        memory = memory + self.self_attention(normalized, normalized)
        return memory + self.ff(self.ff_norm(memory))
