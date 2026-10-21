"""全入力系列の時間RoPE付きself-attentionと、今回のMを読むquery用attention。"""

from __future__ import annotations

from torch import Tensor, nn
import torch.nn.functional as F

from .position import RotaryTimeEmbedding
from .feedforward import SwiGLU


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

    def __init__(self, width: int, heads: int, hidden_dim: int, dropout: float) -> None:
        super().__init__()
        self.query_norm, self.context_norm = nn.LayerNorm(width), nn.LayerNorm(width)
        self.attention = Attention(width, heads, dropout)
        self.ff_norm = nn.LayerNorm(width)
        self.ff = SwiGLU(width, hidden_dim, dropout)

    def forward(self, queries: Tensor, context: Tensor, valid: Tensor | None = None) -> Tensor:
        """query単位で独立な読出しを行い、Neural Fieldの分割評価を可能にする。"""
        result = queries + self.attention(self.query_norm(queries), self.context_norm(context), valid)
        return result + self.ff(self.ff_norm(result))


class RotarySelfAttention(nn.Module):
    """観測と履歴を区別せず、全有効トークン間の双方向attentionを計算する。"""

    def __init__(self, width: int, heads: int, dropout: float, rope_base: float) -> None:
        super().__init__()
        self.heads, self.head_width, self.dropout = heads, width // heads, dropout
        self.qkv = nn.Linear(width, 3 * width)
        self.out = nn.Linear(width, width)
        self.rope = RotaryTimeEmbedding(self.head_width, rope_base)

    def forward(self, tokens: Tensor, valid: Tensor, times: Tensor) -> Tensor:
        b, n, _ = tokens.shape
        q, k, v = self.qkv(tokens).reshape(b, n, 3, self.heads, self.head_width).permute(2, 0, 3, 1, 4)
        q, k = self.rope(q, times), self.rope(k, times)
        result = F.scaled_dot_product_attention(
            q,
            k,
            v,
            attn_mask=valid[:, None, None, :],
            dropout_p=self.dropout if self.training else 0.0,
        )
        return self.out(result.transpose(1, 2).contiguous().flatten(2))


class TransformerBlock(nn.Module):
    """PreNorm self-attention + SwiGLU。履歴を含む系列全体を各層で更新する。"""

    def __init__(self, width: int, heads: int, hidden_dim: int, dropout: float, rope_base: float) -> None:
        super().__init__()
        self.attention_norm = nn.LayerNorm(width)
        self.attention = RotarySelfAttention(width, heads, dropout, rope_base)
        self.ff_norm = nn.LayerNorm(width)
        self.ff = SwiGLU(width, hidden_dim, dropout)

    def forward(self, tokens: Tensor, valid: Tensor, times: Tensor) -> Tensor:
        tokens = tokens + self.attention(self.attention_norm(tokens), valid, times)
        return tokens + self.ff(self.ff_norm(tokens))
