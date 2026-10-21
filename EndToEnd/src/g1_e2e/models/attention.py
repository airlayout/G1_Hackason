"""全入力系列を融合する、時間RoPE付きself-attention。"""

from __future__ import annotations

from torch import Tensor, nn
import torch.nn.functional as F

from .position import RotaryTimeEmbedding
from .feedforward import SwiGLU


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
