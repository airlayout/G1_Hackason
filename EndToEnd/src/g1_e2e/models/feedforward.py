"""Transformerとquery読出しで共用する、biasなしのSwiGLU FFN。"""

from torch import Tensor, nn
import torch.nn.functional as F


class SwiGLU(nn.Module):
    """SiLU(gate) ⊙ upをdownで元の幅へ戻す。gate/upは一括投影する。"""

    def __init__(self, width: int, hidden_dim: int, dropout: float) -> None:
        super().__init__()
        self.hidden_dim = hidden_dim
        self.gate_up = nn.Linear(width, 2 * hidden_dim, bias=False)
        self.down = nn.Linear(hidden_dim, width, bias=False)
        self.dropout = nn.Dropout(dropout)

    def forward(self, tokens: Tensor) -> Tensor:
        gate, up = self.gate_up(tokens).chunk(2, dim=-1)
        return self.dropout(self.down(F.silu(gate) * up))
