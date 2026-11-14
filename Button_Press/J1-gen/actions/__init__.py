"""G1 の腕の定型動作（敬礼・指さしなど）。関節角の列で書き、common/arm の腕の指令部分（rt/arm_sdk）で送る。"""

from .poses import ACTIONS, Action, Step

__all__ = ["ACTIONS", "Action", "Step"]
