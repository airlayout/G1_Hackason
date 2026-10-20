"""腕を動かすときの安全チェック（純粋な計算だけ。通信はしない）。

HANDOFF 5章の安全要件のうち、関節リミット・1ステップの移動量の上限・作業空間の箱を扱う。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np


class UnsafeTargetError(ValueError):
    """安全チェックに通らない目標。送信せずに拒否する。"""


def check_finite(q: np.ndarray, what: str = "目標") -> None:
    """NaN / inf が入っていたら拒否する。"""
    if not np.all(np.isfinite(q)):
        raise UnsafeTargetError(f"{what}に NaN / inf が入っている: {q}")


def check_joint_limits(
    q: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    margin: float,
    names: Sequence[str],
) -> None:
    """関節リミットから margin 以上内側にあるかを確認する。外なら拒否する（黙ってクリップしない）。"""
    bad = []
    for i, (v, lo, hi) in enumerate(zip(q, lower, upper)):
        if v < lo + margin or v > hi - margin:
            bad.append(f"{names[i]}={v:+.3f}（可動範囲 {lo:+.3f}〜{hi:+.3f}、余裕 {margin}）")
    if bad:
        raise UnsafeTargetError("関節リミットに近すぎる／外れている: " + ", ".join(bad))


def rate_limit(q_now: np.ndarray, q_target: np.ndarray, max_step: float) -> np.ndarray:
    """1ステップあたりの各関節の移動量を max_step [rad] 以下に抑えた次の指令値を返す。"""
    step = np.clip(q_target - q_now, -max_step, max_step)
    return q_now + step


def smoothstep(s: float | np.ndarray) -> float | np.ndarray:
    """s = 3t² − 2t³。t=0 と t=1 で速度が 0 になる補間の重み。"""
    t = np.clip(s, 0.0, 1.0)
    return 3.0 * t**2 - 2.0 * t**3


def interpolate_joint(q0: np.ndarray, q1: np.ndarray, duration: float, dt: float) -> np.ndarray:
    """q0 → q1 を duration 秒かけて smoothstep で補間した点列（q0 は含まず、q1 を含む）。"""
    n = max(1, int(np.ceil(duration / dt)))
    s = smoothstep(np.arange(1, n + 1) / n)
    return q0[None, :] + np.asarray(s)[:, None] * (q1 - q0)[None, :]


@dataclass
class WorkspaceBox:
    """手先が入ってよい箱（pelvis 座標、単位 m）。外に出る目標は拒否する。"""

    lower: np.ndarray
    upper: np.ndarray

    @classmethod
    def from_config(cls, cfg: dict) -> "WorkspaceBox":
        return cls(np.asarray(cfg["min"], dtype=float), np.asarray(cfg["max"], dtype=float))

    def check(self, p: np.ndarray, what: str = "手先") -> None:
        if np.any(p < self.lower) or np.any(p > self.upper):
            raise UnsafeTargetError(
                f"{what}が作業空間の外: {np.round(p, 3)}（箱 {self.lower} 〜 {self.upper}）"
            )
