"""頭カメラで測ったゴミ箱の置き方から、立つ位置まで歩く量を求める。sim / real 共通（歩行の SDK には依存しない）。

やり方は「測る → 少し歩く → 止まって測り直す」の繰り返し（approach_step を 1 回ずつ呼ぶ）:
- 止まって測るので、画像がぶれず、計算の遅れ（1 回 0.5 秒ほど）も問題にならない
- 毎回測り直すので、歩行のずれ（進み過ぎ・足りない・曲がる）があっても直せる
- 1 回に歩く量・速さに上限を付け、ゴミ箱が見えなくなったら止める

立つ位置: ゴミ箱の置き方が、設定の置き方（configs/bin_scene.yaml の bin。押す動作を確かめた位置）になる位置。
2D の姿勢で書くと、測った置き方 T（pelvis 座標でのゴミ箱）、目標の置き方 T* として、
ロボットが動くべき量 M（今の pelvis 座標で表した、動いたあとの pelvis）は M = T · T*⁻¹:
  向き dyaw = yaw − yaw*、位置 d = c − R(dyaw) · c*
（M だけ動くと、ゴミ箱は新しい pelvis 座標で M⁻¹ · T = T* に見える）
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from bin_geometry import BinPose


@dataclass
class ApproachConfig:
    tol_xy_m: float = 0.02         # 立つ位置とのずれがこれ以内で終わる（前後・左右それぞれ）
    tol_yaw_deg: float = 3.0       # 向きのずれがこれ以内で終わる
    max_step_m: float = 0.15       # 1 回に歩く距離の上限
    max_step_yaw_deg: float = 15.0  # 1 回に回る角度の上限
    max_vx: float = 0.15           # 速さの上限 [m/s]（Navigation の loco_driver の仮の上限 0.30 より遅く）
    max_vy: float = 0.10
    max_vyaw: float = 0.30         # [rad/s]
    max_reverse_vx: float = 0.05   # 後ろへ下がる速さの上限（後ろは死角なので、行き過ぎを直す程度だけ）
    min_step_s: float = 0.5        # 1 回に歩く時間の下限（短すぎると歩き出さない）
    max_steps: int = 20            # これだけ繰り返しても着かなければ止める
    settle_s: float = 1.0          # 歩いたあと、止まって落ち着くのを待つ時間

    @staticmethod
    def from_dict(d: dict[str, Any] | None) -> "ApproachConfig":
        c = ApproachConfig()
        for k, v in (d or {}).items():
            if hasattr(c, k):
                setattr(c, k, type(getattr(c, k))(v))
        return c


@dataclass
class ApproachStep:
    """1 回分の歩く指令。done なら歩かずに終わる。"""
    done: bool
    error: np.ndarray        # 立つ位置とのずれ (dx, dy, dyaw [rad])（今の pelvis 座標）
    vx: float = 0.0          # 速さの指令 [m/s, m/s, rad/s]（LocoClient.Move と同じ）
    vy: float = 0.0
    vyaw: float = 0.0
    duration_s: float = 0.0  # この速さで歩く時間

    def summary(self) -> str:
        dx, dy, dyaw = self.error
        s = f"ずれ 前 {dx * 100:+.1f} cm・左 {dy * 100:+.1f} cm・向き {np.degrees(dyaw):+.1f}°"
        if self.done:
            return s + " → 着いた"
        return s + f" → vx {self.vx:+.2f} m/s・vy {self.vy:+.2f} m/s・vyaw {self.vyaw:+.2f} rad/s で {self.duration_s:.2f} 秒"


def _rot2(a: float) -> np.ndarray:
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def stand_error(measured: BinPose, target: BinPose) -> np.ndarray:
    """ロボットが動くべき量 (dx, dy, dyaw)（今の pelvis 座標）。上の説明の M = T · T*⁻¹。"""
    dyaw = float(np.arctan2(np.sin(measured.yaw - target.yaw), np.cos(measured.yaw - target.yaw)))
    d = np.asarray(measured.center_xy, dtype=float) - _rot2(dyaw) @ np.asarray(target.center_xy, dtype=float)
    return np.array([d[0], d[1], dyaw])


def approach_step(measured: BinPose, target: BinPose, cfg: ApproachConfig) -> ApproachStep:
    """測った置き方から、次に歩く指令を決める。ずれが許容内なら done。"""
    err = stand_error(measured, target)
    dx, dy, dyaw = err
    if abs(dx) <= cfg.tol_xy_m and abs(dy) <= cfg.tol_xy_m and abs(np.degrees(dyaw)) <= cfg.tol_yaw_deg:
        return ApproachStep(done=True, error=err)
    # 1 回に動く量を上限で切る（遠くても少しずつ。測り直して直す）
    dist = float(np.hypot(dx, dy))
    if dist > cfg.max_step_m:
        dx, dy = dx * cfg.max_step_m / dist, dy * cfg.max_step_m / dist
    lim_yaw = np.radians(cfg.max_step_yaw_deg)
    dyaw = float(np.clip(dyaw, -lim_yaw, lim_yaw))
    # 許容内の成分は動かさない（細かく揺すらない）
    if abs(err[0]) <= cfg.tol_xy_m:
        dx = 0.0
    if abs(err[1]) <= cfg.tol_xy_m:
        dy = 0.0
    if abs(np.degrees(err[2])) <= cfg.tol_yaw_deg:
        dyaw = 0.0
    # どの成分も速さの上限を超えない時間で、同時に動く
    max_vx = cfg.max_vx if dx >= 0 else cfg.max_reverse_vx
    t = max(abs(dx) / max_vx, abs(dy) / cfg.max_vy, abs(dyaw) / cfg.max_vyaw, cfg.min_step_s)
    return ApproachStep(done=False, error=err, vx=dx / t, vy=dy / t, vyaw=dyaw / t, duration_s=float(t))
