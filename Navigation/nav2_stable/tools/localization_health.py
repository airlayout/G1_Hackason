"""いま使っている map→odom がスキャンと合っているかを判定する（ROS 非依存）。

`map_localizer.py` が照合のたびに呼ぶ。ROS を import しないので CI でテストできる
（`tests/test_localization_health.py`）。

## 何を見るか

**いま実際に使っている** map→odom（hold なら初期値、correct なら補正後、
observe なら静的に出している初期値）で点群を地図に重ね、**壁から 20cm 以内に入る点の割合
（inlier 率）**を出す。位置が合っていれば高く、ずれていれば下がる。崩れ方の種類
（内蔵SLAM の値の嘘・ドリフト・初期合わせの誤り）を問わず、直接「合っていない」が分かる。

## 判定の規則（2026-11-03 に記録で決めた。findings/localizer_timing_20261103.md）

- **旋回中（odom の yaw 速度が 20°/s 超）のフレームは判定に使わない。** 52°/s では
  1 フレーム 0.1 秒の間に約 5° 回って点群が流れ、正しい位置でも inlier 率が 59% まで下がる
- 残ったフレームの**直近 10 個（2Hz で 5 秒）の平均**が **70% 未満で `lost`**、
  **75% 以上で `ok`**（行ったり来たりしないよう差をつける）
- 窓が埋まるまでは `unknown`

記録での結果: 正しい位置で誤報 0 回（11/01 の巡回 2 本・約 12 分、11/03 の手動歩行 約 10 分）。
わざとずらすと 0.5m で 99%、1m で 100%、10° で 80%、20° で 100% の窓が `lost` になる。
⚠️ 余裕は小さい。11/01 の 1 本目で 5 秒平均の最小が 71% だった（3 秒窓だと 65% で誤報が出た）。
"""

from __future__ import annotations

import math
from collections import deque

UNKNOWN = "unknown"
OK = "ok"
LOST = "lost"


class LocalizationHealth:
    def __init__(self, window: int = 10, lost_below: float = 0.70, ok_at: float = 0.75,
                 max_yaw_rate_deg: float = 20.0) -> None:
        if not lost_below < ok_at:
            raise ValueError(f"lost_below({lost_below}) は ok_at({ok_at}) より小さくすること")
        self.window = window
        self.lost_below = lost_below
        self.ok_at = ok_at
        self.max_yaw_rate = math.radians(max_yaw_rate_deg)
        self.reset()

    def reset(self) -> None:
        """位置を差し替えたとき（relocalize）に呼ぶ。過去の inlier 率は新しい位置と無関係になる。"""
        self._values: deque[float] = deque(maxlen=self.window)
        self._prev: tuple[float, float] | None = None
        self.state = UNKNOWN
        self.spinning = False
        self.last_inlier: float | None = None

    @property
    def average(self) -> float | None:
        if len(self._values) < self.window:
            return None
        return sum(self._values) / len(self._values)

    def update(self, t: float, odom_yaw: float, inlier: float) -> str:
        """1 回の照合ぶん。t[秒]・odom 系のセンサーの yaw[rad]・inlier 率(0〜1)。いまの状態を返す。"""
        self.last_inlier = inlier
        self.spinning = False
        if self._prev is not None and t > self._prev[0]:
            dyaw = math.atan2(math.sin(odom_yaw - self._prev[1]), math.cos(odom_yaw - self._prev[1]))
            self.spinning = abs(dyaw) / (t - self._prev[0]) > self.max_yaw_rate
        self._prev = (t, odom_yaw)
        if self.spinning:
            return self.state   # 旋回中は判定に使わない（状態も変えない）
        self._values.append(inlier)
        avg = self.average
        if avg is None:
            return self.state
        if self.state != LOST and avg < self.lost_below:
            self.state = LOST
        elif self.state != OK and avg >= self.ok_at:
            self.state = OK
        return self.state
