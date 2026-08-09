"""SLAM 用の自動巡回。

キーボード操作の代わりに、LiDAR のスキャンを見ながら Warehouse を
自動で歩き回って地図を作る。人が操作できない状況（夜間の自動実行など）
でも SLAM を完了させるために用いる。

方針:
    前方が開いていれば前進する。塞がっていれば、より開いている側へ
    その場旋回する。壁沿いを追うのではなく単純な反応型にしてある。
    歩行ポリシーは後退が苦手（-0.3 m/s 以上で転倒）なので後退は使わない。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .command import VelocityCommand
from .lidar import ScanData

# 前方とみなす角度の範囲 [deg]（正面 ±この値）
FRONT_HALF_ANGLE_DEG: float = 35.0
# 前進を続けてよい前方の余裕 [m]。これを下回ると旋回に切り替える。
FRONT_CLEAR_DISTANCE: float = 1.6
# 旋回を止めてよい前方の余裕 [m]。FRONT_CLEAR_DISTANCE より大きくして
# 境界での前進と旋回の往復（チャタリング）を防ぐ。
FRONT_RESUME_DISTANCE: float = 2.2
# 巡回時の前進速度 [m/s]（実測で +0.5 指令 -> 0.38 実速度）
PATROL_SPEED: float = 0.5
# 旋回速度 [rad/s]（実測で指令とほぼ一致する）
PATROL_TURN_RATE: float = 0.6
# 同じ場所で旋回し続けたときに諦めて向きを変えるまでのステップ数
STUCK_TURN_STEPS: int = 400


@dataclass
class PatrolStats:
    """巡回の統計。進捗確認に使う。"""

    steps: int = 0
    forward_steps: int = 0
    turn_steps: int = 0


class AutoPatrol:
    """LiDAR を見ながら自動で歩き回る巡回コントローラ。

    状態は「前進中」か「旋回中」の 2 つだけ。前方が塞がったら
    開いている側へ旋回し、十分開いたら前進に戻る。
    """

    def __init__(self, seed: int = 0) -> None:
        """巡回を初期化する。

        Args:
            seed: 旋回方向をランダムに選ぶ際の乱数種。再現性のため固定する。
        """
        self._rng = random.Random(seed)
        self._turning = False
        # 旋回方向（+1: 左, -1: 右）。旋回に入るたびに決め直す。
        self._turn_sign = 1.0
        self._turn_steps = 0
        self.stats = PatrolStats()
        # 扇形ごとのビーム番号。スキャンの形は毎回同じなので初回に作って使い回す。
        self._sector_cache: dict[tuple[float, float], list[int]] = {}

    def _sector_indices(
        self, scan: ScanData, center_deg: float, half_width_deg: float
    ) -> list[int]:
        """指定方向の扇形に入るビーム番号を返す（初回のみ計算して以後は再利用）。"""
        key = (center_deg, half_width_deg)
        cached = self._sector_cache.get(key)
        if cached is not None:
            return cached

        indices: list[int] = []
        for index in range(len(scan.ranges)):
            angle_deg = math.degrees(scan.angle_min + index * scan.angle_increment)
            # 角度差を -180..180 に正規化して比較する
            delta = (angle_deg - center_deg + 180.0) % 360.0 - 180.0
            if abs(delta) <= half_width_deg:
                indices.append(index)
        self._sector_cache[key] = indices
        return indices

    def _sector_min_distance(
        self, scan: ScanData, center_deg: float, half_width_deg: float
    ) -> float:
        """指定方向を中心とした扇形の最小距離を返す。

        Args:
            scan: LiDAR のスキャン
            center_deg: 扇の中心角 [deg]（0 が正面、左が正）
            half_width_deg: 扇の半幅 [deg]

        Returns:
            その範囲の最小距離 [m]。有効な点が無ければ range_max。
        """
        ranges = scan.ranges
        minimum = scan.range_max
        for index in self._sector_indices(scan, center_deg, half_width_deg):
            distance = ranges[index]
            # inf（測距不能）は比較しても minimum を下げないので判定不要
            if distance < minimum:
                minimum = distance
        return minimum

    def step(self, scan: ScanData) -> VelocityCommand:
        """スキャンを見て次の速度指令を決める。

        Args:
            scan: 現在の LiDAR スキャン

        Returns:
            この制御周期で与える速度指令
        """
        self.stats.steps += 1

        front = self._sector_min_distance(scan, 0.0, FRONT_HALF_ANGLE_DEG)

        if self._turning:
            # 十分開けたら前進に戻る
            if front > FRONT_RESUME_DISTANCE:
                self._turning = False
                self._turn_steps = 0
                self.stats.forward_steps += 1
                return VelocityCommand(vx=PATROL_SPEED, vy=0.0, yaw_rate=0.0)

            self._turn_steps += 1
            # 長く回り続けても開けない場合は逆向きに切り替えて脱出を試みる
            if self._turn_steps > STUCK_TURN_STEPS:
                self._turn_sign = -self._turn_sign
                self._turn_steps = 0
                print("[Patrol] 旋回が長引いたため向きを反転します")

            self.stats.turn_steps += 1
            return VelocityCommand(
                vx=0.0, vy=0.0, yaw_rate=PATROL_TURN_RATE * self._turn_sign
            )

        # 前進中: 前方が塞がったら旋回に入る
        if front < FRONT_CLEAR_DISTANCE:
            # 左右のどちらが開いているかを見て旋回方向を決める
            left = self._sector_min_distance(scan, 90.0, 45.0)
            right = self._sector_min_distance(scan, -90.0, 45.0)
            if abs(left - right) < 0.5:
                # ほぼ同じならランダムに選ぶ（対称な場所で固まらないように）
                self._turn_sign = self._rng.choice((1.0, -1.0))
            else:
                self._turn_sign = 1.0 if left > right else -1.0

            self._turning = True
            self._turn_steps = 0
            print(
                f"[Patrol] 前方 {front:.2f} m で旋回開始 "
                f"（左 {left:.1f} / 右 {right:.1f}、"
                f"{'左' if self._turn_sign > 0 else '右'}へ）"
            )
            self.stats.turn_steps += 1
            return VelocityCommand(
                vx=0.0, vy=0.0, yaw_rate=PATROL_TURN_RATE * self._turn_sign
            )

        self.stats.forward_steps += 1
        return VelocityCommand(vx=PATROL_SPEED, vy=0.0, yaw_rate=0.0)
