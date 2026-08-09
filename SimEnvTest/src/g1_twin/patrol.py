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
# 旋回しても FRONT_RESUME_DISTANCE に届かない狭い場所では、
# これ以上の余裕があれば前進を再開する（デッドロック脱出用）。
# 通路が狭く四方が 2.2 m 未満だと、どちらを向いても旋回を抜けられず
# その場で回り続けてしまうため（実際に発生した）。
FRONT_ESCAPE_DISTANCE: float = 1.2
# この回数だけ旋回方向を反転しても抜けられなければ脱出モードに入る
STUCK_REVERSALS_BEFORE_ESCAPE: int = 2
# 脱出時に、旋回判定を抑止して強制的に前進し続けるステップ数。
# これが無いと前進を再開した次の周期でまた旋回に入り、同じ場所に留まる。
# 50Hz なので 100 step = シム内 2 秒 = 約 0.8 m 進む。
ESCAPE_FORWARD_STEPS: int = 100
# 脱出中でも、これより近ければ衝突するので旋回に戻す [m]
ESCAPE_ABORT_DISTANCE: float = 0.7
# 巡回時の前進速度 [m/s]（実測で +0.5 指令 -> 0.38 実速度）
PATROL_SPEED: float = 0.5
# 旋回速度 [rad/s]（実測で指令とほぼ一致する）
PATROL_TURN_RATE: float = 0.6
# 同じ場所で旋回し続けたときに諦めて向きを変えるまでのステップ数。
# 50Hz なので 150 step = シム内 3 秒。旋回速度 0.6 rad/s では約 100 度回る。
# これで開けないなら向きを変えても無駄と判断する。
STUCK_TURN_STEPS: int = 150


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
        # 旋回方向を反転した回数（デッドロック検知用）
        self._reversals = 0
        # 脱出のため強制的に前進する残りステップ数
        self._escape_steps = 0
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
                self._reversals = 0
                self.stats.forward_steps += 1
                return VelocityCommand(vx=PATROL_SPEED, vy=0.0, yaw_rate=0.0)

            # 何度反転しても開けない場合は、狭い通路にいるとみなして
            # 判定を緩める。そうしないと同じ場所で回り続けてしまう。
            if (
                self._reversals >= STUCK_REVERSALS_BEFORE_ESCAPE
                and front > FRONT_ESCAPE_DISTANCE
            ):
                self._turning = False
                self._turn_steps = 0
                self._reversals = 0
                # しばらく旋回判定を抑止しないと、次の周期で再び旋回に入り
                # 同じ場所から動けなくなる（実際に発生した）
                self._escape_steps = ESCAPE_FORWARD_STEPS
                print(
                    f"[Patrol] 狭い場所なので前方 {front:.2f} m で前進を再開します"
                    f"（{ESCAPE_FORWARD_STEPS} step は旋回しない）"
                )
                self.stats.forward_steps += 1
                return VelocityCommand(vx=PATROL_SPEED, vy=0.0, yaw_rate=0.0)

            self._turn_steps += 1
            # 長く回り続けても開けない場合は逆向きに切り替えて脱出を試みる
            if self._turn_steps > STUCK_TURN_STEPS:
                self._turn_sign = -self._turn_sign
                self._turn_steps = 0
                self._reversals += 1
                print(
                    f"[Patrol] 旋回が長引いたため向きを反転します "
                    f"（{self._reversals} 回目、前方 {front:.2f} m）"
                )

            self.stats.turn_steps += 1
            return VelocityCommand(
                vx=0.0, vy=0.0, yaw_rate=PATROL_TURN_RATE * self._turn_sign
            )

        # 脱出中はしばらく旋回せずに前進する。ただし衝突しそうなら中止する。
        if self._escape_steps > 0:
            self._escape_steps -= 1
            if front > ESCAPE_ABORT_DISTANCE:
                self.stats.forward_steps += 1
                return VelocityCommand(vx=PATROL_SPEED, vy=0.0, yaw_rate=0.0)
            # 近すぎるので脱出を打ち切って通常の判定に戻す
            self._escape_steps = 0
            print(f"[Patrol] 前方 {front:.2f} m まで近づいたため脱出を中止します")

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
            self._reversals = 0
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
