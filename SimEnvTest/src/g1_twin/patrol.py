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
# 旋回し終わったあと、この余裕があれば前進を再開する [m]。
# FRONT_CLEAR_DISTANCE より小さくしてあるのは、狭い通路でも進めるようにするため。
# 同じ値にすると、回りきっても前進条件を満たさず旋回し直しを繰り返す。
FRONT_ESCAPE_DISTANCE: float = 1.2
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
# 制御周期 [s]。回った角度を step 数から求めるのに使う（runner の CONTROL_DT と一致）。
CONTROL_DT_SEC: float = 0.02
# 1 周期でこれ未満しか進まなければ「足踏み」とみなす [m]。
# 前進速度 0.38 m/s なら 1 周期で 0.0076 m 進むので、その 1/4 を閾値にする。
STALL_DISTANCE: float = 0.002
# 足踏みがこの回数続いたら、LiDAR に映らない障害物にぶつかっているとみなす。
# 50Hz なので 150 step = シム内 3 秒。
STALL_STEPS_BEFORE_TURN: int = 150


@dataclass
class PatrolStats:
    """巡回の統計。進捗確認に使う。"""

    steps: int = 0
    forward_steps: int = 0
    turn_steps: int = 0
    # LiDAR に映らない障害物から脱出した回数
    stall_recoveries: int = 0


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
        # 今回の旋回で回るべき角度 [deg]
        self._turn_target_deg = 90.0
        # 脱出のため強制的に前進する残りステップ数
        self._escape_steps = 0
        # 足踏み検知用
        self._last_position: tuple[float, float] | None = None
        self._stall_steps = 0
        self._commanded_forward = False
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

    def _begin_turn(self, scan: ScanData, front: float) -> None:
        """最も開けた方向を選び、そこまで回るように旋回を開始する。

        全方位を 30 度刻みで調べて最良の方向へ向く。左右 90 度だけを見る方式では、
        袋小路（前方 0.5 m / 後方 30 m）で左右とも塞がっているため
        正しい向きを選べなかった。

        Args:
            scan: 現在の LiDAR スキャン
            front: 前方の余裕 [m]（ログ表示用）
        """
        best_angle = 0.0
        best_distance = -1.0
        for angle in range(-180, 180, 30):
            distance = self._sector_min_distance(scan, float(angle), 20.0)
            if distance > best_distance:
                best_distance = distance
                best_angle = float(angle)

        if abs(best_angle) < 1.0:
            # 正面が最良なのに塞がっている場合はランダムに 90 度回る
            self._turn_sign = self._rng.choice((1.0, -1.0))
            self._turn_target_deg = 90.0
        else:
            self._turn_sign = 1.0 if best_angle > 0.0 else -1.0
            self._turn_target_deg = abs(best_angle)

        self._turning = True
        self._turn_steps = 0
        print(
            f"[Patrol] 前方 {front:.2f} m で旋回開始 "
            f"（最も開けた方向 {best_angle:+.0f} 度 = {best_distance:.1f} m、"
            f"{'左' if self._turn_sign > 0 else '右'}へ {self._turn_target_deg:.0f} 度）"
        )

    def notify_position(self, x: float, y: float) -> None:
        """現在位置を伝える。進めているかの判定に使う。

        LiDAR は地上 1.1 m を見ているため、足元の低い障害物（パレットや
        棚の下段）は検出できない。「前方が開いているのに進めない」状況が
        起きるので、位置が動いているかを別途監視する必要がある。

        Args:
            x: ワールド座標の X [m]
            y: ワールド座標の Y [m]
        """
        if self._last_position is None:
            self._last_position = (x, y)
            self._stall_steps = 0
            return

        dx = x - self._last_position[0]
        dy = y - self._last_position[1]
        moved = math.hypot(dx, dy)
        # 前進指令を出しているのに動いていなければ足踏みとみなす
        if self._commanded_forward and moved < STALL_DISTANCE:
            self._stall_steps += 1
        else:
            self._stall_steps = 0
        self._last_position = (x, y)

    def step(self, scan: ScanData) -> VelocityCommand:
        """スキャンを見て次の速度指令を決める。

        Args:
            scan: 現在の LiDAR スキャン

        Returns:
            この制御周期で与える速度指令
        """
        command = self._decide(scan)
        # 足踏み判定のため、前進を指令したかを覚えておく
        self._commanded_forward = command.vx > 0.0
        return command

    def _decide(self, scan: ScanData) -> VelocityCommand:
        """実際の判断を行う（step から呼ばれる）。"""
        self.stats.steps += 1

        # 前方が開いているのに進めていない場合は、LiDAR に映らない
        # 低い障害物にぶつかっている。強制的に向きを変えて脱出する。
        if self._stall_steps >= STALL_STEPS_BEFORE_TURN:
            self._stall_steps = 0
            self._escape_steps = 0
            self._turning = True
            self._turn_steps = 0
            self._turn_sign = self._rng.choice((1.0, -1.0))
            self._turn_target_deg = 120.0
            self.stats.stall_recoveries += 1
            print(
                "[Patrol] 前方は開いているのに進めていないため "
                f"（LiDAR に映らない障害物）、{'左' if self._turn_sign > 0 else '右'}へ "
                "120 度回ります"
            )

        front = self._sector_min_distance(scan, 0.0, FRONT_HALF_ANGLE_DEG)

        if self._turning:
            # 旋回は「決めた角度だけ回りきる」方式にする。
            #
            # 以前は前方が開くまで回り続け、一定時間で反転する方式にしていたが、
            # 壁に鼻先を突きつけた状態（前方 0.5 m / 後方 30 m）では
            # 反転までの時間内に目標方向まで回りきれず、左右に揺れるだけで
            # 永久に脱出できなかった（実測）。角度で管理すれば確実に回りきれる。
            self._turn_steps += 1
            turned_deg = math.degrees(
                PATROL_TURN_RATE * self._turn_steps * CONTROL_DT_SEC
            )

            if turned_deg >= self._turn_target_deg:
                # 回りきった。前方が開いていれば前進に戻る。
                if front > FRONT_ESCAPE_DISTANCE:
                    self._turning = False
                    self._turn_steps = 0
                    # しばらく旋回判定を抑止する。これが無いと次の周期で
                    # また旋回に入り、同じ場所から動けなくなる。
                    self._escape_steps = ESCAPE_FORWARD_STEPS
                    print(
                        f"[Patrol] {turned_deg:.0f} 度回り、前方 {front:.2f} m が"
                        f"開いたので前進します"
                    )
                    self.stats.forward_steps += 1
                    return VelocityCommand(vx=PATROL_SPEED, vy=0.0, yaw_rate=0.0)

                # まだ塞がっている。もう一度、最も開けた方向を選び直す。
                self._begin_turn(scan, front)

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
            self._begin_turn(scan, front)
            self.stats.turn_steps += 1
            return VelocityCommand(
                vx=0.0, vy=0.0, yaw_rate=PATROL_TURN_RATE * self._turn_sign
            )

        self.stats.forward_steps += 1
        return VelocityCommand(vx=PATROL_SPEED, vy=0.0, yaw_rate=0.0)
