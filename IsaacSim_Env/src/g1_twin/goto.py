"""座標を1つ渡すと、そこへ障害物回避なしで直進する単純なコントローラ。

Nav2 のような経路計画は行わない。目標までの直線方向を向き、
向いたら前進するだけ（patrol.py の反応型ロジックと同じ発想）。
壁や障害物があっても避けないため、開けた場所での動作確認や、
Nav2 を起動せずに素早く「目的地まで歩かせたい」場合に使う。
"""

from __future__ import annotations

import math

from .command import VelocityCommand

# 目標に到達したとみなす半径 [m]
ARRIVAL_RADIUS: float = 0.3
# この角度以上ずれていたら、その場旋回のみ行う（歩きながらの旋回は
# 大きくずれた状態では効きが悪いため、まず正面を合わせる） [deg]
TURN_THRESHOLD_DEG: float = 15.0
# 前進速度 [m/s]（patrol.py の PATROL_SPEED と同じ値。実測で歩行が安定する）
FORWARD_SPEED: float = 0.5
# 旋回速度 [rad/s]（patrol.py の PATROL_TURN_RATE と同じ値）
TURN_RATE: float = 0.6
# 目標に近づいたら前進速度を落とし始める距離 [m]（行き過ぎ防止）
SLOWDOWN_RADIUS: float = 1.0
# 前進中の進行方向の補正ゲイン（比例制御）
HEADING_KP: float = 1.2


def _normalize_angle(angle_rad: float) -> float:
    """角度を -pi..pi に正規化する。"""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class GotoController:
    """1つの目標座標 (x, y) へ直進するコントローラ。"""

    def __init__(self, target_xy: tuple[float, float]) -> None:
        """コントローラを初期化する。

        Args:
            target_xy: 目標のワールド座標 (x, y) [m]
        """
        self._target_x, self._target_y = target_xy
        self._reached = False
        print(f"[Goto] 目標: ({self._target_x:.2f}, {self._target_y:.2f})")

    @property
    def reached(self) -> bool:
        """目標に到達したかどうか。"""
        return self._reached

    def step(self, x: float, y: float, yaw: float) -> VelocityCommand:
        """現在位置・向きから、この周期の速度指令を計算する。

        Args:
            x: 現在のワールド座標 X [m]
            y: 現在のワールド座標 Y [m]
            yaw: 現在の向き [rad]（ワールド座標系、反時計回りが正）

        Returns:
            この周期に送る速度指令。到達済みなら常にゼロ。
        """
        if self._reached:
            return VelocityCommand()

        dx = self._target_x - x
        dy = self._target_y - y
        distance = math.hypot(dx, dy)

        if distance <= ARRIVAL_RADIUS:
            self._reached = True
            print(f"[Goto] 到達しました（残差 {distance:.2f} m）")
            return VelocityCommand()

        heading_to_target = math.atan2(dy, dx)
        heading_error = _normalize_angle(heading_to_target - yaw)

        if math.degrees(abs(heading_error)) > TURN_THRESHOLD_DEG:
            # 正面が大きくずれている: その場旋回のみ
            yaw_rate = TURN_RATE if heading_error > 0.0 else -TURN_RATE
            return VelocityCommand(vx=0.0, vy=0.0, yaw_rate=yaw_rate)

        # おおむね正面を向いている: 前進しつつ向きを微調整する。
        # 近づいたら前進速度を落として行き過ぎを防ぐ。
        speed_scale = min(1.0, distance / SLOWDOWN_RADIUS)
        vx = FORWARD_SPEED * speed_scale
        yaw_rate = max(-TURN_RATE, min(TURN_RATE, HEADING_KP * heading_error))
        return VelocityCommand(vx=vx, vy=0.0, yaw_rate=yaw_rate)
