"""キーボード入力を速度コマンドへ変換する。

Isaac Sim の入力системを使い、押下中のキーに応じて VelocityCommand を生成する。
キー配置:
    W / S : 前進 / 後退      (vx)
    A / D : 左移動 / 右移動  (vy)
    Q / E : 左旋回 / 右旋回   (yaw_rate)
    SPACE : 即停止
    SHIFT : 押している間だけ低速（微調整用）
"""

from __future__ import annotations

from .command import (
    ANG_VEL_Z_RANGE,
    LIN_VEL_X_RANGE,
    LIN_VEL_Y_RANGE,
    VelocityCommand,
)

# 通常時の指令値。ポリシーの学習範囲内に収まるようにしている。
DEFAULT_LIN_VEL: float = 0.6
DEFAULT_ANG_VEL: float = 0.6

# 後退のみ別の値を使う。このポリシーは前進のみで学習されており、
# -0.3 m/s 以上の後退で転倒するため（実測）。
DEFAULT_BACK_VEL: float = 0.2

# 横移動の上限は学習範囲 (±0.5) に合わせる
DEFAULT_LAT_VEL: float = 0.4

# SHIFT 押下中に掛ける倍率（微調整用）
SLOW_SCALE: float = 0.35


class KeyboardCommander:
    """キーボードの押下状態から速度コマンドを組み立てる。

    Isaac Sim の `carb.input` を用いてキーイベントを購読する。
    Isaac Sim のアプリが起動した後にインスタンス化する必要がある。
    """

    def __init__(
        self,
        lin_vel: float = DEFAULT_LIN_VEL,
        ang_vel: float = DEFAULT_ANG_VEL,
    ) -> None:
        """キーボード購読を開始する。

        Args:
            lin_vel: W/A/S/D で与える並進速度の大きさ [m/s]
            ang_vel: Q/E で与える旋回角速度の大きさ [rad/s]
        """
        self._lin_vel = lin_vel
        self._ang_vel = ang_vel
        self._pressed: set[str] = set()

        # Isaac Sim 起動後にのみ import 可能
        import carb.input
        import omni.appwindow

        self._carb_input = carb.input
        app_window = omni.appwindow.get_default_app_window()
        self._keyboard = app_window.get_keyboard()
        self._input = carb.input.acquire_input_interface()
        self._subscription = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )
        print("[G1] キーボード操作を開始しました: W/S=前後 A/D=左右 Q/E=旋回 SPACE=停止 SHIFT=低速")

    def _on_keyboard_event(self, event, *args) -> bool:
        """キーイベントを受けて押下集合を更新する。"""
        name = event.input.name
        event_type = event.type

        if event_type == self._carb_input.KeyboardEventType.KEY_PRESS:
            if name == "SPACE":
                # 即停止: 押下中のキーを全て解除する。
                # キーを押しっぱなしのまま SPACE を押した場合でも確実に止まる。
                self._pressed.clear()
            else:
                self._pressed.add(name)
        elif event_type == self._carb_input.KeyboardEventType.KEY_RELEASE:
            self._pressed.discard(name)

        # False を返すと他のハンドラへ伝播しない。ビューポート操作を残すため True。
        return True

    def _held(self, *names: str) -> bool:
        """指定キーのいずれかが押下中か。"""
        return any(n in self._pressed for n in names)

    def poll(self) -> VelocityCommand:
        """現在の押下状態から速度コマンドを生成する。

        キーが何も押されていなければ停止コマンドを返す。
        （SPACE は押下集合を空にするので、この経路で停止になる）
        """
        scale = SLOW_SCALE if self._held("LEFT_SHIFT", "RIGHT_SHIFT") else 1.0
        lin = self._lin_vel * scale
        ang = self._ang_vel * scale
        back = DEFAULT_BACK_VEL * scale
        lat = DEFAULT_LAT_VEL * scale

        vx = 0.0
        vy = 0.0
        yaw_rate = 0.0

        if self._held("W", "UP"):
            vx += lin
        if self._held("S", "DOWN"):
            # 後退は転倒しやすいため控えめな値にする
            vx -= back
        # 左が正
        if self._held("A"):
            vy += lat
        if self._held("D"):
            vy -= lat
        # 反時計回りが正
        if self._held("Q", "LEFT"):
            yaw_rate += ang
        if self._held("E", "RIGHT"):
            yaw_rate -= ang

        return VelocityCommand(vx=vx, vy=vy, yaw_rate=yaw_rate).clamped()

    def close(self) -> None:
        """キーボード購読を解除する。"""
        if self._subscription is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._subscription)
            self._subscription = None
