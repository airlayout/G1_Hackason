"""押したあとの確認（差し込み口）。タスク6。

押し込み → 保持 → 手前の姿勢へ戻った時点で呼ばれる。結果に応じて、少し深くして押し直す。

- ボトル押し: NoCheck（何もしない。常に OK）
- ボタン押し（10/14 以降の予定）: 「ボタンが点灯したか」を頭カメラで確かめ、点かなければ
  deeper_m だけ深くして押し直す確認を、この形で作って差し込む

    class LampCheck:
        def __call__(self, ctx: PostPressContext) -> PostPressResult:
            frame = ctx.grab_frame()                   # 頭カメラの最新のフレーム（RgbdFrame か None）
            lit = ...                                  # 枠の中の明るさなどで判定
            return PostPressResult(ok=lit, deeper_m=0.0 if lit else 0.005, message="点灯した" if lit else "点かない")

押し直しの回数と深さには上限がある（configs/press.yaml の post_check と press.max_press_depth_m）。
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Protocol


@dataclass
class PostPressResult:
    ok: bool
    # ok=False のとき、次の押し込みをどれだけ深くするか [m]（0 なら押し直さない）
    deeper_m: float = 0.0
    message: str = ""
    # ログに残したいもの（判定に使った値など）
    data: dict[str, Any] = field(default_factory=dict)


@dataclass
class PostPressContext:
    attempt: int  # 何回目の押し込みか（1 から）
    depth_m: float  # 今回の押し込みの深さ [m]
    plan: Any  # 今回の PressPlan
    grab_frame: Callable[[], Any]  # 頭カメラの最新のフレームを返す（カメラが無ければ None を返す）


class PostPressCheck(Protocol):
    def __call__(self, ctx: PostPressContext) -> PostPressResult: ...


class NoCheck:
    """ボトル押し用。何もしない。"""

    def __call__(self, ctx: PostPressContext) -> PostPressResult:
        return PostPressResult(ok=True, message="確認なし（ボトル）")
