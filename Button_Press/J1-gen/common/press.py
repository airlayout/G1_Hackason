"""押し込みの実行（PressPlan を ArmCommander で送る）。sim / real 共通。タスク2・6 で使う。"""

from __future__ import annotations

import numpy as np

from .arm import ArmCommander
from .press_planner import PressPlan


def execute_press(arm: ArmCommander, plan: PressPlan, hold_s: float, return_to: np.ndarray | None = None) -> None:
    """手前の姿勢へ移動 → 直線で押し込む → 止まる → 直線で戻る →（与えれば）return_to へ戻る。

    各段階で ArmCommander が安全チェックと「動いたか」の確認を行う。確認モードなら段階ごとに Enter を待つ。
    """
    print(f"[press] {plan.summary()}")
    arm.move_to(plan.q_approach, label="手前の姿勢へ移動")
    arm.follow(plan.press_in, label="押し込み")
    arm.follow([plan.press_in[-1]] * max(1, int(hold_s / arm.dt)), label="押し込んだまま保持")
    arm.follow(plan.press_out, label="戻り")
    if return_to is not None:
        arm.move_to(return_to, label="開始姿勢へ戻る")
