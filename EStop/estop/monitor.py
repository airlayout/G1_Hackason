"""ボタン・ロボット・状態遷移をつなぐ周期ループ。"""

from __future__ import annotations

import queue
import time
from collections.abc import Callable

from .io import ButtonSource, RobotPort
from .safe_stop import SafeStop

PERIOD_S = 0.01  # 100Hz。ボタン断から指令まで最大 1 周期 + 通信遅延
OPERATOR_COMMANDS = ("reset", "damp")


def tick(
    now: float,
    button: ButtonSource,
    robot: RobotPort,
    safe: SafeStop,
    operator: queue.Queue[str],
    log: Callable[[str], None] = print,
) -> None:
    """1 周期ぶん。ループから分けてあるのはテストのため。"""
    while not operator.empty():
        _handle_operator(now, operator.get_nowait(), safe, button.is_ok())
    cmd = safe.step(now, button.is_ok(), robot.observe())
    for t, state, reason in safe.pop_events():
        log(f"[estop] t={t:.2f} -> {state.value}: {reason}")
    if not safe.permits_motion:
        robot.apply(cmd)
    if safe.alarm:
        log("[estop] 警報: 静止できません。ゼロ速度を保持中。ダンピングは操作者が指示(damp)")


def _handle_operator(now: float, text: str, safe: SafeStop, button_ok: bool) -> None:
    if text == "reset":
        safe.reset(now, operator_ack=button_ok)
    elif text == "damp":
        safe.request_damp(now)


def run(
    button: ButtonSource,
    robot: RobotPort,
    safe: SafeStop,
    operator: queue.Queue[str],
    stop_flag: Callable[[], bool] = lambda: False,
) -> None:
    while not stop_flag():
        tick(time.monotonic(), button, robot, safe, operator)
        time.sleep(PERIOD_S)
