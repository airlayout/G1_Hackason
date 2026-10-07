"""Yada の API / 模擬 DDS に接続する。実機コマンドの既定値は変更しない。"""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from contest.interface import Action, Agent
from yada import YadaButtonController


class PushButtonAgent(Agent):
    def __init__(self):
        self.controller = YadaButtonController(os.environ.get("G1_YADA_WEIGHTS") or None)

    def reset(self, task):
        # 採点用の寸法・押込量を実機へ持ち込まない。実機は従来の real/ を使う。
        if task.sim not in ("mujoco", "isaac", "sim_dds"):
            raise ValueError("この入口は採点環境専用です。実機は real/align_button.py を使ってください")
        self.controller.reset(task)

    def act(self, obs):
        try:
            q_upper, done = self.controller.act(obs)
        except ValueError as exc:
            raise ValueError(f"{exc}（段階: {self.controller.phase}、計測: {self.controller.measurement_count} 回）") from exc
        return Action(q_target=q_upper, done=done)


def make_agent():
    return PushButtonAgent()
