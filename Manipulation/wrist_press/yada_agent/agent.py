"""Yada's standard --agent entry point (MuJoCo, Isaac and DDS)."""
from pathlib import Path
import os
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from contest.interface import Action, Agent
from Manipulation.wrist_press.adapter import SampleBuilder
from Manipulation.wrist_press.controller import PressController
from Manipulation.wrist_press.settings import load_settings


class WristPressAgent(Agent):
    def __init__(self):
        self.controller = None

    def reset(self, task):
        if self.controller:
            self.controller.close()
        cfg = load_settings(os.environ.get("G1_PRESS_CONFIG"),
                            mode=os.environ.get("G1_PRESS_MODE"),
                            detector=os.environ.get("G1_PRESS_DETECTOR"))
        # The original Action schema has only joint targets, no torque field.
        cfg["gravity_mode"]="position"
        self.controller = PressController(cfg)
        self.controller.reset(task.target, task.upper_kp)
        self.builder = SampleBuilder(self.controller.robot, task.sim)

    def act(self, obs):
        command = self.controller.act(self.builder.build(obs))
        return Action(q_target=command.q[12:].copy(), done=command.done)

    def close(self):
        if self.controller:
            self.controller.close()


def make_agent():
    return WristPressAgent()
