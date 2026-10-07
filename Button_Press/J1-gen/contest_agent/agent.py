"""Yada の評価環境（Button_Press/Yada/contest）で動かす、J1-gen のエージェント。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seeds smoke
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seed 1 --view
    J1GEN_BUTTON_DETECTOR=yolo $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent

ボタンを探す方法は configs/elevator.yaml の detector（classic / yolo）。環境変数 J1GEN_BUTTON_DETECTOR で上書きできる
（yolo は sim/train_button_yolo.py で学習した重みを使う）。

中身は common/elevator_press.py（見つける・計画・押す）。ここは評価環境の決まり（Agent / Observation / Action）に
つなぐだけ。評価環境は Button_Press/Yada を import の検索先に入れるので、`common` は Yada のパッケージになる。
J1-gen の部品は、Yada の j1gen_bridge で別名（j1gen_common）として読み込む。
シミュレーターの正解の値（ボタンの座標など）は使わない。
"""

from __future__ import annotations

import os

from common.j1gen_bridge import j1gen, j1gen_config
from contest.interface import UPPER_BODY_IDX, Action, Agent, Observation, TaskInfo


class J1genElevatorAgent(Agent):
    def __init__(self) -> None:
        elevator = j1gen_config("elevator.yaml")
        if os.environ.get("J1GEN_BUTTON_DETECTOR"):
            elevator["detector"] = os.environ["J1GEN_BUTTON_DETECTOR"]
        self.ctl = j1gen("elevator_press").ElevatorPressController.from_config(
            j1gen_config("robot.yaml"), j1gen_config("press.yaml"), j1gen_config("arm.yaml"), elevator)
        print(f"[j1gen-agent] ボタンを探す方法: {elevator['detector']}")

    def reset(self, task: TaskInfo) -> None:
        self.ctl.reset(task.target, task.control_dt, task.upper_kp)

    def act(self, obs: Observation) -> Action:
        q29, done = self.ctl.step(obs.rgb, obs.depth, obs.K, obs.q, obs.t, obs.image_t, obs.imu_quat)
        return Action(q_target=q29[list(UPPER_BODY_IDX)], done=done)


def make_agent() -> Agent:
    return J1genElevatorAgent()
