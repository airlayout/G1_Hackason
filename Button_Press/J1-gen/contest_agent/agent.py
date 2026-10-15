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

環境変数 J1GEN_BODY_LOG にフォルダを渡すと、毎周期の関節角・IMU・段階をそこに記録する（実機で、押している間に
足側のバランス制御が腰をどれだけ動かしたかを見るため。読み方は real/body_motion.py）:

    J1GEN_BODY_LOG=_local/button_press/body_log $P Button_Press/Yada/contest/run_dds.py --agent ... （実機の引数）
    $P Button_Press/J1-gen/real/body_motion.py _local/button_press/body_log/<日時>_<up|down>
"""

from __future__ import annotations

import csv
import json
import os
import time
from pathlib import Path

import numpy as np

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
        self.body_log = BodyLog.open(task.target)

    def act(self, obs: Observation) -> Action:
        q29, done = self.ctl.step(obs.rgb, obs.depth, obs.K, obs.q, obs.t, obs.image_t, obs.imu_quat)
        if self.body_log is not None:
            self.body_log.write(obs, self.ctl)
        return Action(q_target=q29[list(UPPER_BODY_IDX)], done=done)


class BodyLog:
    """毎周期の関節角（29）・IMU・段階を body_log.csv に書く（列は common/body_motion.read_lowstate_csv が読む形）。
    計画ができたら、押す点（pelvis 座標）を meta.json に書く。"""

    def __init__(self, folder: Path) -> None:
        folder.mkdir(parents=True, exist_ok=False)
        self.dir = folder
        self._f = open(folder / "body_log.csv", "w", newline="", encoding="utf-8", buffering=1)
        self._w = csv.writer(self._f)
        self._w.writerow(["t", "phase"] + [f"q{i}" for i in range(29)] + ["qw", "qx", "qy", "qz"])
        self._planned = False
        print(f"[j1gen-agent] 体の動きの記録: {folder}")

    @classmethod
    def open(cls, target: str) -> "BodyLog | None":
        root = os.environ.get("J1GEN_BODY_LOG")
        if not root:
            return None
        return cls(Path(root) / f"{time.strftime('%Y%m%d_%H%M%S')}_{target}")

    def write(self, obs: Observation, ctl: object) -> None:
        phase = getattr(ctl, "phase", "")
        self._w.writerow([f"{obs.t:.4f}", phase] + [f"{v:.6f}" for v in obs.q]
                         + [f"{v:.6f}" for v in np.asarray(obs.imu_quat, dtype=float)])
        plan = getattr(ctl, "plan_", None)
        if plan is not None and not self._planned:
            self._planned = True
            (self.dir / "meta.json").write_text(json.dumps(
                {"target_point": np.asarray(plan.target_point).tolist(), "planned_t": obs.t,
                 "phase": phase}, ensure_ascii=False, indent=2))


def make_agent() -> Agent:
    return J1genElevatorAgent()
