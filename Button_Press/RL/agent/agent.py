"""評価環境（Button_Press/Yada）の Agent: 画像でボタンを見つけ、学習した教師の方策で右腕を動かして押す。

    P=~/miniconda3/envs/lerobot/bin/python
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/RL/agent --target down --trials 20
    bash Button_Press/Yada/contest/evaluate_isaac.sh --agent Button_Press/RL/agent --target down --seeds practice

流れ:
1. 最初の観測で、見本のエージェント（Yada の example_agent）と同じ方法でボタンを見つける
   （カラーで明るい丸を探し、深度で 3 次元の位置にする）。目標（▼）の面の中心を pelvis 座標で持つ
2. 毎周期、体の感覚（右腕の角度・速度・目標との差・目標・前の行動）と、面の位置・指先の位置（FK）から
   観測を作り、方策が出す右腕 7 関節の目標の差分を足す（学習環境と同じ計算: button_press_rl/proprio.py）
3. 腰と左腕は、開始時の角度のまま

方策のファイル: 環境変数 BUTTON_PRESS_RL_POLICY か、このフォルダの policy.pt（scripts/eval_teacher.sh --export で書き出す）。
評価の決まりどおり、シミュレーターは import しない。torch（CPU）だけを使う。
"""

from __future__ import annotations

import importlib.util
import os
import sys
from collections.abc import Callable
from pathlib import Path

import numpy as np

RL_DIR = Path(__file__).resolve().parents[1]
YADA_DIR = RL_DIR.parent / "Yada"
sys.path.insert(0, str(RL_DIR))
sys.path.insert(0, str(YADA_DIR))

from common.j1gen_bridge import j1gen, j1gen_config  # noqa: E402
from contest.interface import RIGHT_ARM_IDX, UPPER_BODY_IDX, Action, Agent, Observation, TaskInfo  # noqa: E402

from button_press_rl.proprio import NUM_ACTIONS, POLICY_OBS_DIM, policy_obs, proprio, step_command  # noqa: E402
from button_press_rl.spec import load_task_spec  # noqa: E402

DEFAULT_POLICY = Path(__file__).with_name("policy.pt")
# BUTTON_PRESS_RL_DEBUG=1 で、指先と面の位置を 10 周期ごとに表示する（シミュレーターの違いを調べるため）
DEBUG = bool(os.environ.get("BUTTON_PRESS_RL_DEBUG"))
Locator = Callable[[Observation], np.ndarray | None]


def _example_agent_module():
    """Yada の見本のエージェント（ボタンの検出に使う）を別名で読み込む。"""
    path = YADA_DIR / "contest" / "example_agent" / "agent.py"
    spec = importlib.util.spec_from_file_location("yada_example_agent", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_policy(path: str | Path | None = None) -> Callable[[np.ndarray], np.ndarray]:
    """TorchScript の方策を読み、観測 (D,) → 行動 (7,) の関数にする。"""
    import torch

    p = Path(path or os.environ.get("BUTTON_PRESS_RL_POLICY") or DEFAULT_POLICY)
    if not p.exists():
        raise FileNotFoundError(f"方策のファイルが無い: {p}（bash Button_Press/RL/scripts/eval_teacher.sh --export {DEFAULT_POLICY} --no-eval）")
    model = torch.jit.load(str(p), map_location="cpu")
    model.eval()

    def run(obs: np.ndarray) -> np.ndarray:
        with torch.inference_mode():
            return model(torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0))[0].numpy()

    return run


class TeacherAgent(Agent):
    def __init__(self, policy: Callable[[np.ndarray], np.ndarray] | None = None, locate: Locator | None = None):
        self.policy = policy if policy is not None else load_policy()
        self._locate = locate
        self.spec = load_task_spec()
        robot_cfg = j1gen_config("robot.yaml")
        self.kin = j1gen("kinematics").ArmKinematics(robot_cfg, "right", j1gen_config("press.yaml")["ik"])
        self.arm = list(RIGHT_ARM_IDX)
        self.q0 = np.array([self.spec.initial_pose[n] for n in
                            (j1gen("robot_model").JOINT_NAMES[i] for i in self.arm)])
        if locate is None:
            self._example = _example_agent_module()
            self.cam_tf = j1gen("camera_geometry").HeadCameraTransform(robot_cfg)

    # ---- Agent ----------------------------------------------------------------

    def reset(self, task: TaskInfo) -> None:
        self.task = task
        self.face: np.ndarray | None = None
        self.q_hold: np.ndarray | None = None
        self.q_cmd: np.ndarray | None = None
        self.last_action = np.zeros(NUM_ACTIONS)
        lo, hi = self.kin.lower[self.kin.arm_idx], self.kin.upper[self.kin.arm_idx]
        self.lo, self.hi = lo + self.spec.limit_margin, hi - self.spec.limit_margin

    def act(self, obs: Observation) -> Action:
        if self.q_hold is None:
            self.q_hold = obs.q.copy()
            self.q_cmd = np.clip(obs.q[self.arm], self.lo, self.hi)
        if self.face is None:
            self.face = self.locate(obs)
            if self.face is None:
                # 見つからなければ、腕を動かさずに終える（違うボタンを押すより安全）
                return Action(q_target=self.q_hold[list(UPPER_BODY_IDX)], done=True)
        q, dq = obs.q[self.arm], obs.dq[self.arm]
        q29 = self.q_hold.copy()
        q29[self.arm] = q
        tip = self.kin.fk_pos(q29)
        prop = proprio(q, dq, self.q_cmd, self.q0, self.last_action, np.concatenate)
        x = policy_obs(prop, self.face, tip, np.concatenate).astype(np.float32)
        assert x.shape == (POLICY_OBS_DIM,)
        action = np.clip(np.asarray(self.policy(x), dtype=float).reshape(-1), -1.0, 1.0)
        if DEBUG and int(round(obs.t / self.task.control_dt)) % 10 == 0:
            print(f"[rl_agent] t={obs.t:.2f} 指先−面={np.round(tip - self.face, 3).tolist()} 行動={np.round(action, 1).tolist()}")
        self.q_cmd = step_command(self.q_cmd, action, self.lo, self.hi, np.clip)
        self.last_action = action
        q_des = self.q_hold.copy()
        q_des[self.arm] = self.q_cmd
        return Action(q_target=q_des[list(UPPER_BODY_IDX)])

    # ---- 内部 -----------------------------------------------------------------

    def locate(self, obs: Observation) -> np.ndarray | None:
        """目標のボタンの面の中心（pelvis 座標）。見つからなければ None。"""
        if self._locate is not None:
            return self._locate(obs)
        buttons = self._example.find_buttons(obs, self.cam_tf)
        for b in buttons:
            if b.name == self.task.target:
                print(f"[rl_agent] {b.name} を見つけた: {np.round(b.center, 3).tolist()}（pelvis 座標 [m]）")
                return np.asarray(b.center, dtype=float)
        print(f"[rl_agent] 目標のボタン {self.task.target} が見つからない（見つかったのは {[b.name for b in buttons]}）")
        return None


def make_agent() -> Agent:
    return TeacherAgent()
