"""エージェントとロボット（シミュレーター / 実機）の間の決まり。各チームはこのファイルの Agent を作る。

    from contest.interface import Action, Agent, Observation, TaskInfo, UPPER_BODY_IDX

    class MyAgent(Agent):
        def reset(self, task: TaskInfo) -> None: ...
        def act(self, obs: Observation) -> Action: ...

    def make_agent() -> Agent:
        return MyAgent()

決まり:
- 上半身（腰 3 + 左腕 7 + 右腕 7）は関節の目標の角度で動かす（実機の rt/arm_sdk と同じ）。
- 下半身は速度の指令（vx, vy, yaw_rate）で動かす（実機の LocoClient.Move と同じ）。
  シミュレーションでは今は腰（pelvis）を固定しているので、下半身の指令は無視する（TaskInfo.base_enabled）。
- エージェントはシミュレーター（mujoco / isaacsim / isaaclab）を import してはいけない。
  シミュレーションでも実機でも、同じコードのまま動かすため。
- 観測には、実機でも取れるものだけを入れる（ボタンの座標などの正解の値は渡さない）。

関節の並びは motor 番号（lowcmd / lowstate の motor_cmd[i] / motor_state[i]、0〜28）。
名前は JOINT_NAMES（J1-gen の common/robot_model.py と同じ）。
"""

from __future__ import annotations

import importlib.util
import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

JOINT_NAMES: tuple[str, ...] = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
NUM_JOINTS: int = 29
# 上半身（Action.q_target の並び）= motor 12〜28
UPPER_BODY_IDX: tuple[int, ...] = tuple(range(12, 29))
WAIST_IDX: tuple[int, ...] = (12, 13, 14)
LEFT_ARM_IDX: tuple[int, ...] = tuple(range(15, 22))
RIGHT_ARM_IDX: tuple[int, ...] = tuple(range(22, 29))

BUTTONS: tuple[str, ...] = ("up", "down")


@dataclass(frozen=True)
class TaskInfo:
    """試行の開始時に渡す情報。"""

    instruction: str  # 言葉での指示（例: 「上のボタンを押して」）
    target: str  # 同じ指示を記号で書いたもの（"up" / "down"）
    sim: str  # "mujoco" / "isaac" / "real"
    time_limit: float  # 制限時間 [秒]
    control_dt: float  # act() を呼ぶ周期 [秒]
    base_enabled: bool  # 下半身の指令（Action.base_cmd）が効くか。今のシミュレーションは False
    # 上半身の PD の強さ（UPPER_BODY_IDX の並び）。腕が重力で垂れる量の見積もりなどに使ってよい
    upper_kp: np.ndarray = field(default_factory=lambda: np.zeros(len(UPPER_BODY_IDX)))
    upper_kd: np.ndarray = field(default_factory=lambda: np.zeros(len(UPPER_BODY_IDX)))


@dataclass(frozen=True)
class Observation:
    """毎周期の観測。すべて実機でも取れるもの。"""

    rgb: np.ndarray  # (H, W, 3) uint8。頭カメラのカラー画像
    depth: np.ndarray  # (H, W) float32 [m]。頭カメラの深度（光学軸方向の距離。取れない画素は 0）
    K: np.ndarray  # (3, 3)。頭カメラの内部パラメータ（fx, fy, cx, cy）
    q: np.ndarray  # (29,) 関節の角度 [rad]
    dq: np.ndarray  # (29,) 関節の速度 [rad/s]
    t: float  # 試行の開始からの時間 [秒]
    # 腰（pelvis）の IMU の姿勢 (w, x, y, z)。実機の lowstate の imu_state.quaternion と同じ（重力の向きが分かる）
    imu_quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    # 画像を撮った時刻（試行の開始から [秒]）。画像の遅れがあると t より前になる
    image_t: float = 0.0


@dataclass
class Action:
    """エージェントの出力。"""

    q_target: np.ndarray  # (17,) 上半身の目標の角度 [rad]（UPPER_BODY_IDX の並び）
    base_cmd: np.ndarray = field(default_factory=lambda: np.zeros(3))  # (vx [m/s], vy [m/s], yaw_rate [rad/s])
    done: bool = False  # 押し終わったら True（省略すると制限時間まで続く）


class Agent(ABC):
    """各チームが作るエージェント。"""

    @abstractmethod
    def reset(self, task: TaskInfo) -> None:
        """試行の開始に 1 回呼ばれる。"""

    @abstractmethod
    def act(self, obs: Observation) -> Action:
        """control_dt ごとに呼ばれる。"""

    def close(self) -> None:
        """すべての試行の終わりに 1 回呼ばれる（モデルの解放など）。"""


def load_agent(path: str | Path) -> Agent:
    """agent.py を読み込み、make_agent() でエージェントを作る。

    agent.py のあるフォルダを import の検索先に足すので、同じフォルダの別ファイルを import できる。
    """
    p = Path(path).resolve()
    if p.is_dir():
        p = p / "agent.py"
    if not p.exists():
        raise FileNotFoundError(f"エージェントが無い: {p}")
    sys.path.insert(0, str(p.parent))
    spec = importlib.util.spec_from_file_location(f"contest_agent_{p.parent.name}", p)
    if spec is None or spec.loader is None:
        raise ImportError(f"エージェントを読み込めない: {p}")
    module = importlib.util.module_from_spec(spec)
    # 実行の前に登録する（登録しないと、agent.py の中の dataclass が自分のモジュールを見つけられずに失敗する）
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    if not hasattr(module, "make_agent"):
        raise AttributeError(f"{p} に make_agent() が無い")
    agent = module.make_agent()
    if not isinstance(agent, Agent):
        raise TypeError(f"make_agent() が Agent を返さない: {type(agent)}")
    return agent
