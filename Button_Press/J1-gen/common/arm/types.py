"""腕の指令部分で使うデータの型。"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..robot_model import NUM_MOTORS


@dataclass
class JointState:
    """lowstate から読んだ関節の状態（29関節、motor 番号順）。"""

    q: np.ndarray
    dq: np.ndarray
    # motor_state[i].mode。0 はゼロトルク（何を送っても動かない）、1 は有効
    motor_mode: np.ndarray
    mode_machine: int
    # 受信した時刻（time.monotonic()）
    stamp: float
    # IMU の姿勢（四元数 w, x, y, z）。重力補償で pelvis の傾きを知るのに使う
    imu_quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    # 模擬ロボット（sim/sim_robot_server.py）が送った lowstate か（reserve[0] の目印）
    sim_marker: bool = False


@dataclass
class JointCommand:
    """1 回分の指令（29関節、motor 番号順）。

    各関節のトルクは τ = kp(q − q_実測) + kd(0 − dq_実測) + tau で決まる（PD 制御）。
    kp = kd = 0 の関節は「指令なし」として扱う。
    """

    q: np.ndarray = field(default_factory=lambda: np.zeros(NUM_MOTORS))
    kp: np.ndarray = field(default_factory=lambda: np.zeros(NUM_MOTORS))
    kd: np.ndarray = field(default_factory=lambda: np.zeros(NUM_MOTORS))
    tau: np.ndarray = field(default_factory=lambda: np.zeros(NUM_MOTORS))
    # arm_sdk の weight（motor_cmd[29].q）。0 で内蔵コントローラ、1 でこの指令。lowcmd では使わない
    weight: float = 0.0

    def copy(self) -> "JointCommand":
        return JointCommand(self.q.copy(), self.kp.copy(), self.kd.copy(), self.tau.copy(), self.weight)
