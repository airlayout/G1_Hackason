"""公開29DoF歩行モデルの検証用アダプター。480次元の仕様はこの中だけに閉じる。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from g1_e2e.contracts import JointCommand, Observation, VelocityCommand, vector
from g1_e2e.robot import RobotSpec


class UnitreeOnnxController:
    """観測termごとの時系列を連結し、policy順をSDK順へ戻す。"""

    def __init__(self, robot: RobotSpec, control_dt: float, options: dict) -> None:
        import onnxruntime as ort

        root = Path(options["root"])
        config = yaml.safe_load((root / "assets/cache/demo/deploy.yaml").read_text())
        if not np.isclose(control_dt, config["step_dt"]):
            raise ValueError("公開モデルの制御周期は0.02秒です。環境設定を合わせてください")
        self.mapping = np.asarray(config["joint_ids_map"], dtype=np.int64)
        if sorted(self.mapping.tolist()) != list(range(29)):
            raise ValueError("公開モデルのjoint_ids_mapが29関節の置換ではありません")
        self.default = vector(config["default_joint_pos"], 29, "default_joint_pos")
        if not np.allclose(robot.default_position[self.mapping], self.default):
            raise ValueError("ロボットの初期姿勢と公開モデルの初期姿勢が一致しません")
        self.action = config["actions"]["JointPositionAction"]
        self.terms = config["observations"]
        self.ranges = config["commands"]["base_velocity"]["ranges"]
        self.command = vector(options.get("velocity", [0, 0, 0]), 3, "velocity")
        self.set_velocity(self.command)
        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = 1
        session_options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            str(root / "assets/cache/demo/policy.onnx"),
            sess_options=session_options,
            providers=["CPUExecutionProvider"],
        )
        self.input_name = self.session.get_inputs()[0].name
        self.reset()

    def set_velocity(self, command: np.ndarray) -> None:
        """公開モデルが定義した範囲内に希望速度を制限する。"""
        command = vector(command, 3, "velocity")
        self.requested_velocity = command.copy()
        limits = np.asarray([self.ranges[k] for k in ["lin_vel_x", "lin_vel_y", "ang_vel_z"]])
        self.command = np.clip(command, limits[:, 0], limits[:, 1]).astype(np.float32)

    @property
    def velocity_command(self):
        return VelocityCommand(self.requested_velocity.copy(), self.command.copy())

    def reset(self) -> None:
        """履歴は最初の観測で埋める。前episodeのactionを引き継がない。"""
        self.last_action = np.zeros(29, dtype=np.float32)
        self.history: dict[str, np.ndarray] = {}

    def act(self, observation: Observation) -> JointCommand:
        """モデル固有の正規化・history・相対角度変換を適用する。"""
        values = {
            "base_ang_vel": observation.angular_velocity_body_rad_s,
            "projected_gravity": observation.projected_gravity_body,
            "velocity_commands": self.command,
            "joint_pos_rel": observation.joint_position_rad[self.mapping] - self.default,
            "joint_vel_rel": observation.joint_velocity_rad_s[self.mapping],
            "last_action": self.last_action,
        }
        flattened = []
        for name, cfg in self.terms.items():
            value = np.asarray(values[name], dtype=np.float32)
            if cfg["clip"] is not None:
                value = np.clip(value, *cfg["clip"])
            value = value * np.asarray(cfg["scale"], dtype=np.float32)
            if name not in self.history:
                self.history[name] = np.tile(value, (cfg["history_length"], 1))
            else:
                self.history[name] = np.concatenate((self.history[name][1:], value[None]), axis=0)
            flattened.append(self.history[name].reshape(-1))
        model_input = np.concatenate(flattened)[None].astype(np.float32)
        action = vector(self.session.run(None, {self.input_name: model_input})[0][0], 29, "policy action")
        self.last_action = action.copy()
        if self.action["clip"] is not None:
            action = np.clip(action, *self.action["clip"])
        position_policy = np.asarray(self.action["offset"]) + np.asarray(self.action["scale"]) * action
        position_sdk = np.empty(29, dtype=np.float32)
        position_sdk[self.mapping] = position_policy
        return JointCommand(position_sdk)


def create_controller(*, robot: RobotSpec, control_dt: float, options: dict) -> UnitreeOnnxController:
    """CLIと外部モデルで共用するfactory。"""
    return UnitreeOnnxController(robot, control_dt, options)
