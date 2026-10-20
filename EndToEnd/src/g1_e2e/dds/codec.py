"""HG LowState/LowCmdと、SDK順29関節の物理量との相互変換。"""

from dataclasses import dataclass

import numpy as np

from g1_e2e.contracts import JointCommand
from .sdk import load_sdk


@dataclass(frozen=True)
class MotorTargets:
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    kp: np.ndarray
    kd: np.ndarray

    def effort_command(self, observation, effort_limit):
        # ゲインをゼロにしたPhysXへ、各物理stepで計算したhybrid PDの合力を渡す。
        effort = self.tau + self.kp * (self.q - observation.joint_position_rad)
        effort += self.kd * (self.dq - observation.joint_velocity_rad_s)
        return JointCommand(
            observation.joint_position_rad.copy(),
            feedforward_torque_nm=np.clip(effort, -effort_limit, effort_limit).astype(np.float32),
        )


class LowLevelCodec:
    def __init__(self, mode_machine=5):
        if not 0 <= mode_machine <= 255:
            raise ValueError("mode_machineはuint8です")
        self.sdk = load_sdk()
        self.mode_machine = mode_machine

    def decode_command(self, message):
        if not isinstance(message, self.sdk.LowCmd):
            raise ValueError("HG LowCmd型が必要です")
        if self.sdk.crc.Crc(message) != message.crc:
            raise ValueError("CRC")
        if message.mode_pr != 0:
            raise ValueError("PR座標(mode_pr=0)のみ対応しています")
        if message.mode_machine != self.mode_machine:
            raise ValueError("mode_machine")
        if any(m.mode != 0 for m in message.motor_cmd[29:]):
            raise ValueError("未実装のmotor slot 29..34")
        values = np.asarray(
            [[m.q, m.dq, m.tau, m.kp, m.kd] for m in message.motor_cmd[:29]],
            dtype=np.float32,
        )
        modes = np.asarray([m.mode for m in message.motor_cmd[:29]])
        if not np.isfinite(values).all() or not np.isin(modes, [0, 1]).all():
            raise ValueError("有限値とmotor mode 0/1が必要です")
        q, dq, tau, kp, kd = values.T.copy()
        if np.any((kp < 0) | (kp > 2000) | (kd < 0) | (kd > 100)):
            raise ValueError("PD gainの範囲外です")
        # 公式SDKの位置/速度停止値は、それぞれのPD項だけを無効化する。
        position_stop = np.isclose(q, 2.146e9, rtol=1e-6)
        velocity_stop = np.isclose(dq, 16000.0, rtol=1e-6)
        q[position_stop], kp[position_stop] = 0, 0
        dq[velocity_stop], kd[velocity_stop] = 0, 0
        if np.any(np.abs(q) > 100) or np.any(np.abs(dq) > 200):
            raise ValueError("関節位置/速度の範囲外です")
        kp[modes == 0], kd[modes == 0], tau[modes == 0] = 0, 0, 0
        for value in (q, dq, tau, kp, kd):
            value.flags.writeable = False
        return MotorTargets(q, dq, tau, kp, kd)

    def encode_command(self, command, kp, kd):
        command = command.validated()
        message = self.sdk.new_command()
        message.mode_machine = self.mode_machine
        for i, motor in enumerate(message.motor_cmd[:29]):
            motor.mode = 1
            motor.q = float(command.position_rad[i])
            motor.dq = float(command.velocity_rad_s[i])
            motor.tau = float(command.feedforward_torque_nm[i])
            motor.kp, motor.kd = float(kp[i]), float(kd[i])
        message.crc = self.sdk.crc.Crc(message)
        return message

    @staticmethod
    def _fill_imu(message, orientation_xyzw, angular_velocity, specific_force):
        x, y, z, w = orientation_xyzw
        message.quaternion = [float(w), float(x), float(y), float(z)]
        message.gyroscope = np.asarray(angular_velocity, dtype=np.float32).tolist()
        message.accelerometer = np.asarray(specific_force, dtype=np.float32).tolist()
        message.rpy = [
            float(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            float(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))),
            float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))),
        ]
        return message

    def encode_imu(self, frame):
        return self._fill_imu(
            self.sdk.new_imu(),
            frame.orientation_xyzw,
            frame.angular_velocity_sensor_rad_s,
            frame.specific_force_sensor_m_s2,
        )

    def encode_state(self, observation, specific_force=None, *, imu=None, previous=None, dt=None):
        message = self.sdk.new_state()
        message.mode_machine = self.mode_machine
        message.tick = round(observation.simulation_time_s * 1000) & 0xFFFFFFFF
        if imu is not None:
            if (
                imu.episode_id != observation.episode_id
                or abs(imu.time_s - observation.simulation_time_s) > 1e-9
            ):
                raise ValueError("LowStateとIMUのepisode・時刻が一致しません")
            message.imu_state = self.encode_imu(imu)
        else:
            if specific_force is None:
                raise ValueError("LowStateにはIMUまたは比力が必要です")
            self._fill_imu(
                message.imu_state,
                observation.orientation_xyzw,
                observation.angular_velocity_body_rad_s,
                specific_force,
            )
        for i, motor in enumerate(message.motor_state[:29]):
            motor.mode = 1
            motor.q = float(observation.joint_position_rad[i])
            motor.dq = float(observation.joint_velocity_rad_s[i])
            motor.tau_est = float(observation.joint_torque_nm[i])
            if previous is not None and dt is not None and dt > 0:
                motor.ddq = float(
                    (observation.joint_velocity_rad_s[i] - previous.joint_velocity_rad_s[i]) / dt
                )
        message.crc = self.sdk.crc.Crc(message)
        return message
