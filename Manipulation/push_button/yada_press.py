"""Yada採点用の押下指令と画像による点灯確認。ボタンの真の沈み量は使わない。"""

from __future__ import annotations

import numpy as np

from alignment import JointCommandFilter
from trajectory import validate_arm_q

PRESS_COMMAND_DEPTH_M = 0.015
RETRY_EXTRA_DEPTH_M = 0.010
MAX_COMMAND_DEPTH_M = 0.030
MAX_RETRIES = 1


class PressJointCommandFilter(JointCommandFilter):
    """速度・加速度を制限した減衰付き追従。目標付近で早めに減速する。

    IK目標が姿勢補正で動くと、位置差/dtを追う既存フィルタでは減速が遅れ、
    横方向の補正とともに振動が増えた。Yada用だけ臨界減衰の追従にする。
    """

    def next_goal(self, goal, dt):
        goal = np.array(validate_arm_q(np.asarray(goal).tolist()))
        if not np.isfinite(dt) or not 0 < dt <= 0.10:
            raise ValueError("関節ストリームの時間刻みが不正です")
        steps = int(np.ceil(dt / self.settings.period_s))
        h = dt / steps
        omega = 12.0
        for _ in range(steps):
            acceleration = omega**2 * (goal - self.position) - 2 * omega * self.velocity
            acceleration = np.clip(acceleration, -self.settings.joint_acceleration_rad_s2,
                                   self.settings.joint_acceleration_rad_s2)
            self.velocity = np.clip(self.velocity + acceleration * h,
                                    -self.settings.joint_speed_rad_s,
                                    self.settings.joint_speed_rad_s)
            self.position = np.array(validate_arm_q((self.position + self.velocity * h).tolist()))
        return self.position.copy()


class PressCommandController:
    """奥の指令目標で押す力を作る。推定押込量だけで押下完了にはしない。

    J1-genの押下・FK補正を参考にする。座標は機体の向きを固定した同じ基準。
    実際のボタンストロークと、弱いPDへ送る指令の深さを分ける。
    """

    def __init__(self, face, normal, radius, start_xyz, command_depth_m=PRESS_COMMAND_DEPTH_M):
        self.face, self.normal = np.array(face, dtype=float), np.array(normal, dtype=float)
        self.start_xyz = np.array(start_xyz, dtype=float)
        if (any(p.shape != (3,) for p in (self.face, self.normal, self.start_xyz))
                or not np.all(np.isfinite([*self.face, *self.normal, *self.start_xyz, radius, command_depth_m]))
                or not np.isclose(np.linalg.norm(self.normal), 1.0)
                or not 0 < radius < 0.04 or not 0 < command_depth_m <= MAX_COMMAND_DEPTH_M):
            raise ValueError("採点用の押下指令の位置・方向・深さが不正です")
        self.radius, self.command_depth_m = radius, command_depth_m
        self.end_xyz = self.start_xyz + (command_depth_m - self.penetration(self.start_xyz)) * self.normal
        # smoothstepの最大速度は移動量の1.5倍/時間。25mm/sを超えない。
        self.duration_s = max(5.0, 1.5 * np.linalg.norm(self.end_xyz - self.start_xyz) / 0.025)
        self.elapsed_s = 0.0
        self.command_xyz = self.start_xyz.copy()
        self.correction = np.zeros(3)
        self.correction_frozen = False

    def penetration(self, tip):
        tip = np.asarray(tip, dtype=float)
        if tip.shape != (3,) or not np.all(np.isfinite(tip)):
            raise ValueError("手先位置が不正です")
        return float((tip - self.face) @ self.normal + self.radius)

    def next_goal(self, tip, dt):
        if not np.isfinite(dt) or not 0 < dt <= 0.10:
            raise ValueError("連続押下の状態更新が遅すぎる、または時刻が不正です")
        tip = np.asarray(tip, dtype=float)
        depth = self.penetration(tip)
        error = self.command_xyz - tip
        axial = float(error @ self.normal)
        if np.linalg.norm(error - axial * self.normal) > 0.01:
            raise ValueError("押下指令と手先の横方向の差が1cmを超えました")
        # 接触で残る指令と実測の差は、ボタンの推定ストロークとは別に制限する。
        if abs(axial) > self.command_depth_m + 0.010:
            raise ValueError("押下指令と手先の法線方向の差が許容量を超えました")
        self.elapsed_s += dt
        s = min(1.0, self.elapsed_s / self.duration_s)
        s = s*s*(3 - 2*s)
        nominal = (1 - s) * self.start_xyz + s * self.end_xyz
        if depth >= -0.010:
            self.correction_frozen = True
        if not self.correction_frozen:
            lateral = nominal - tip
            lateral -= float(lateral @ self.normal) * self.normal
            self.correction += 3.0 * dt * lateral
            size = np.linalg.norm(self.correction)
            if size > 0.01:
                self.correction *= 0.01 / size
        # 法線方向を積分補償すると、接触の反力を位置誤差と取り違えるので補償しない。
        self.command_xyz = nominal + self.correction
        if self.elapsed_s >= self.duration_s + 0.6:
            return None  # 保持が終わっただけ。成功は後退後の画像で確認する。
        return self.command_xyz.copy()


class ButtonLampCheck:
    """再計測した対象枠の明るさを、新しい画像で比較する。"""

    def __init__(self, rgb, bbox, image_t):
        values = np.asarray(bbox, dtype=float)
        if values.shape != (4,) or not np.all(np.isfinite(values)):
            raise ValueError("点灯確認用のボタン枠が不正です")
        self.bbox = tuple(values)
        self.baseline = self._brightness(rgb)
        self.reset_frames(image_t)

    def reset_frames(self, after):
        self.after, self.last_image_t = after, after
        self.checked_frames = self.bright_frames = 0

    def _brightness(self, rgb):
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3 or not np.all(np.isfinite(image)):
            raise ValueError("点灯確認用のRGB画像が不正です")
        x1, y1 = np.floor(self.bbox[:2]).astype(int)
        x2, y2 = np.ceil(self.bbox[2:]).astype(int)
        if x1 < 0 or y1 < 0 or x2 > image.shape[1] or y2 > image.shape[0] or min(x2-x1, y2-y1) < 6:
            raise ValueError("点灯確認用のボタン枠が画像外または小さすぎます")
        return float(image[y1:y2, x1:x2].mean())

    def observe(self, rgb, t, image_t):
        if not np.isfinite(image_t) or not np.isfinite(t) or not 0 <= t - image_t <= 0.5:
            raise ValueError("点灯確認の画像が古すぎる、または撮影時刻が不正です")
        if image_t <= self.last_image_t:
            return None
        self.last_image_t = image_t
        bright = self._brightness(rgb) - self.baseline >= 25.0
        self.checked_frames += 1
        self.bright_frames = self.bright_frames + 1 if bright else 0
        if self.bright_frames >= 3:
            return True
        if self.checked_frames >= 3 and not bright:
            return False
        return None
