"""RGB-D目標の検査、1回再計測後のFK追従、反復再計測の制御器。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Callable

import numpy as np

from vision import ButtonTarget, validate_reachable_target
from trajectory import validate_arm_q, validate_button_stroke


@dataclass(frozen=True)
class CorrectionTiming:
    """実機・模擬機で共通の補正時間。intervalは移動開始同士の最小間隔。"""

    interval_s: float = 2.0
    move_s: float = 0.6
    settle_s: float = 0.6
    phase_timeout_s: float = 60.0

    def __post_init__(self):
        values = (self.interval_s, self.move_s, self.settle_s, self.phase_timeout_s)
        if (not all(np.isfinite(values)) or min(values) <= 0
                or self.interval_s < self.move_s + self.settle_s
                or self.phase_timeout_s <= self.interval_s):
            raise ValueError("補正間隔は移動時間＋静止待ち以上、段階の制限時間未満が必要です")

    @classmethod
    def from_args(cls, args):
        return cls(args.correction_interval, args.correction_duration,
                   args.settle_time, args.phase_timeout)


def add_correction_arguments(parser):
    parser.add_argument("--alignment-mode", choices=("single", "iterative"), default="single",
                        help="single: 手前で1回だけ再計測。iterative: 従来の反復再計測")
    parser.add_argument("--normal-motion", choices=("smooth", "stepped"), default="smooth",
                        help="singleの接近・押下。smooth: 連続移動、stepped: 従来の区間ごとの停止")
    parser.add_argument("--remeasure-settle", type=float, default=0.3,
                        help="singleの撮り直し前の静止待ち [s]")
    parser.add_argument("--feedback-timeout", type=float, default=20.0,
                        help="singleのFK位置合わせ・接近・押下それぞれの制限時間 [s]")
    defaults = CorrectionTiming()
    parser.add_argument("--correction-interval", type=float, default=defaults.interval_s,
                        help="iterativeの補正開始同士の最小間隔 [s] (既定: 2)")
    parser.add_argument("--correction-duration", type=float, default=defaults.move_s,
                        help="iterativeの1回の補正移動時間 [s]。関節速度制限で延長する")
    parser.add_argument("--settle-time", type=float, default=defaults.settle_s,
                        help="iterativeの補正移動後、再計測までの静止待ち [s]")
    parser.add_argument("--phase-timeout", type=float, default=defaults.phase_timeout_s,
                        help="iterativeの位置合わせ・接近・押下それぞれの制限時間 [s]")


class CorrectionCadence:
    """待機を画像取得より先に行う。遅れた分の補正を連続実行しない。"""

    def __init__(self, timing: CorrectionTiming, now: Callable[[], float],
                 wait: Callable[[float], None]):
        self.timing, self.now, self.wait = timing, now, wait
        self.phase_started_at = now()
        self.last_started_at = None

    def check_timeout(self):
        current = self.now()
        if not np.isfinite(current) or current < self.phase_started_at:
            raise ValueError("補正時計が不正です")
        if current - self.phase_started_at >= self.timing.phase_timeout_s:
            raise ValueError("補正が段階の制限時間内に収束しません")

    def wait_for_next(self):
        self.check_timeout()
        if self.last_started_at is not None:
            delay = max(0.0, self.last_started_at + self.timing.interval_s - self.now())
            if self.now() + delay - self.phase_started_at >= self.timing.phase_timeout_s:
                raise ValueError("補正が段階の制限時間内に収束しません")
            if delay > 0:
                self.wait(delay)
        self.check_timeout()

    def start(self):
        """画像検査・IK・経路検査後、実際の移動開始直前に呼ぶ。"""
        self.check_timeout()
        current = self.now()
        if (self.last_started_at is not None
                and current - self.last_started_at < self.timing.interval_s - 1e-9):
            raise ValueError("補正開始の間隔が短すぎます")
        self.last_started_at = current
        return current


@dataclass(frozen=True)
class Observation:
    target: ButtonTarget
    captured_at: float
    sequence: int


@dataclass(frozen=True)
class SingleCorrectionTiming:
    capture_settle_s: float = 0.3
    retarget_move_s: float = 1.0
    feedback_move_s: float = 0.2
    feedback_settle_s: float = 0.1
    phase_timeout_s: float = 20.0

    def __post_init__(self):
        values = tuple(vars(self).values())
        if not all(np.isfinite(values)) or min(values) <= 0 or self.phase_timeout_s <= self.retarget_move_s:
            raise ValueError("1回再計測の待機・移動・制限時間が不正です")

    @classmethod
    def from_args(cls, args):
        return cls(capture_settle_s=getattr(args, "remeasure_settle", 0.3),
                   phase_timeout_s=getattr(args, "feedback_timeout", 20.0))


class FixedTargetAlignment:
    """1回の新しい計測で目標を固定し、その後は関節角のFKだけで追従を確認する。"""

    tolerance_m = 0.003

    def __init__(self, observation, tip, radius, clearance, now):
        if not 0 <= now - observation.captured_at <= 0.5:
            raise ValueError("ボタン計測が古い、または時刻が不正です")
        validate_reachable_target(observation.target)
        self.face = np.array(observation.target.face_xyz)
        self.normal = np.array(observation.target.press_direction)
        self.radius = radius
        self.goal = self.face - (radius + clearance) * self.normal
        tip = self._check_tip(tip)
        if np.linalg.norm(self.goal - tip) > 0.08:
            raise ValueError("1回の位置補正が8cmを超えました")
        # 更新した実測姿勢でIKを解き直す。接近前の指令誤差は積分へ持ち越さない。
        self.command_xyz = self.goal.copy()
        self.settled = 0
        self.iterations = 0

    def _check_tip(self, tip):
        tip = np.asarray(tip, dtype=float)
        if tip.shape != (3,) or not np.all(np.isfinite(tip)):
            raise ValueError("手先位置が不正です")
        if float((self.face - tip) @ self.normal - self.radius) < 0.008:
            raise ValueError("接触に近すぎるため位置補正を中止します")
        return tip

    def next_goal(self, tip):
        tip = self._check_tip(tip)
        self.iterations += 1
        if self.iterations > 80:
            raise ValueError("固定目標への追従が収束しません")
        error = self.goal - tip
        distance = float(np.linalg.norm(error))
        self.settled = self.settled + 1 if distance <= self.tolerance_m else 0
        if self.settled >= 3:
            return None
        if distance <= self.tolerance_m:
            return self.command_xyz.copy()
        if np.linalg.norm(self.command_xyz - tip) > 0.01:
            raise ValueError("手先の指令と実測の差が1cmを超えました")
        candidate = self.command_xyz + error * min(1.0, 0.005 / distance)
        if np.linalg.norm(candidate - tip) > 0.01:
            raise ValueError("手先の指令と実測の差が1cmを超えました")
        self.command_xyz = candidate
        return candidate.copy()


def align_fixed_target(observation, radius, clearance, timing,
                       now, wait, read_tip, move):
    """moveはIK/経路検査と移動を担当する。画像取得は呼び出し元の1回だけ。"""
    started = now()
    servo = FixedTargetAlignment(observation, read_tip(), radius, clearance, now())
    initial_error = float(np.linalg.norm(servo.goal - read_tip()))
    move(servo.command_xyz, timing.retarget_move_s, timing.capture_settle_s)
    feedback_steps = []
    while True:
        if now() - started >= timing.phase_timeout_s:
            raise ValueError("固定目標への追従が制限時間内に収束しません")
        tip = read_tip()
        goal = servo.next_goal(tip)
        if goal is None:
            return servo, {"initial_error_m": initial_error,
                           "final_error_m": float(np.linalg.norm(servo.goal - tip)),
                           "duration_s": now() - started, "fk_feedback_steps": feedback_steps}
        if np.linalg.norm(servo.goal - tip) <= servo.tolerance_m:
            wait(timing.feedback_settle_s)
        else:
            feedback_steps.append({"time_s": now(), "tip_goal_xyz_m": goal.tolist()})
            move(goal, timing.feedback_move_s, timing.feedback_settle_s)


class TargetTracker:
    """初回に一意に選択し、以降は近傍の同じボタンだけを採用する。"""

    def __init__(self, window: int = 3, spread_m: float = 0.004,
                 association_m: float = 0.025, max_shift_m: float = 0.08):
        self.samples = deque(maxlen=window)
        self.spread_m = spread_m
        self.association_m = association_m
        self.max_shift_m = max_shift_m
        self.initial = None
        self.previous = None
        self.sequence = 0

    def begin_measurement(self):
        """対象の同一性を保持し、静止待ち前の画像を安定性判定から除く。"""
        self.samples.clear()

    def update(self, candidates: list[ButtonTarget], captured_at: float) -> Observation | None:
        for target in candidates:
            validate_reachable_target(target)
        if self.previous is not None:
            candidates = [t for t in candidates if np.linalg.norm(
                np.array(t.face_xyz) - self.previous) <= self.association_m]
        if len(candidates) != 1:
            self.samples.clear()
            raise ValueError(f"同じボタンを一意に追跡できません (候補={len(candidates)})")
        target = candidates[0]
        position = np.array(target.face_xyz)
        if self.initial is None:
            self.initial = position.copy()
        if np.linalg.norm(position - self.initial) > self.max_shift_m:
            raise ValueError("ボタン目標の移動が補正上限を超えました")
        self.previous = position
        self.samples.append(target)
        self.sequence += 1
        if len(self.samples) < self.samples.maxlen:
            return None
        positions = np.array([t.face_xyz for t in self.samples])
        center = np.median(positions, axis=0)
        if np.max(np.linalg.norm(positions - center, axis=1)) > self.spread_m:
            return None
        normals = np.array([t.press_direction for t in self.samples])
        normal = np.median(normals, axis=0)
        normal /= np.linalg.norm(normal)
        if np.min(normals @ normal) < np.cos(np.deg2rad(5)):
            return None
        stable = ButtonTarget(tuple(center), tuple(normal), target.depth_m,
                              min(t.depth_valid_fraction for t in self.samples),
                              min(t.plane_inlier_fraction for t in self.samples))
        return Observation(stable, captured_at, self.sequence)


@dataclass(frozen=True)
class AlignmentStep:
    tip_goal: np.ndarray
    error_m: float
    lateral_error_m: float
    normal_error_m: float
    converged: bool


class AlignmentController:
    """ボタン表面と手先球中心を区別する。接触前のみ補正する。"""

    def __init__(self, tip_radius: float = 0.015, clearance: float = 0.03,
                 step_m: float = 0.005, tolerance_m: float = 0.003,
                 max_age_s: float = 0.5, max_iterations: int = 80):
        values = (tip_radius, clearance, step_m, tolerance_m, max_age_s)
        if not all(np.isfinite(values)) or min(values) <= 0 or max_iterations < 1:
            raise ValueError("位置合わせの設定が不正です")
        self.tip_radius = tip_radius
        self.clearance = clearance
        self.step_m = step_m
        self.tolerance_m = tolerance_m
        self.max_age_s = max_age_s
        self.max_iterations = max_iterations
        self.iterations = 0
        self.last_sequence = -1
        self.settled = 0
        self.command_xyz = None

    def next_step(self, observation: Observation, tip_xyz: np.ndarray,
                  now: float) -> AlignmentStep:
        if (not np.isfinite(now) or not np.isfinite(observation.captured_at)
                or not 0 <= now - observation.captured_at <= self.max_age_s):
            raise ValueError("ボタン計測が古い、または時刻が不正です")
        if observation.sequence <= self.last_sequence:
            raise ValueError("同じフレームの再利用はできません")
        validate_reachable_target(observation.target)
        tip = np.asarray(tip_xyz, dtype=float)
        if tip.shape != (3,) or not np.all(np.isfinite(tip)):
            raise ValueError("手先位置が不正です")
        self.last_sequence = observation.sequence
        self.iterations += 1
        if self.iterations > self.max_iterations:
            raise ValueError("位置合わせが制限回数内に収束しません")
        face = np.array(observation.target.face_xyz)
        normal = np.array(observation.target.press_direction)
        gap = float((face - tip) @ normal - self.tip_radius)
        if gap < 0.008:
            raise ValueError("接触に近すぎるため位置補正を中止します")
        goal = face - (self.tip_radius + self.clearance) * normal
        error = goal - tip
        axial = float(error @ normal)
        lateral = error - axial * normal
        distance = float(np.linalg.norm(error))
        settled = distance <= self.tolerance_m
        self.settled = self.settled + 1 if settled else 0
        # 横方向を合わせる間はパネルへ近づかない。
        delta = (lateral if np.linalg.norm(lateral) > self.tolerance_m else error).copy()
        length = float(np.linalg.norm(delta))
        if length > self.step_m:
            delta *= self.step_m / length
        if settled:
            delta = np.zeros(3)
        if self.command_xyz is None:
            self.command_xyz = tip.copy()
        # 前回の指令位置に誤差を加算し、重力等による定常追従誤差も補正する。
        # 指令と実測の差を1cm以内に制限して積分の蓄積を抑える。
        offset = self.command_xyz - tip
        if np.linalg.norm(offset) > 0.01:
            raise ValueError("手先の指令と実測の差が1cmを超えました")
        candidate = self.command_xyz + delta
        if np.linalg.norm(candidate - tip) > 0.01:
            raise ValueError("手先の指令と実測の差が1cmを超えました")
        self.command_xyz = candidate
        return AlignmentStep(self.command_xyz.copy(), distance, float(np.linalg.norm(lateral)),
                             abs(axial), self.settled >= 3)


def normal_completion_tolerance(penetration_m, single=False):
    # 反復方式は最大1mm。1回再計測方式は最大0.1mm、短いストロークでは1/30。
    maximum = 0.0001 if single else 0.001
    divisor = 30 if single else 8
    return min(maximum, penetration_m / divisor) if penetration_m > 0 else 0.0001


def press_overtravel_allowance(stroke):
    return min(0.001, stroke / 10) if stroke > 0 else 0.001


def press_compensation_allowance(stroke):
    # 重力・接触による指令とFKの差はストロークに比例しない。
    # 小ストロークでも3mmは確保し、実測の過押しは別の上限で止める。
    return min(0.004, max(0.003, stroke)) if stroke > 0 else 0.004


class NormalMotionController:
    """接触直前・押下の定常追従誤差を、法線方向だけで補正する。"""

    def __init__(self, face, normal, tip_radius, penetration_m, command_xyz, single=False):
        self.face = np.array(face)
        self.normal = np.array(normal)
        self.tip_radius = tip_radius
        self.penetration_m = penetration_m
        self.overtravel_m = press_overtravel_allowance(penetration_m)
        self.compensation_m = press_compensation_allowance(penetration_m)
        self.single = single
        # 接触直前の3mmの空隙は両方式とも0.1mmまで確認する。
        self.completion_tolerance_m = normal_completion_tolerance(penetration_m, single)
        self.command_xyz = np.array(command_xyz).copy()
        self.iterations = 0

    def next_goal(self, tip_xyz):
        penetration = float((np.asarray(tip_xyz) - self.face) @ self.normal + self.tip_radius)
        if penetration > max(0.0, self.penetration_m + self.overtravel_m):
            raise ValueError("手先の押込量が設定値を超えました")
        error = self.penetration_m - penetration
        if abs(error) <= self.completion_tolerance_m:
            return None
        self.iterations += 1
        if self.iterations > 80:
            raise ValueError("法線方向の追従が収束しません")
        # 短いストロークで1mmずつ押すと、静止待ち中の過渡的な過押しが大きい。
        near_step = (min(0.0005, self.penetration_m / 8) if self.single
                     else self.completion_tolerance_m) if self.penetration_m > 0 else 0.001
        step = 0.004 if abs(error) > 0.010 else near_step
        candidate = self.command_xyz + np.clip(error, -step, step) * self.normal
        command_penetration = float((candidate - self.face) @ self.normal + self.tip_radius)
        if command_penetration - self.penetration_m > self.compensation_m:
            # 上限をまたぐ最後の補正を残りの許容量に切り詰める。
            # 上限へ到達しても追従しない場合は、それ以上積分せず中止する。
            candidate -= (command_penetration - self.penetration_m - self.compensation_m) * self.normal
            if float((candidate - self.command_xyz) @ self.normal) <= 1e-6:
                raise ValueError(f"押下指令の追従補償が{self.compensation_m*1000:g}mmを超えました")
        self.command_xyz = candidate
        return self.command_xyz.copy()


@dataclass(frozen=True)
class SmoothNormalSettings:
    period_s: float = 0.02
    approach_speed_m_s: float = 0.025
    close_speed_m_s: float = 0.005
    press_speed_m_s: float = 0.003
    acceleration_m_s2: float = 0.04
    completion_tolerance_m: float = 0.0001
    completion_hold_s: float = 0.12
    joint_speed_rad_s: float = 0.8
    joint_acceleration_rad_s2: float = 2.0

    def __post_init__(self):
        values = tuple(vars(self).values())
        if not all(np.isfinite(values)) or min(values) <= 0 or self.period_s > 0.10:
            raise ValueError("連続押下の速度・加速度・時間設定が不正です")


class ContinuousNormalController:
    """法線方向に連続して進め、FK誤差から速度を調整する。撮影・区間停止は行わない。"""

    def __init__(self, face, normal, radius, stroke, command_xyz, settings=None):
        self.settings = settings or SmoothNormalSettings()
        self.face, self.normal = np.asarray(face, dtype=float), np.asarray(normal, dtype=float)
        if (self.face.shape != (3,) or self.normal.shape != (3,)
                or not np.all(np.isfinite([*self.face, *self.normal, radius, stroke]))
                or not np.isclose(np.linalg.norm(self.normal), 1.0)
                or not 0 < radius < 0.04):
            raise ValueError("連続押下の目標・寸法が不正です")
        self.radius, self.stroke = radius, validate_button_stroke(stroke)
        self.overtravel_m = press_overtravel_allowance(stroke)
        self.compensation_m = press_compensation_allowance(stroke)
        self.completion_tolerance_m = min(self.settings.completion_tolerance_m, stroke / 30)
        self.command_xyz = np.array(command_xyz, dtype=float).copy()
        self.velocity = 0.0
        self.settled_s = 0.0

    def penetration(self, tip):
        tip = np.asarray(tip, dtype=float)
        if tip.shape != (3,) or not np.all(np.isfinite(tip)):
            raise ValueError("手先位置が不正です")
        return float((tip - self.face) @ self.normal + self.radius)

    def next_goal(self, tip, dt):
        if not np.isfinite(dt) or not 0 < dt <= 0.10:
            raise ValueError("連続押下の状態更新が遅すぎる、または時刻が不正です")
        penetration = self.penetration(tip)
        if penetration > self.stroke + self.overtravel_m:
            raise ValueError("手先の押込量が設定値を超えました")
        if np.linalg.norm(self.command_xyz - np.asarray(tip)) > 0.01:
            raise ValueError("手先の指令と実測の差が1cmを超えました")
        error = self.stroke - penetration
        settings = self.settings
        if penetration < -0.005:
            speed = float(np.interp(-penetration, [0.005, 0.020],
                                    [settings.close_speed_m_s, settings.approach_speed_m_s]))
        else:
            # 小ストローク用の低速は押下付近で使い、空隙5mm全体をその速度で進まない。
            speed = float(np.interp(-penetration, [0.0, 0.005],
                                    [min(settings.press_speed_m_s, self.stroke / 2),
                                     settings.close_speed_m_s]))
        desired_velocity = float(np.clip(3.0 * error, -speed, speed))
        if abs(error) <= self.completion_tolerance_m:
            desired_velocity = 0.0
            self.settled_s += dt
        else:
            self.settled_s = 0.0
        self.velocity += float(np.clip(desired_velocity - self.velocity,
                                        -settings.acceleration_m_s2 * dt,
                                        settings.acceleration_m_s2 * dt))
        if self.settled_s >= settings.completion_hold_s and abs(self.velocity) < 0.0002:
            return None
        candidate = self.command_xyz + self.velocity * dt * self.normal
        depth = self.penetration(candidate)
        if depth > self.stroke + self.compensation_m:
            candidate -= (depth - self.stroke - self.compensation_m) * self.normal
            if np.linalg.norm(candidate - self.command_xyz) < 1e-8:
                raise ValueError(f"押下指令の追従補償が{self.compensation_m*1000:g}mmを超えました")
        self.command_xyz = candidate
        return candidate.copy()


class JointCommandFilter:
    """ストリーム指令の関節速度・加速度を制限し、IK解の変化をそのまま送らない。"""

    def __init__(self, initial, settings=None):
        self.settings = settings or SmoothNormalSettings()
        self.position = np.array(validate_arm_q(np.asarray(initial).tolist()))
        self.velocity = np.zeros(7)

    def next_goal(self, goal, dt):
        goal = np.array(validate_arm_q(np.asarray(goal).tolist()))
        if not np.isfinite(dt) or not 0 < dt <= 0.10:
            raise ValueError("関節ストリームの時間刻みが不正です")
        desired = np.clip((goal - self.position) / dt,
                          -self.settings.joint_speed_rad_s, self.settings.joint_speed_rad_s)
        self.velocity += np.clip(desired - self.velocity,
                                 -self.settings.joint_acceleration_rad_s2 * dt,
                                 self.settings.joint_acceleration_rad_s2 * dt)
        self.position = np.array(validate_arm_q((self.position + self.velocity * dt).tolist()))
        return self.position.copy()


def run_continuous_normal(servo, now, wait, read_tip, send_goal, timeout_s, on_step):
    """send_goalはIK・経路検査・指令更新。待機は次の50Hz更新までだけで停止区間を作らない。"""
    started = previous = now()
    first = True
    while True:
        stamp = now()
        if stamp - started >= timeout_s:
            raise ValueError("連続押下が制限時間内に収束しません")
        dt = servo.settings.period_s if first else stamp - previous
        first, previous = False, stamp
        tip = read_tip()
        goal = servo.next_goal(tip, dt)
        if goal is None:
            return {"duration_s": now() - started,
                    "final_penetration_m": servo.penetration(tip)}
        phase = "press" if servo.penetration(goal) >= -0.003 else "contact"
        actual_goal = send_goal(goal, dt, phase)
        on_step({"phase": phase, "time_s": stamp, "motion_started_s": stamp,
                 "dt_s": dt,
                 "tip_goal_xyz_m": goal.tolist(), "tip_xyz_m": tip.tolist(),
                 "command_speed_m_s": servo.velocity, "joint_goal_rad": np.asarray(actual_goal).tolist()})
        wait(max(0.0, servo.settings.period_s - (now() - stamp)))
