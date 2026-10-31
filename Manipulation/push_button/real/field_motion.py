"""現場試行の監視。保持中も50Hz監視を続け、通常の変動を恒久エラーにしない。"""
from __future__ import annotations

import math
import os
import select
import sys
import threading
import time

import numpy as np

from align_button import ArmMotion
from push_button import CONTROL_HZ, MAX_JOINT_SPEED, TRACKING_LIMIT
from push_button import check_speed
from trajectory import JOINT_LIMITS, interpolate, validate_arm_q
from trial_policy import TrialPause


class FieldMonitor:
    def __init__(self, policy, configuration, *, now=time.monotonic):
        self.policy, self.now = policy, now
        self.lock = threading.RLock()
        self.events, self.references = [], []
        self.pending = None
        self.bad_since = None
        self.stable_since = None
        self.latest = None
        self._validate(configuration)
        self._reference(configuration)

    def _validate(self, configuration):
        q, imu, stamp, leg, gyro = configuration
        if (len(q) != 29 or len(imu) != 3 or not np.all(np.isfinite([*q, *imu, stamp, leg, gyro]))
                or leg < 0 or gyro < 0):
            raise RuntimeError('実機状態に不正な値があります')
        if not 0 <= self.now()-stamp <= .1:
            raise RuntimeError('押下制御の状態が100msより古い、または時刻が不正です')
        if any(not lo <= angle <= hi for angle, (lo, hi) in zip(q[22:29], JOINT_LIMITS)):
            raise RuntimeError('腕の実測関節角が関節限界の外です')

    def _reference(self, configuration):
        self.waist = np.array(configuration[0][12:15], copy=True)
        self.imu = np.array(configuration[1], copy=True)
        self.references.append({'time_s': self.now(), 'waist_q_rad': self.waist.tolist(),
                                'received_at_s': float(configuration[2]), 'imu_rpy_rad': self.imu.tolist()})

    def event(self, kind, **details):
        self.events.append({'time_s': self.now(), 'event': kind, **details})

    def update(self, configuration):
        with self.lock:
            self._validate(configuration)
            if self.latest is not None and configuration[2] < self.latest[2]:
                return
            self.latest = configuration
            now = self.now()
            q, imu, _, leg, gyro = configuration
            moving = leg > self.policy.leg_speed_rad_s or gyro > self.policy.gyro_rad_s
            if moving:
                self.stable_since = None
                if self.bad_since is None:
                    self.bad_since = now
                    self.pause('脚または骨盤の動きを検出したため保持します', code='body_motion', automatic=True)
                if self.pending and now-self.bad_since >= self.policy.motion_persistence_s:
                    if self.pending.automatic:
                        self.event('operator_wait', code='body_motion')
                    self.pending.automatic = False
            else:
                self.bad_since = None
                if self.stable_since is None:
                    self.stable_since = now
            waist_delta = np.max(np.abs(np.array(q[12:15])-self.waist))
            angles = np.array(imu[:2])-self.imu[:2]
            imu_delta = np.max(np.abs(np.arctan2(np.sin(angles), np.cos(angles))))
            if waist_delta > math.radians(self.policy.waist_delta_deg) or imu_delta > math.radians(self.policy.imu_delta_deg):
                self.pause('試行の監視基準から姿勢が変化しました。現在姿勢を確認してください', code='pose')

    def pause(self, reason, *, code='measurement', automatic=False):
        with self.lock:
            if self.pending is None:
                self.pending = TrialPause(reason, code=code, automatic=automatic)
                self.event('hold', reason=reason, code=code, automatic=automatic)
            elif not automatic:
                self.pending.automatic = False
            return self.pending

    def stable(self):
        with self.lock:
            if self.latest is None:
                return False
            self._validate(self.latest)
            return self.stable_since is not None and self.now()-self.stable_since >= self.policy.stable_hold_s

    def resume(self, *, rebase=False):
        with self.lock:
            if not self.stable():
                raise TrialPause(f'脚・骨盤が{self.policy.stable_hold_s*1000:g}ms安定するまで待ってください',
                                 code='body_motion')
            if rebase:
                self._reference(self.latest)
            self.event('resume', rebase=rebase)
            self.pending = None


class FieldArmMotion(ArmMotion):
    def __init__(self, arm, calibration, *, monitor, gravity=None, state_sink=None):
        self.monitor = monitor
        self.enabled = False
        self.holding = False
        self.hold_goal = None
        self.holding_pause = None
        self.last_sent_target = None
        self.last_sent_at = None
        super().__init__(arm, calibration, gravity=gravity, state_sink=state_sink)
        # YES入力や計測待ちの間も状態を監視する。enableまでは指令を出さない。
        self.thread.start()

    def begin_trial(self):
        """YES後の現在の両腕を保持姿勢にする。確認待ち中の手動調整を反映する。"""
        self.fatal_check()
        if self.enabled:
            raise RuntimeError('有効化後は開始姿勢を取り直せません')
        configuration = self.arm.current_configuration()
        self.monitor.update(configuration)
        self.arm.set_hold(tuple(configuration[0][15:29]))
        with self.lock, self.monitor.lock:
            self.home = tuple(configuration[0][22:29])
            self.target = self.home
            self.target_velocity = np.zeros(7)
            self.holding, self.hold_goal, self.holding_pause = False, None, None
            self.monitor._reference(configuration)
            if self.monitor.stable():
                self.monitor.pending = None
            self.monitor.event('trial_start')

    def fatal_check(self):
        if self.error is None:
            try:
                with self.monitor.lock:
                    if self.monitor.latest is not None:
                        self.monitor._validate(self.monitor.latest)
            except RuntimeError as exc:
                self.error = exc
                self.monitor.event('fault', reason=str(exc))
                self.stop.set()
        if self.error is not None:
            raise RuntimeError(f'腕の状態監視で中止: {self.error}') from self.error

    def _run(self):
        bad_since = None
        next_tick = time.monotonic()
        try:
            while not self.stop.is_set():
                configuration = self.arm.current_configuration()
                if self.state_sink is not None:
                    self.state_sink(configuration)
                self.monitor.update(configuration)
                with self.lock, self.monitor.lock:
                    paused = self.monitor.pending is not None
                    if paused and self.holding_pause is not self.monitor.pending:
                        self.hold_goal = np.array(configuration[0][22:29], copy=True)
                        self.holding_pause = self.monitor.pending
                        self.target_velocity = np.zeros(7)
                    if paused:
                        self._hold_step()
                    self.holding = paused
                    if not paused:
                        self.holding_pause = None
                    target, weight = np.array(self.target), self.weight
                    velocity = self.target_velocity.copy()
                tracking = np.max(np.abs(np.array(configuration[0][22:29])-target))
                if tracking > TRACKING_LIMIT:
                    bad_since = bad_since or time.monotonic()
                    if time.monotonic()-bad_since > .5:
                        raise RuntimeError('腕の追従誤差が継続しています')
                else:
                    bad_since = None
                if self.enabled and not self.stop.is_set():
                    self.publish(configuration, target, weight, velocity=velocity)
                    with self.lock:
                        self.last_sent_target, self.last_sent_at = target.copy(), time.monotonic()
                next_tick += 1/CONTROL_HZ
                self.stop.wait(max(0., next_tick-time.monotonic()))
        except Exception as exc:
            self.error = exc
            self.monitor.event('fault', reason=str(exc))
            self.stop.set()
        finally:
            try:
                self.arm.release()
            except Exception as exc:
                self.release_error = str(exc)

    def check(self):
        self.fatal_check()
        with self.monitor.lock:
            if self.monitor.pending is not None:
                raise self.monitor.pending

    def enable(self):
        self.enabled = True
        start = self.weight
        for step in range(101):
            with self.lock, self.monitor.lock:
                self.check()
                self.weight = start+(1-start)*step/100
            self.wait(.02)

    def stream(self, goal, *, velocity=None):
        goal = validate_arm_q(np.asarray(goal).tolist())
        with self.lock, self.monitor.lock:
            self.check()
            self.target_velocity = ((np.array(goal)-self.target)*CONTROL_HZ
                                    if velocity is None else np.array(velocity))
            self.target = goal
        if self.record_approach:
            self.approach.append(goal, 1/CONTROL_HZ)

    def move(self, goal, duration, settle_s=.1):
        goal = validate_arm_q(np.asarray(goal).tolist())
        start = self.current_target()
        duration = max(duration, 1.5*max(abs(a-b) for a, b in zip(start, goal))/.8)
        check_speed(tuple(start), goal, duration)
        previous = start
        for target in interpolate(start, goal, duration, CONTROL_HZ):
            self.stream(target, velocity=(np.array(target)-previous)*CONTROL_HZ)
            previous = np.array(target)
            self.wait(1/CONTROL_HZ)
        with self.lock, self.monitor.lock:
            self.check()
            self.target_velocity = np.zeros(7)
        self.wait(settle_s)

    def hold(self, pause):
        self.fatal_check()
        self.monitor.pause(str(pause), code=pause.code, automatic=pause.automatic)
        # checkと次の50Hz tickの間にも古い補間指令を書き進めない。
        configuration = self.arm.current_configuration()
        self.monitor.update(configuration)
        with self.lock:
            if self.holding_pause is not self.monitor.pending:
                self.hold_goal = np.array(configuration[0][22:29], copy=True)
                self.holding_pause = self.monitor.pending
            # 追従遅れがあっても、SDKブリッジの速度制限を超えて実測角へ跳ばさない。
            self._hold_step()
            self.target_velocity = np.zeros(7)
            self.holding = True

    def _hold_step(self):
        previous = np.array(self.target if self.last_sent_target is None else self.last_sent_target)
        dt = 1/CONTROL_HZ if self.last_sent_at is None else max(0., min(1/CONTROL_HZ, time.monotonic()-self.last_sent_at))
        self.target = previous+np.clip(self.hold_goal-previous, -MAX_JOINT_SPEED*dt, MAX_JOINT_SPEED*dt)
        self.target_velocity = np.zeros(7)


class FieldOperator:
    def __init__(self, motion, localizer, *, stdin=None, output=print):
        self.motion, self.localizer = motion, localizer
        self.stdin, self.output = sys.stdin if stdin is None else stdin, output
        self.input_buffer = ''

    def read(self, prompt, *, check_live=True):
        self.output(prompt)
        while True:
            if check_live:
                self.motion.fatal_check()
                if self.localizer is not None:
                    self.localizer.check()
            if '\n' in self.input_buffer:
                line, self.input_buffer = self.input_buffer.split('\n', 1)
                return line.strip().lower()
            ready, _, _ = select.select([self.stdin], [], [], .05)
            if ready:
                # 部分的なパイプ入力でもreadlineで止まらず、監視を継続する。
                chunk = os.read(self.stdin.fileno(), 4096)
                if not chunk:
                    line, self.input_buffer = self.input_buffer, ''
                    return line.strip().lower() if line else 'q'
                self.input_buffer += chunk.decode('utf-8', errors='replace')

    def __call__(self, pause, *, allow_resume):
        self.motion.hold(pause)
        monitor = self.motion.monitor
        while True:
            self.motion.fatal_check()
            self.localizer.check()
            with monitor.lock:
                automatic = monitor.pending is not None and monitor.pending.automatic
            if automatic and allow_resume:
                if monitor.stable():
                    monitor.resume()
                    return 'resume'
                time.sleep(.01)
                continue
            choices = 'r:再計測して再開 / b:復帰 / q:終了' if allow_resume else 'b:復帰 / q:終了（この試行の押下は終了）'
            answer = self.read(f'一時停止: {pause}\n{choices}')
            if answer == 'q':
                monitor.event('operator_quit')
                return 'quit'
            if answer not in (('r', 'b') if allow_resume else ('b',)):
                continue
            try:
                monitor.resume(rebase=True)
            except TrialPause as exc:
                self.output(str(exc))
                continue
            monitor.event('operator_action', action=answer)
            return 'resume' if answer == 'r' else 'return'
