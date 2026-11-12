"""Measured-joint button control shared by simulation and the SDK executor.

The integral term is a bounded torque, never an extra press displacement.
Only the reference clock pauses; the arm remains under feedback control.
"""
import time

import mujoco
import numpy as np

from alignment import JointCommandFilter
from robust_scene import update_robot, contact_metrics
from trajectory import ARM_TORQUE_LIMITS_NM, ARM_JOINTS

PERIOD = .02
COMMAND_CLEARANCE_M = .003
KI = np.array([40., 40., 40., 40., 20., 20., 20.])
BIAS_LIMIT = np.array([6., 6., 4., 6., 2., 1.5, 1.5])
BIAS_SLEW = np.array([2., 2., 2., 2., 1., 1., 1.])
TORQUE_LIMIT = np.asarray(ARM_TORQUE_LIMITS_NM)


def controller_metadata():
    return dict(version=1, mode='joint', period_s=PERIOD,
        ki_nm_rad_s=KI.tolist(), bias_limit_nm=BIAS_LIMIT.tolist(),
        bias_slew_nm_s=BIAS_SLEW.tolist(), initial_hold_s=2.,
        press_settle_s=.12, home_tolerance_m=.002,
        extra_press_displacement_m=0., command_clearance_m=COMMAND_CLEARANCE_M)


class TorqueIntegral:
    def __init__(self):
        self.bias = np.zeros(7)
        self.baseline = None
        self.retreat_base = self.retreat_contact = None
        self.stamp = None

    def update(self, error, stamp, feedforward, *, depth, stroke, retreat=False):
        # A repeated cached DDS sample must not accumulate another integral step.
        dt = 0. if self.stamp is None else np.clip(stamp-self.stamp, 0., .04)
        if self.stamp is None or stamp > self.stamp:
            self.stamp = stamp
        increment = np.clip(KI*np.asarray(error)*dt, -BIAS_SLEW*dt, BIAS_SLEW*dt)
        if depth >= -.005 and self.baseline is None:
            self.baseline = self.bias.copy()
        previous = self.bias.copy()
        if retreat:
            if self.retreat_base is None:
                self.retreat_base = (self.bias if self.baseline is None else self.baseline).copy()
                self.retreat_contact = self.bias-self.retreat_base
            amount = np.clip((depth+.005)/(stroke+.005), 0., 1.)
            self.retreat_base = np.clip(self.retreat_base+increment,-BIAS_LIMIT,BIAS_LIMIT)
            desired = self.retreat_base+amount*self.retreat_contact
            self.bias += np.clip(desired-self.bias, -BIAS_SLEW*dt, BIAS_SLEW*dt)
        else:
            self.bias += increment
        self.bias = np.clip(self.bias, -BIAS_LIMIT, BIAS_LIMIT)
        raw = feedforward+self.bias
        saturated = (raw-np.clip(raw, -TORQUE_LIMIT, TORQUE_LIMIT))*(self.bias-previous) > 0
        self.bias[saturated] = previous[saturated]
        if retreat:
            self.retreat_base[saturated] -= increment[saturated]
        return np.clip(feedforward+self.bias, -TORQUE_LIMIT, TORQUE_LIMIT)


class ButtonTrackingController:
    def __init__(self, kin, scene, candidate, settings, *, diagnostic=False):
        if scene.task != 'button':
            raise ValueError('Joint feedback requires a button scene')
        if candidate.motion_metrics.get('button_feedback') != controller_metadata():
            raise ValueError('Button feedback controller differs from the verified plan')
        self.kin, self.scene, self.candidate, self.settings = kin, scene, candidate, settings
        self.qs, self.tcps = candidate.q.copy(), candidate.tcp.copy()
        self.labels = list(candidate.phases)
        self.press_end = candidate.press_end
        self.diagnostic = diagnostic
        if diagnostic:
            depth = (self.tcps-np.asarray(scene.face))@np.asarray(scene.normal)+scene.tip_radius
            near = np.flatnonzero(depth[:candidate.press_start+1] >= -.005-1e-9)
            cut = max(0, int(near[0])-1) if len(near) else candidate.press_start-1
            self.qs = np.r_[self.qs[:cut+1], self.qs[:cut][::-1]]
            self.tcps = np.r_[self.tcps[:cut+1], self.tcps[:cut][::-1]]
            self.labels = ['approach']*(cut+1)+['return']*cut
            self.press_end = cut
        self.filter = JointCommandFilter(self.qs[0], settings.motion_limits)
        self.integral = TorqueIntegral()
        self.index = 0
        self.credit = 0.
        self.settled = 0
        self.retreat = False
        self.done = False
        self.error = None
        self.recovery_started = None
        self.last_progress = 0.
        self.guard_holds = 0
        self.normal = np.asarray(scene.normal)
        self.limits = kin.model.jnt_range[[kin.model.joint(n).id for n in ARM_JOINTS]]

    def start_from_command(self, q):
        """Keep command continuity with the SDK hold target after handover."""
        self.filter = JointCommandFilter(q,self.settings.motion_limits)

    def recover(self, elapsed, reason):
        """Return over the reference already traversed; never jump to the press end."""
        if self.recovery_started is not None:
            return
        self.error = reason
        self.recovery_started = elapsed
        self.qs = self.qs[:self.index+1][::-1].copy()
        self.tcps = self.tcps[:self.index+1][::-1].copy()
        self.labels = ['recovery']*len(self.qs)
        self.index = 0
        self.credit = 0.
        self.settled = 0
        self.retreat = True

    def _body_corrected_goal(self, pose, nominal, tcp):
        # Recompute the arm solution in the measured waist/IMU pose. The target
        # remains in the calibrated world frame; no fixed height offset is used.
        q = nominal.copy()
        for _ in range(4):
            ref = pose.copy(); ref[self.kin.qadr] = q; self.kin.update(ref)
            error = tcp-self.kin.tip_position()
            if np.linalg.norm(error) < .0001:
                break
            jac = np.zeros((3, self.kin.model.nv))
            mujoco.mj_jacSite(self.kin.model, self.kin.data, jac, None, self.kin.site_id)
            arm_jac = jac[:, self.kin.dadr]
            dq = arm_jac.T@np.linalg.solve(arm_jac@arm_jac.T+.0001*np.eye(3), error)
            q = np.clip(q+np.clip(dq, -.04, .04),
                np.maximum(self.limits[:, 0]+.025, nominal-.12),
                np.minimum(self.limits[:, 1]-.025, nominal+.12))
        return q

    def step(self, q, imu, stamp, elapsed, offset=None):
        from robust_planner import reference_torque
        offset = np.zeros(3) if offset is None else np.asarray(offset)
        if self.recovery_started is None and (elapsed > self.settings.button_feedback_timeout_s
                or elapsed-self.last_progress > 20.):
            self.recover(elapsed, 'Button tracking did not converge within its time budget')
        if self.recovery_started is not None and elapsed-self.recovery_started > 30.:
            raise RuntimeError('Button recovery exceeded 30s')
        pose = update_robot(self.kin, q, imu)
        measured_tip = self.kin.tip_position().copy()
        # Measured penetrations remain a hard failure even in collection mode.
        contact_metrics(self.kin, self.scene, contact=not self.diagnostic)
        target = self.tcps[self.index]+offset
        depth = float((target-self.scene.face)@self.normal+self.scene.tip_radius)
        actual_depth = float((measured_tip-self.scene.face)@self.normal+self.scene.tip_radius)
        error = float(np.linalg.norm(measured_tip-target))
        lateral = float(np.linalg.norm((measured_tip-target)-((measured_tip-target)@self.normal)*self.normal))
        previous_index = self.index
        if elapsed >= 2. or self.retreat:
            low, high = ((.003,.012) if depth < -.05 else (.001,.005)
                         if depth < -.005 or self.retreat else (.001,.003))
            rate = float(np.clip((high-error)/(high-low), 0., 1.))
            if not self.retreat and depth >= -.005 and (lateral > .001 or depth-actual_depth > .001):
                rate = 0.
            if not self.retreat and self.index == self.press_end:
                reached = self.diagnostic or (actual_depth >= self.scene.stroke-.0005 and lateral <= .001)
                self.settled = self.settled+1 if reached else 0
                if self.settled >= 6:
                    self.retreat = True
                    self.settled = 0
                else:
                    rate = 0.
            self.credit += rate
            advance = int(self.credit); self.credit -= advance
            stop = len(self.qs)-1 if self.retreat else self.press_end
            self.index = min(stop, self.index+advance)
            if self.retreat and self.index == len(self.qs)-1:
                self.settled = self.settled+1 if error <= .002 else 0
                self.done = self.settled >= 10
        target = self.tcps[self.index]+offset
        depth = float((target-self.scene.face)@self.normal+self.scene.tip_radius)
        desired = self._body_corrected_goal(pose, self.qs[self.index], target)
        old_position, old_velocity = self.filter.position.copy(), self.filter.velocity.copy()
        goal = self.filter.next_goal(desired, PERIOD)
        target_geom = self.kin.model.geom('robust_target_geom').id

        def validate(command):
            self.kin.update(pose)
            self.kin.validate_path(command, allow_tip_contact=not self.diagnostic,
                allowed_tip_geoms={target_geom})
            commanded = pose.copy(); commanded[self.kin.qadr] = command; self.kin.update(commanded)
            if contact_metrics(self.kin, self.scene, contact=not self.diagnostic) < COMMAND_CLEARANCE_M:
                raise ValueError(f'Command has less than {COMMAND_CLEARANCE_M * 1000:g}mm obstacle clearance')
        try:
            validate(goal)
        except ValueError:
            self.guard_holds += 1
            self.index = previous_index
            self.credit = 0.
            # Decelerate within the same speed/acceleration contract. Freezing
            # velocity instantly would violate the SDK command filter contract.
            self.filter.position, self.filter.velocity = old_position, old_velocity
            goal = self.filter.next_goal(old_position, PERIOD)
            validate(goal)
            target = self.tcps[self.index]+offset
            depth = float((target-self.scene.face)@self.normal+self.scene.tip_radius)
        if self.index != previous_index:
            self.last_progress = elapsed
        self.kin.update(pose)
        ff = reference_torque(self.kin, goal, self.filter.velocity, self.settings.gravity_scale)
        tau = self.integral.update(goal-np.asarray(q)[22:29], stamp, ff,
            depth=depth, stroke=self.scene.stroke, retreat=self.retreat)
        return dict(q=goal, velocity=self.filter.velocity.copy(), tau=tau,
            torque_bias=self.integral.bias.copy(), measured_tip=measured_tip, target=target,
            error_m=error, actual_depth_m=actual_depth, reference_depth_m=depth,
            phase='initial_correction' if elapsed < 2. and not self.retreat else self.labels[self.index],
            index=self.index, reference_held=self.index==previous_index,
            guard_holds=self.guard_holds, done=self.done)
