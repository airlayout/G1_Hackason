"""Shared bottle feedback and command generation for planning and hardware."""
import mujoco
import numpy as np

from motion_limits import CONTROL_PERIOD_S
from trajectory import ARM_TORQUE_LIMITS_NM
from upper_body_control import WAIST_INDICES, WAIST_TORQUE_LIMIT_NM


def waist_support_torque(kin, right_q=None):
    """Bounded gravity feedforward at the held waist and nominal arm target."""
    original = kin.data.qpos.copy()
    try:
        if right_q is not None:
            pose = original.copy(); pose[kin.qadr] = right_q; kin.update(pose)
        addresses = [int(kin.model.joint(index).dofadr[0]) for index in WAIST_INDICES]
        return np.clip(kin.data.qfrc_bias[addresses],-WAIST_TORQUE_LIMIT_NM,WAIST_TORQUE_LIMIT_NM).copy()
    finally:
        if right_q is not None:
            kin.update(original)


class BottleFeedback:
    """Follow relative object motion; a fixed camera translation cancels in the delta."""
    def __init__(self, scene):
        self.scene = scene
        self.normal = np.asarray(scene.normal)
        self.side = np.array([-self.normal[1], self.normal[0], 0.])
        self.side /= np.linalg.norm(self.side)
        self.limit_m = scene.uncertainty_m+.005
        self.offset = np.zeros(3)
        self.last_id = -1

    def update(self, displacement, progress, sample_id, *, freeze=False):
        displacement = np.asarray(displacement, dtype=float)
        if displacement.shape != (3,) or not np.all(np.isfinite(displacement)):
            raise RuntimeError('Invalid bottle displacement')
        forward, sideways = float(displacement@self.normal), float(displacement@self.side)
        if abs(progress-forward) > .010:
            raise RuntimeError('Bottle progress differs from the reference by more than 10mm')
        if abs(sideways) > .010 or abs(displacement[2]) > .010:
            raise RuntimeError('Bottle moved more than 10mm off the push line')
        if sample_id == self.last_id:
            return self.offset.copy()
        self.last_id = sample_id
        if freeze:
            return self.offset.copy()
        # The object initially moves away from an off-centre contact. Following
        # that motion with gain >1 recentres contact while bounding its deflection.
        along = np.clip(2*(progress-forward), -self.limit_m, self.limit_m)
        across = np.clip(4*sideways, -self.limit_m, self.limit_m)
        target = along*self.normal+across*self.side
        delta = target-self.offset
        self.offset += delta*min(1., .0005/max(np.linalg.norm(delta), 1e-12))
        return self.offset.copy()


def fade_return_offset(offset, candidate, index):
    """Retain compensation until clear of the bottle, then return to exact home."""
    if index <= candidate.retract_end:
        return offset
    remaining = len(candidate.q)-1-index
    return offset*max(0., remaining/max(1, len(candidate.q)-1-candidate.retract_end))


def reference_command(kin, candidate, index, offset, command_filter, gravity_scale):
    """Identical Jacobian correction, limits, torque and FK in simulation and real execution."""
    original = kin.data.qpos.copy()
    try:
        pose = original.copy()
        pose[kin.qadr] = candidate.q[index]
        kin.update(pose)
        jacobian = np.zeros((3, kin.model.nv))
        mujoco.mj_jacSite(kin.model, kin.data, jacobian, None, kin.site_id)
        dq = np.linalg.lstsq(jacobian[:, kin.dadr], offset, rcond=.01)[0]
        goal = command_filter.next_goal(candidate.q[index]+dq, CONTROL_PERIOD_S)
        pose[kin.qadr] = goal
        kin.update(pose)
        tcp = kin.tip_position().copy()
        torque = np.clip((kin.data.qfrc_bias[kin.dadr]+1.5*command_filter.velocity)*gravity_scale,
                         -np.array(ARM_TORQUE_LIMITS_NM), ARM_TORQUE_LIMITS_NM)
        return goal, torque, tcp
    finally:
        kin.update(original)


def bottle_contact_margin(scene, tip, centre, bottom):
    """Distance inside the actual cylinder; the trial already applies position uncertainty."""
    side = np.array([-scene.normal[1], scene.normal[0], 0.])
    side /= np.linalg.norm(side)
    horizontal = scene.radius-scene.tip_radius-abs(float((tip-centre)@side))
    vertical = min(tip[2]-scene.tip_radius-bottom,
                   bottom+scene.object_height-tip[2]-scene.tip_radius)
    return float(min(horizontal, vertical))
