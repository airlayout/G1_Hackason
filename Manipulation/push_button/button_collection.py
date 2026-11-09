"""Collision-checked IK retries and observation policy for explicit collection mode."""
import time

import numpy as np
from scipy.optimize import least_squares

from kinematics import IkSolution, IK_AXIS_TOLERANCE_DEG
from trajectory import ARM_JOINTS
from robust_perception import CorrectionGate


class CollectionCorrectionGate(CorrectionGate):
    """Record target movement and keep visual correction within the checked range."""

    def __init__(self, scene):
        super().__init__(scene)
        self.normal_deviations = []
        self.position_changes = []

    def update(self, observed, progress, *, freeze=False):
        if self.scene.task != 'button':
            return super().update(observed, progress, freeze=freeze)
        if observed.frame_id == self.last_id:
            return self.offset.copy()
        face = np.asarray(observed.geometry['face'],dtype=float)
        if face.shape != (3,) or not np.all(np.isfinite(face)):
            raise RuntimeError('Invalid observed target position')
        normal = np.asarray(observed.geometry['normal'], dtype=float)
        if normal.shape != (3,) or not np.all(np.isfinite(normal)) or np.linalg.norm(normal) < 1e-9:
            raise RuntimeError('Invalid observed target normal')
        angle = float(np.degrees(np.arccos(np.clip(
            (normal/np.linalg.norm(normal)) @ np.asarray(self.scene.normal), -1., 1.))))
        if angle > 3. and observed.frame_id != self.last_id:
            self.normal_deviations.append(dict(frame_id=observed.frame_id,
                captured_at_s=observed.captured_at, angle_deg=angle,
                observed_normal=normal.tolist(), planned_normal=list(self.scene.normal)))
        planned_normal = np.asarray(self.scene.normal)
        displacement = face-np.asarray(self.scene.face)
        forward = float(displacement@planned_normal)
        lateral = displacement-forward*planned_normal
        length = float(np.linalg.norm(lateral))
        desired = np.zeros(3) if length <= .001 else lateral*min(1.,.005/length)
        if not freeze:
            delta = desired-self.offset
            self.offset += delta*min(1.,.0005/max(float(np.linalg.norm(delta)),1e-12))
        self.last_id = observed.frame_id
        self.position_changes.append(dict(frame_id=observed.frame_id,captured_at_s=observed.captured_at,
            observed_face_m=face.tolist(),displacement_m=displacement.tolist(),
            press_direction_change_m=forward,lateral_change_m=length,
            correction_offset_m=self.offset.tolist(),correction_clamped=length > .005,
            target_locked=freeze))
        return self.offset.copy()


def solve_collection_waypoint(kin, target, normal, *, seed, rng, deadline):
    """Try other arm postures without accepting an IK residual or a colliding path."""
    original = kin.data.qpos.copy()
    target, normal = np.asarray(target), np.asarray(normal)
    rest = original[kin.qadr].copy()
    allowed = {kin.model.geom('robust_target_geom').id}
    limits = kin.model.jnt_range[[kin.model.joint(n).id for n in ARM_JOINTS]]

    def validate(solution):
        if time.perf_counter() >= deadline:
            raise TimeoutError('collision-checked IK search budget')
        kin.update(original)
        kin.validate_path(solution.joint_angles, allow_tip_contact=True, allowed_tip_geoms=allowed)
        return solution

    def residual(q):
        if time.perf_counter() >= deadline:
            raise TimeoutError('collision-checked IK search budget')
        pose = original.copy()
        pose[kin.qadr] = q
        kin.update(pose)
        axis = kin.data.site_xmat[kin.site_id].reshape(3, 3)[:, 0]
        return np.r_[kin.tip_position()-target, .1*(axis-normal), .0001*(q-rest)]

    try:
        if time.perf_counter() >= deadline:
            raise TimeoutError('collision-checked IK search budget')
        try:
            return validate(kin.solve(target, normal, seed=seed))
        except ValueError:
            pass
        for attempt in range(30):
            initial = rest if attempt == 0 else np.clip(
                rest+rng.normal(0, 1.2, 7), limits[:, 0]+.026, limits[:, 1]-.026)
            result = least_squares(residual, initial,
                bounds=(limits[:, 0]+.025, limits[:, 1]-.025), max_nfev=150)
            error = residual(result.x)
            position_error = float(np.linalg.norm(error[:3]))
            angle = float(np.degrees(np.arccos(np.clip(
                1-np.linalg.norm(error[3:6]/.1)**2/2, -1, 1))))
            if position_error >= .0001 or angle >= IK_AXIS_TOLERANCE_DEG:
                continue
            margin = float(np.minimum(result.x-limits[:, 0], limits[:, 1]-result.x).min())
            try:
                return validate(IkSolution(result.x, position_error, angle, margin, result.nfev))
            except ValueError:
                continue
        raise ValueError('No collision-free IK path after alternate arm postures')
    finally:
        kin.update(original)
