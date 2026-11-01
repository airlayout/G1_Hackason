"""Observed scenes for planning. No simulator ground truth or measured table outline."""
from dataclasses import dataclass, asdict
from pathlib import Path
from xml.etree import ElementTree as ET
import copy

import mujoco
import numpy as np

from bottle import _fit_circle
from kinematics import ArmKinematics, load_robot
from vision import _points


@dataclass(frozen=True)
class SceneSnapshot:
    task: str
    face: tuple
    normal: tuple
    radius: float
    q: tuple
    imu: tuple
    captured_at: float
    tip_radius: float = .015
    uncertainty_m: float = .002
    base_height: float | None = None
    object_height: float = .22
    stroke: float = .0015
    push_distance: float = .05
    frame_ids: tuple = ()

    def __post_init__(self):
        numbers = (*self.face, *self.normal, *self.q, *self.imu, self.radius,
                   self.tip_radius, self.uncertainty_m, self.captured_at,
                   self.object_height, self.stroke, self.push_distance)
        if (self.task not in ('button', 'bottle') or len(self.q) != 29
                or len(self.face) != 3 or len(self.normal) != 3 or len(self.imu) != 3
                or not np.all(np.isfinite(numbers)) or not .02 <= self.radius <= .06
                or not 0 < self.tip_radius < self.radius or not 0 <= self.uncertainty_m <= .02
                or abs(np.linalg.norm(self.normal)-1) > .001
                or self.normal[0] < .9 or not .0015 <= self.stroke <= .015
                or not 0 < self.push_distance <= .1):
            raise ValueError('Invalid observed scene')
        if self.task == 'bottle' and (self.base_height is None or
                not np.isfinite(self.base_height) or not .4 <= self.base_height <= 1.2):
            raise ValueError('Bottle bottom is not observable; specify --bottle-base-height')
        if self.frame_ids and (len(self.frame_ids) != 3 or len(set(self.frame_ids)) != 3):
            raise ValueError('Snapshot requires three distinct frame IDs')

    @property
    def wall_x(self):
        return self.face[0]  # fixed initial front tangent, never the moving centre

    def to_dict(self):
        return asdict(self)


def estimate_bottle(depth, bbox, intrinsics, transform, *, base_height=None, contact_height=.08):
    """Fit cylindrical foreground; estimate its visible lower silhouette, not a table plane.

    Require valid depth down to the detector's bottom edge. A cropped/occluded bottom
    is unobservable and must not silently become the bottom of the fitted cylinder.
    """
    depth = np.asarray(depth)
    transform = np.asarray(transform)
    x0, y0, x1, y1 = map(int, bbox)
    if (depth.ndim != 2 or transform.shape != (4, 4) or not np.all(np.isfinite(transform))
            or min(x0, y0) < 0 or x1 > depth.shape[1] or y1 > depth.shape[0]
            or (base_height is None and y1 >= depth.shape[0]-2)
            or x1-x0 < 12 or y1-y0 < 24):
        raise ValueError('Bottle bounds or bottom visibility invalid')
    yy, xx = np.mgrid[y0:y1, x0:x1]
    z = depth[yy, xx]
    valid = np.isfinite(z) & (z > .15) & (z < 2.)
    if valid.mean() < .6:
        raise ValueError('Insufficient bottle depth')
    xyz = _points(np.c_[xx[valid], yy[valid]], z[valid], intrinsics)
    xyz = xyz @ transform[:3, :3].T + transform[:3, 3]
    pixels_y = yy[valid]
    body = (pixels_y > y0+(y1-y0)*.3) & (pixels_y < y0+(y1-y0)*.85)
    # An oblique view makes the cylinder diagonal within its axis-aligned bbox.
    # Avoid mixed silhouette pixels and, when measured, the support surface and
    # tapered shoulder. Use the same physical body band as the existing bottle
    # localizer; the measured height does not replace curved-depth evidence.
    pixels_x = xx[valid]
    body &= (pixels_x > x0+(x1-x0)*.08) & (pixels_x < x1-(x1-x0)*.08)
    if base_height is not None:
        if not np.isfinite(base_height) or not .4 <= base_height <= 1.2:
            raise ValueError('Invalid measured bottle bottom height')
        body &= ((xyz[:, 2] > base_height+.025)
                 & (xyz[:, 2] < base_height+contact_height+.055))
    if body.sum() < 30:
        raise ValueError('Insufficient bottle body depth above the support surface')
    centre, radius, _ = _fit_circle(xyz[body])
    cylinder = np.abs(np.linalg.norm(xyz[:, :2]-centre, axis=1)-radius) < .004
    lower = cylinder & (pixels_y >= y1-max(4, (y1-y0)*.06))
    if base_height is None:
        if lower.sum() < 12 or np.ptp(xyz[lower, 1]) < radius*.5:
            raise ValueError('Bottle bottom depth missing; specify --bottle-base-height')
        # An oblique camera's bottom image band spans several physical heights.
        # Use the cylinder's lowest world-Z envelope, rather than requiring the
        # entire image band to be horizontal.
        envelope = cylinder & (xyz[:,2] <= np.percentile(xyz[cylinder,2],5))
        if envelope.sum() < 12 or np.ptp(xyz[envelope,1]) < radius*.5:
            raise ValueError('Bottle lower silhouette is insufficient')
        base_height = float(np.percentile(xyz[envelope,2],10))
        if np.percentile(xyz[envelope, 2], 90)-base_height > .015:
            raise ValueError('Bottle bottom height is uncertain')
    # Include the narrower shoulder/cap inside the fitted cylinder; its outer
    # surface alone ends at the body and underestimates a PET bottle's height.
    inside = np.linalg.norm(xyz[:, :2]-centre, axis=1) <= radius+.004
    top = float(np.percentile(xyz[inside, 2], 98))
    height = top-base_height
    if not .10 <= height <= .40:
        raise ValueError('Bottle height or bottom is inconsistent')
    return dict(face=(float(centre[0]-radius), float(centre[1]), float(base_height+contact_height)),
                normal=(1., 0., 0.), radius=float(radius), base_height=float(base_height),
                object_height=height)


def add_scene(root, scene):
    world = root.find('worldbody')
    face, normal = np.array(scene.face), np.array(scene.normal)
    if scene.task == 'bottle':
        # Finite representation of B, covering the complete arm workspace.
        ET.SubElement(world, 'geom', name='robust_wall', type='box',
            pos=f'{scene.wall_x+1.5} 0 {scene.base_height-1.5}', size='1.5 3 1.5',
            friction='.3 .005 .0001', rgba='.85 .2 .2 .25')
        centre = face + [scene.radius, 0, scene.base_height+scene.object_height/2-face[2]]
        body = ET.SubElement(world, 'body', name='robust_target', pos=' '.join(map(str, centre)))
        ET.SubElement(body, 'freejoint', name='robust_object_joint')
        ET.SubElement(body, 'geom', name='robust_target_geom', type='cylinder',
            size=f'{scene.radius} {scene.object_height/2}', mass='.35',
            friction='.3 .005 .0001', condim='4', rgba='.2 .7 .8 1')
    else:
        # Local X is the measured panel normal. Four boxes leave only the target aperture.
        axis = np.cross([1., 0, 0], normal)
        quat = np.r_[1+normal[0], axis]; quat /= np.linalg.norm(quat)
        panel = ET.SubElement(world, 'body', name='robust_panel', pos=' '.join(map(str, face)),
                              quat=' '.join(map(str, quat)))
        r = scene.radius+.001
        for i, (y, z, sy, sz) in enumerate(((r+1, 0, 1, 2), (-r-1, 0, 1, 2),
                                            (0, r+1, r, 1), (0, -r-1, r, 1))):
            ET.SubElement(panel, 'geom', name=f'robust_panel_{i}', type='box',
                          pos=f'.025 {y} {z}', size=f'.025 {sy} {sz}', rgba='.3 .3 .3 1')
        target = ET.SubElement(panel, 'body', name='robust_target', pos='.004 0 0')
        ET.SubElement(target, 'joint', name='robust_object_joint', type='slide', axis='1 0 0',
            range=f'0 {scene.stroke}', limited='true', stiffness='350', damping='3',
            solreflimit='.004 1', solimplimit='.99 .99 .0001')
        ET.SubElement(target, 'geom', name='robust_target_geom', type='cylinder',
            quat='.70710678 0 .70710678 0', size=f'{scene.radius} .004', mass='.035', rgba='.8 .8 .8 1')


def build_scene_model(path, calibration, scene):
    model, _ = load_robot(Path(path).resolve(), tuple(calibration['tip_offset_m']),
        calibration['pelvis_height_m'], scene.tip_radius, lambda root: add_scene(root, scene))
    model.opt.timestep = .002
    kin = ArmKinematics(model)
    q = model.qpos0.copy(); q[:29] = scene.q
    # update_fixed_base requires the robot-only nq; apply the observed pelvis rotation here.
    pelvis = model.body('pelvis').id
    from scipy.spatial.transform import Rotation
    # The existing camera calibration defines forward yaw=0 in the stopped floor
    # frame. IMU yaw is absolute; keep it for change detection, not model heading.
    xyzw = Rotation.from_euler('xyz', (scene.imu[0],scene.imu[1],0.)).as_quat()
    model.body_quat[pelvis] = xyzw[[3, 0, 1, 2]]
    kin.update(q)
    # rt/arm_sdk owns waist + both arms. Match all 17 gains, including the
    # holding joints; leaving the model's kp=500 hides waist sag on hardware.
    from upper_body_control import UPPER_BODY_JOINTS, UPPER_BODY_KP, UPPER_BODY_KD
    actuators = np.array([model.actuator(name).id for name in UPPER_BODY_JOINTS])
    gains = np.array(UPPER_BODY_KP)
    model.actuator_gainprm[actuators, 0] = gains
    model.actuator_biasprm[actuators, 1] = -gains
    model.actuator_biasprm[actuators, 2] = -np.array(UPPER_BODY_KD)
    return kin


def update_robot(kin, q, imu):
    from scipy.spatial.transform import Rotation
    xyzw = Rotation.from_euler('xyz',(imu[0],imu[1],0.)).as_quat()
    kin.model.body_quat[kin.model.body('pelvis').id] = xyzw[[3,0,1,2]]
    pose = kin.data.qpos.copy(); pose[:29] = q
    kin.update(pose)
    return pose


def contact_metrics(kin, scene, *, contact=False):
    """Actual collisions; contact=True exempts only the tip/selected target pair."""
    target = kin.model.geom('robust_target_geom').id
    clearance = .05
    for c in kin.data.contact:
        pair = {int(c.geom1), int(c.geom2)}
        if not pair & kin.arm_geoms:
            continue
        if pair == {kin.tip_id, target} and contact:
            continue
        clearance = min(clearance, float(c.dist))
        if c.dist < -.0005:
            raise ValueError(f'Unexpected arm contact: {sorted(pair)}')
    # Distance to static obstacles and self pairs even before contact is generated.
    if not hasattr(kin,'_robust_obstacles'):
        kin._robust_obstacles = tuple(g for g in range(kin.model.ngeom)
            if kin.model.geom(g).name.startswith('robust_') and g != target
            and (kin.model.geom_contype[g] or kin.model.geom_conaffinity[g]))
    obstacles = kin._robust_obstacles
    for a in kin.arm_geoms:
        for b in obstacles:
            distance = mujoco.mj_geomDistance(kin.model, kin.data, a, b, .05, None)
            clearance = min(clearance, float(distance))
    return clearance


def clone_kin(kin):
    return ArmKinematics(copy.copy(kin.model))
