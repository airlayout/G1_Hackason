"""Bounded, observation-only trajectory search with complete physical rollouts."""
from dataclasses import dataclass, field, asdict, replace
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from itertools import product
from collections import deque
import time
import math

import mujoco
import numpy as np
from scipy.interpolate import CubicHermiteSpline, PchipInterpolator
from scipy.optimize import minimize
from mujoco import rollout

from robust_scene import SceneSnapshot, build_scene_model, clone_kin, contact_metrics
from trajectory import ARM_JOINTS, ARM_TORQUE_LIMITS_NM
from alignment import press_compensation_allowance, press_overtravel_allowance
from alignment import JointCommandFilter
from robust_control import BottleFeedback, bottle_contact_margin, fade_return_offset, reference_command, waist_support_torque
from motion_limits import (JointMotionLimits, CONTROL_PERIOD_S, MOTION_OVERHEAD_S,
                           PLANNED_LIMIT_FRACTION, MOTION_TOLERANCE_S)

PERIOD = CONTROL_PERIOD_S


@dataclass(frozen=True)
class SearchSettings:
    budget_s: float = 10.
    max_rollouts: int = 1000
    scenarios: int = 3
    workers: int = 8
    max_candidates: int = 50
    seed: int = 0
    position_error_m: float = .001
    normal_error_deg: float = 1.
    tracking_error_rad: float = .001
    mass_range: tuple = (.28, .42)
    friction_range: tuple = (.24, .36)
    gravity_scale: float = 1.
    motion_duration_s: float = 30.
    joint_speed_rad_s: float = 1.2
    joint_acceleration_rad_s2: float = 4.
    position_span_m: float = .003
    bottle_feedback: str = 'off'

    @property
    def motion_limits(self):
        return JointMotionLimits(self.joint_speed_rad_s, self.joint_acceleration_rad_s2)

    def __post_init__(self):
        self.motion_limits
        if not np.isfinite(self.motion_duration_s) or not MOTION_OVERHEAD_S + PERIOD < self.motion_duration_s <= 300:
            raise ValueError('Motion duration must allow time for enable, home confirmation and release')
        counts = (self.max_rollouts,self.scenarios,self.workers,self.max_candidates,self.seed)
        if any(not isinstance(v,int) or isinstance(v,bool) for v in counts) or self.seed < 0:
            raise ValueError('Search counts and seed must be integers')
        if (not np.isfinite(self.budget_s) or not 0 < self.budget_s <= 300
                or not 1 <= self.scenarios <= self.max_rollouts <= 10000
                or not 1 <= self.workers <= 32 or not 1 <= self.max_candidates <= 100
                or not 0 <= self.gravity_scale <= 1
                or not 0 <= self.position_error_m <= .02
                or not np.isfinite(self.position_span_m) or not 0 <= self.position_span_m <= .02
                or self.bottle_feedback not in ('off', 'relative')
                or not 0 <= self.normal_error_deg <= 10
                or not 0 <= self.tracking_error_rad <= .02):
            raise ValueError('Invalid robust search settings')
        for bounds in (self.mass_range, self.friction_range):
            if len(bounds) != 2 or not np.all(np.isfinite(bounds)) or not 0 < bounds[0] <= bounds[1]:
                raise ValueError('Invalid physical uncertainty range')


@dataclass
class Candidate:
    index: int
    q: np.ndarray
    tcp: np.ndarray
    phases: list
    press_start: int
    press_end: int
    retract_end: int
    duration_s: float
    min_joint_margin_rad: float
    min_clearance_m: float
    tau: np.ndarray
    motion_metrics: dict = field(default_factory=dict)
    waist_tau: np.ndarray | None = None


@dataclass
class PlanResult:
    scene: SceneSnapshot
    candidate: Candidate | None = None
    score: float | None = None
    report: dict = field(default_factory=dict)

    def to_dict(self):
        result = dict(self.report, scene=self.scene.to_dict(), accepted=self.candidate is not None,
                      score=self.score)
        if self.candidate is not None:
            c = self.candidate
            result['trajectory'] = dict(candidate=c.index, period_s=PERIOD, q=c.q.tolist(),
                tcp=c.tcp.tolist(), phases=c.phases, press_start=c.press_start,
                press_end=c.press_end, retract_end=c.retract_end, duration_s=c.duration_s,
                motion_metrics=c.motion_metrics)
            result['trajectory']['waist_tau_nm'] = None if c.waist_tau is None else c.waist_tau.tolist()
        return result


def spline_extrema(spline):
    """Per-segment, per-joint extrema, including between control ticks."""
    a, b, c = spline.c[:3]
    h = np.diff(spline.x)[:, None]
    root = np.divide(-b, 3*a, out=np.zeros_like(b), where=a != 0)
    interior = (a != 0) & (root > 0) & (root < h)
    velocity = np.maximum(np.abs(c), np.abs(3*a*h*h+2*b*h+c))
    velocity = np.maximum(velocity, np.where(interior, np.abs(3*a*root*root+2*b*root+c), 0))
    acceleration = np.maximum(np.abs(2*b), np.abs(6*a*h+2*b))
    return velocity, acceleration


def spline_peaks(spline):
    velocity, acceleration = spline_extrema(spline)
    return float(velocity.max()), float(acceleration.max())


def allocate_bottle_phases(points, durations, labels, settings, deadline):
    """Reserve sliding-contact time and minimize free-motion time within joint headroom."""
    lower = np.array([1.1*d if phase == 'press' else d if phase == 'contact' else PERIOD
                      for d, phase in zip(durations, labels[1:])])
    limits = settings.motion_limits

    def constraints(values):
        if time.perf_counter() >= deadline:
            raise TimeoutError('phase allocation budget')
        times = np.r_[0., np.cumsum(values)]
        derivatives = PchipInterpolator(times, points, axis=0).derivative()(times)
        derivatives[0] = derivatives[-1] = 0.
        velocity, acceleration = spline_extrema(CubicHermiteSpline(times, points, derivatives, axis=0))
        return np.r_[1-velocity.ravel()/(PLANNED_LIMIT_FRACTION*limits.joint_speed_rad_s),
                     1-acceleration.ravel()/(PLANNED_LIMIT_FRACTION*limits.joint_acceleration_rad_s2)]

    result = minimize(np.sum, np.maximum(durations, lower), jac=lambda values: np.ones_like(values),
        bounds=[(float(v), None) for v in lower], method='SLSQP',
        constraints={'type': 'ineq', 'fun': constraints}, options={'maxiter': 80, 'ftol': 1e-8})
    if not result.success or np.min(constraints(result.x)) < -1e-6:
        raise ValueError('Unable to allocate stable contact time within joint headroom')
    return np.r_[0., np.cumsum(result.x)]


def retime_reference(times, points, settings):
    """Keep the geometric spline; fit the motion budget with 20% command headroom."""
    derivatives = PchipInterpolator(times, points, axis=0).derivative()(times)
    derivatives[0] = derivatives[-1] = 0.
    spline = CubicHermiteSpline(times, points, derivatives, axis=0)
    speed, acceleration = spline_peaks(spline)
    limits = settings.motion_limits
    minimum_scale = max(speed/(PLANNED_LIMIT_FRACTION*limits.joint_speed_rad_s),
                        math.sqrt(acceleration/(PLANNED_LIMIT_FRACTION*limits.joint_acceleration_rad_s2)))
    # Each sample is held for PERIOD, including the final home command.
    desired_end = max(PERIOD, settings.motion_duration_s-MOTION_OVERHEAD_S-PERIOD)
    end = max(desired_end, times[-1]*minimum_scale)
    ticks = max(1, math.ceil(end/PERIOD-1e-9))
    end = ticks*PERIOD
    duration = (ticks+1)*PERIOD
    if duration+MOTION_OVERHEAD_S > settings.motion_duration_s+MOTION_TOLERANCE_S+1e-9:
        raise ValueError('Motion budget cannot retain 20% joint speed/acceleration headroom')
    scale = end/times[-1]
    times = times*scale
    spline = CubicHermiteSpline(times, points, derivatives/scale, axis=0)
    speed, acceleration = spline_peaks(spline)
    metrics = dict(trajectory_s=duration, expected_motion_s=duration+MOTION_OVERHEAD_S,
                   target_motion_s=settings.motion_duration_s, overhead_s=MOTION_OVERHEAD_S,
                   max_joint_speed_rad_s=speed, max_joint_acceleration_rad_s2=acceleration,
                   speed_limit_fraction=speed/limits.joint_speed_rad_s,
                   acceleration_limit_fraction=acceleration/limits.joint_acceleration_rad_s2,
                   planned_limit_fraction=PLANNED_LIMIT_FRACTION, limits=limits.to_dict())
    return times, spline, np.arange(ticks+1)*PERIOD, metrics


def reference_torque(kin, q, velocity, scale):
    original = kin.data.qpos.copy()
    try:
        pose = original.copy(); pose[kin.qadr] = q; kin.update(pose)
        tau = kin.data.qfrc_bias[kin.dadr].copy()
        # dq_ref compensation matches the SDK's dq=0, kd=1.5 contract.
        tau += 1.5*velocity
        return np.clip(tau*scale, -np.array(ARM_TORQUE_LIMITS_NM), ARM_TORQUE_LIMITS_NM)
    finally:
        kin.update(original)


def generate_candidate(kin, scene, index, seed_offset, waypoint_offset, settings, deadline):
    original = kin.data.qpos.copy()
    face, normal = np.array(scene.face), np.array(scene.normal)
    r = scene.tip_radius
    home = original[kin.qadr].copy()
    points, labels = [home], ['approach']
    tip = kin.tip_position()
    raised = face-(r+.12)*normal + [0, waypoint_offset[0], (.10 if scene.task=='bottle' else 0.)+waypoint_offset[1]]
    depth = (scene.stroke+press_compensation_allowance(scene.stroke)
             if scene.task == 'button' else scene.push_distance)
    relative_feedback = scene.task == 'bottle' and settings.bottle_feedback == 'relative'
    contact_clearance = .005+scene.uncertainty_m+settings.position_error_m if relative_feedback else .005
    target_points = [(raised, 'approach'), (face-(r+.05)*normal, 'approach'),
                     (face-(r+contact_clearance)*normal, 'close'), (face-r*normal, 'contact')]
    for d in np.linspace(0, depth, max(2, math.ceil(depth/.005)+1))[1:]:
        target_points.append((face+(d-r)*normal, 'press'))
    target_points += [(face-(r+.05)*normal, 'retract'), (raised, 'return')]
    for target, phase in target_points:
        if time.perf_counter() >= deadline:
            raise TimeoutError('candidate generation budget')
        seed = kin.data.qpos[kin.qadr].copy()
        if len(points) == 1:
            seed[:3] += seed_offset
        solution = kin.solve(target, normal, seed=seed)
        kin.validate_path(solution.joint_angles, allowed_tip_geoms={kin.model.geom('robust_target_geom').id},
                          allow_tip_contact=True)
        pose = kin.data.qpos.copy(); pose[kin.qadr] = solution.joint_angles; kin.update(pose)
        points.append(solution.joint_angles.copy()); labels.append(phase)
    points.append(home); labels.append('home')
    points = np.array(points)
    # These durations are relative phase weights, not minimum execution times.
    node_tcp = []
    for q in points:
        pose = original.copy(); pose[kin.qadr] = q; kin.update(pose); node_tcp.append(kin.tip_position())
    node_tcp = np.array(node_tcp)
    durations = []
    for i in range(1, len(points)):
        if scene.task == 'bottle':
            # Keep time for stable sliding contact; use the faster arm limits in free space.
            speed = dict(press=.005, contact=.005 if relative_feedback else .02,
                         close=.04, retract=.1).get(labels[i], .2)
        else:
            speed = min(.003, scene.stroke/2) if labels[i] == 'press' else (
                     .005 if labels[i] == 'contact' else .01 if labels[i] == 'close' else
                     .025 if labels[i] == 'retract' else .05)
        durations.append(max(.3, 1.6*np.linalg.norm(node_tcp[i]-node_tcp[i-1])/speed,
                             2*np.max(np.abs(points[i]-points[i-1]))/settings.joint_speed_rad_s,
                             math.sqrt(8*np.max(np.abs(points[i]-points[i-1]))/settings.joint_acceleration_rad_s2)))
    times = (allocate_bottle_phases(points, durations, labels, settings, deadline)
             if scene.task == 'bottle' else np.r_[0., np.cumsum(durations)])
    times, spline, t, metrics = retime_reference(times, points, settings)
    qs = spline(t)
    if scene.task == 'bottle':
        metrics['bottle_feedback_mode'] = settings.bottle_feedback
        metrics['bottle_feedback'] = dict(offset_limit_m=scene.uncertainty_m+.005,
            observation_period_s=.2, observation_delay_s=.2, window_frames=3,
            observation_noise_m=settings.position_error_m, gravity_scale=settings.gravity_scale)
    intervals = np.minimum(np.searchsorted(times, t, side='left'), len(labels)-1)
    phases = [labels[i] for i in intervals]
    limits = kin.model.jnt_range[[kin.model.joint(n).id for n in ARM_JOINTS]]
    margin = float(np.min(np.minimum(qs-limits[:, 0], limits[:, 1]-qs)))
    if margin < .025:
        raise ValueError('Interpolated joint limit')
    tcp, taus, waist_taus, clearance = [], [], [], .05
    for i, q in enumerate(qs):
        if time.perf_counter() >= deadline:
            raise TimeoutError('static validation budget')
        pose = original.copy(); pose[kin.qadr] = q; kin.update(pose)
        clearance = min(clearance, contact_metrics(kin, scene, contact=True))
        tcp.append(kin.tip_position())
        waist_taus.append(waist_support_torque(kin))
        taus.append(np.clip((kin.data.qfrc_bias[kin.dadr]+1.5*spline(t[i],1))*settings.gravity_scale,
                            -np.array(ARM_TORQUE_LIMITS_NM),ARM_TORQUE_LIMITS_NM))
    kin.update(original)
    if clearance < .003:
        raise ValueError('Static obstacle clearance is below 3mm')
    start = next(i for i, p in enumerate(phases) if p == 'contact')
    end = max(i for i, p in enumerate(phases) if p == 'press')
    retract = max(i for i, p in enumerate(phases) if p == 'retract')
    return Candidate(index, qs, np.array(tcp), phases, start, end, retract, len(qs)*PERIOD,
                     margin, clearance, np.array(taus), metrics, np.array(waist_taus))


def uncertainty_scenarios(settings, scene, *, include_corners=True, full_uncertainty=False):
    rng = np.random.default_rng(settings.seed)
    rows = []
    for i in range(settings.scenarios):
        # Consume the same random draws so mass/friction/angle/tracking samples retain their seeds.
        span = settings.position_error_m+scene.uncertainty_m if full_uncertainty else settings.position_span_m
        position = np.zeros(3) if i == 0 else rng.uniform(-1, 1, 3)*span
        if not full_uncertainty and i in (1, 2):
            position = np.asarray(scene.normal)*span*(-1 if i == 1 else 1)
        if full_uncertainty and include_corners and settings.scenarios >= 9 and 1 <= i <= 8:
            position = np.array(list(product((-1., 1.), repeat=3))[i-1])*(settings.position_error_m+scene.uncertainty_m)
        angles = np.zeros(3) if i == 0 else rng.uniform(-1, 1, 3)*np.deg2rad(settings.normal_error_deg)
        tracking = np.zeros(7) if i == 0 else rng.uniform(-1, 1, 7)*settings.tracking_error_rad
        rows.append(dict(index=i, position=position.tolist(), angles=angles.tolist(), tracking=tracking.tolist(),
            mass=float(np.mean(settings.mass_range)) if i == 0 else float(rng.uniform(*settings.mass_range)),
            friction=float(np.mean(settings.friction_range)) if i == 0 else float(rng.uniform(*settings.friction_range))))
    return rows


def _trial_failure_reasons(scene, *, displacement, peak_press, peak_force, clearance,
                           tracking_max, dynamic_joint_margin, home_error):
    if scene.task == 'button':
        checks = dict(press_depth=.9*scene.stroke <= peak_press <= scene.stroke+press_overtravel_allowance(scene.stroke),
                      press_force=peak_force <= 40.)
    else:
        checks = dict(push_distance=abs(displacement[0]-scene.push_distance) <= .01,
                      push_lateral=abs(displacement[1]) <= .01)
    checks.update(obstacle_clearance=clearance >= .003, tracking_error=tracking_max <= .01,
                  joint_margin=dynamic_joint_margin >= .02, home_return=home_error <= np.deg2rad(2))
    return [name for name, passed in checks.items() if not passed]


def physical_trial(base_kin, scene, c, condition, deadline):
    started = time.perf_counter()
    if started >= deadline:
        return dict(completed=False,reason='deadline before model copy',wall_s=0.)
    kin = clone_kin(base_kin); m, d = kin.model, kin.data
    d.qpos[:] = base_kin.data.qpos
    d.qpos[kin.qadr] += condition['tracking']
    from scipy.spatial.transform import Rotation
    shift = np.array(condition['position'])
    if scene.task == 'bottle':
        adr = int(m.joint('robust_object_joint').qposadr[0])
        d.qpos[adr:adr+3] += shift
        # The initial support boundary shares the scene's placement uncertainty.
        # It is then fixed for the entire trajectory, even when the bottle moves.
        m.geom_pos[m.geom('robust_wall').id] += shift
        body = m.body('robust_target').id
        ratio = condition['mass']/m.body_mass[body]
        m.body_mass[body] *= ratio; m.body_inertia[body] *= ratio
        m.geom_friction[m.geom('robust_wall').id, 0] = condition['friction']
        m.geom_friction[m.geom('robust_target_geom').id, 0] = condition['friction']
    else:
        body = m.body('robust_panel').id
        m.body_pos[body] += shift
        rot = Rotation.from_rotvec(condition['angles']) * Rotation.from_quat(m.body_quat[body][[1,2,3,0]])
        m.body_quat[body] = rot.as_quat()[[3,0,1,2]]
    mujoco.mj_setConst(m, d)
    # mj_setConst is model preprocessing; restore the observed start afterwards.
    d.qpos[:] = base_kin.data.qpos; d.qpos[kin.qadr] += condition['tracking']
    if scene.task == 'bottle':
        d.qpos[adr:adr+3] += shift
    d.ctrl[:] = scene.q
    mujoco.mj_forward(m, d)
    initial_object = d.xpos[m.body('robust_target').id].copy()
    clearance, lateral_max, peak_press, tracking_max = .05, 0., 0., 0.
    target_margin = float('inf')
    peak_force = 0.
    limits = m.jnt_range[[m.joint(name).id for name in ARM_JOINTS]]
    dynamic_joint_margin = float('inf')
    n = np.array(scene.normal)
    ctrl_spec = mujoco.mjtState.mjSTATE_CTRL | mujoco.mjtState.mjSTATE_QFRC_APPLIED
    state_spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
    state = np.empty(mujoco.mj_stateSize(m, state_spec))
    target_id = m.body('robust_target').id
    gains = m.actuator_gainprm[kin.actuator, 0]
    waist_dadr = np.array([int(m.joint(index).dofadr[0]) for index in (12,13,14)])
    waist_taus = c.waist_tau
    if waist_taus is None:
        waist_taus = np.array([waist_support_torque(base_kin,q) for q in c.q])
    feedback = (BottleFeedback(scene) if scene.task == 'bottle'
                and c.motion_metrics.get('bottle_feedback_mode', 'off') == 'relative' else None)
    if feedback is not None:
        command_filter = JointCommandFilter(c.q[0], JointMotionLimits(**c.motion_metrics['limits']))
        reference_kin = clone_kin(base_kin)
        observation_history = deque([np.zeros(3) for _ in range(4)], maxlen=4)
        feedback_settings = c.motion_metrics['bottle_feedback']
        sensor_rng = np.random.default_rng(condition['index']+731)
    max_feedback_offset = 0.
    # Blocks are bounded to 0.2 simulated seconds, so a deadline never waits for a whole trajectory.
    for first in range(0, len(c.q), 10):
        if time.perf_counter() >= deadline:
            return dict(completed=False, reason='deadline', wall_s=time.perf_counter()-started)
        last = min(first+10, len(c.q))
        inputs = np.zeros((last-first, m.nu+m.nv))
        inputs[:, :m.nu] = scene.q
        inputs[:, kin.actuator] = c.q[first:last]+np.array(condition['tracking'])
        inputs[:, m.nu+kin.dadr] = c.tau[first:last]
        inputs[:, m.nu+waist_dadr] = waist_taus[first:last]
        reference_tcp = c.tcp[first:last].copy()
        if feedback is not None:
            # Fixed calibration translation cancels in object displacement.
            # Use a three-frame median and a full camera-period delivery delay.
            noise = sensor_rng.uniform(-1., 1., 3)*feedback_settings['observation_noise_m']
            observation_history.append(d.xpos[target_id]-initial_object+noise)
            observed_delta = np.median(list(observation_history)[:-1], axis=0)
            try:
                for j, i in enumerate(range(first, last)):
                    progress = max(0., float((c.tcp[min(i,c.press_end)]-scene.face)@n+scene.tip_radius))
                    offset = feedback.update(observed_delta, progress, first//10,
                        freeze=i < c.press_start or i > c.press_end)
                    offset = fade_return_offset(offset, c, i)
                    max_feedback_offset = max(max_feedback_offset, float(np.linalg.norm(offset)))
                    goal, torque, tcp = reference_command(reference_kin,c,i,offset,command_filter,
                                                          feedback_settings['gravity_scale'])
                    inputs[j, kin.actuator] = goal+np.array(condition['tracking'])
                    inputs[j, m.nu+kin.dadr] = torque
                    inputs[j, m.nu+waist_dadr] = waist_support_torque(reference_kin,goal)
                    reference_tcp[j] = tcp
            except (ValueError, RuntimeError) as exc:
                return dict(completed=False, reason=str(exc), wall_s=time.perf_counter()-started)
        controls = np.repeat(inputs, 10, axis=0)
        mujoco.mj_getState(m, d, state, state_spec)
        states, _ = rollout.rollout(m, d, state, controls, control_spec=ctrl_spec)
        if np.any(np.diff(states[0, :, 0]) <= 0):
            return dict(completed=False, reason='diverged', wall_s=time.perf_counter()-started)
        for j, st in enumerate(states[0, 9::10]):
            i = first+j
            mujoco.mj_setState(m, d, st, state_spec)
            d.ctrl[:] = inputs[j,:m.nu]
            d.qfrc_applied[:] = inputs[j,m.nu:]
            if scene.task == 'bottle':
                mujoco.mj_fwdPosition(m,d)  # exact collision/FK; no force solve is needed for these metrics
            else:
                mujoco.mj_forward(m,d)
            try:
                clearance = min(clearance, contact_metrics(kin, scene, contact=True))
            except ValueError as exc:
                return dict(completed=False, reason=str(exc), wall_s=time.perf_counter()-started)
            tracking_max = max(tracking_max, float(np.linalg.norm(kin.tip_position()-reference_tcp[j])))
            dynamic_joint_margin = min(dynamic_joint_margin,float(np.min(np.minimum(
                d.qpos[kin.qadr]-limits[:,0],limits[:,1]-d.qpos[kin.qadr]))))
            if c.press_start <= i <= c.press_end:
                if scene.task == 'button':
                    actual_normal = d.xmat[m.body('robust_panel').id].reshape(3,3)[:,0]
                    actual_contact = d.xpos[target_id]
                else:
                    actual_normal = n
                    actual_contact = d.xpos[target_id]+[0,0,scene.face[2]-scene.base_height-scene.object_height/2]
                delta = kin.tip_position()-actual_contact
                lateral_max = max(lateral_max,float(np.linalg.norm(delta-actual_normal*(delta@actual_normal))))
                if scene.task == 'bottle':
                    bottom = d.xpos[target_id][2]-scene.object_height/2
                    target_margin = min(target_margin, bottle_contact_margin(
                        scene,kin.tip_position(),d.xpos[target_id],bottom))
            if scene.task == 'button':
                peak_press = max(peak_press, float(d.qpos[m.joint('robust_object_joint').qposadr[0]]))
                for contact_index, collision in enumerate(d.contact):
                    if {int(collision.geom1),int(collision.geom2)} == {kin.tip_id,m.geom('robust_target_geom').id}:
                        force = np.zeros(6); mujoco.mj_contactForce(m,d,contact_index,force)
                        peak_force = max(peak_force,float(np.linalg.norm(force[:3])))
            elif c.press_start <= i <= c.press_end:
                if d.xmat[target_id].reshape(3,3)[2,2] < math.cos(math.radians(10)):
                    return dict(completed=False, reason='bottle tilted', wall_s=time.perf_counter()-started)
        # The last state, not a kinematic reset, starts the next block.
    if time.perf_counter() >= deadline:
        return dict(completed=False, reason='deadline', wall_s=time.perf_counter()-started)
    displacement = d.xpos[target_id]-initial_object
    if scene.task == 'button':
        target_margin = scene.radius-scene.tip_radius-lateral_max
    home_error = float(np.max(np.abs(d.qpos[kin.qadr]-scene.q[22:29])))
    failures = _trial_failure_reasons(scene, displacement=displacement, peak_press=peak_press,
        peak_force=peak_force, clearance=clearance, tracking_max=tracking_max,
        dynamic_joint_margin=dynamic_joint_margin, home_error=home_error)
    return dict(completed=True, success=not failures, failure_reasons=failures,
        min_clearance_m=clearance, target_margin_m=target_margin, home_error_rad=home_error,
        tracking_max_m=tracking_max, min_joint_margin_rad=dynamic_joint_margin,
        peak_press_m=peak_press, peak_force_n=peak_force if scene.task=='button' else None,
        displacement_m=displacement.tolist(), max_feedback_offset_m=max_feedback_offset,
        wall_s=time.perf_counter()-started)


_worker_kin = None


def _init_worker(path, calibration, scene):
    global _worker_kin
    try:
        from threadpoolctl import threadpool_limits
        threadpool_limits(limits=1)
    except ImportError:
        pass
    _worker_kin = build_scene_model(path, calibration, scene)


def _worker_trial(scene, candidate, condition, deadline):
    return physical_trial(_worker_kin, scene, candidate, condition, deadline)


def search(path, calibration, scene, settings=SearchSettings(), *, monitor=lambda: None):
    started = time.perf_counter()
    reserve = min(.35,settings.budget_s*.05)
    deadline = started+settings.budget_s-reserve
    result = PlanResult(scene)
    report = result.report
    report.update(schema='g1-robust-plan-v1', settings=asdict(settings), static_checks=0,
        completed_rollouts=0, aborted_rollouts=0, candidates=[], stages_wall_s={}, stopped_for_measurement=0)
    workers = min(settings.workers, settings.scenarios)
    report['selection'] = 'first_valid'
    from upper_body_control import control_contract
    report['upper_body_control'] = control_contract()
    report['base_model'] = 'fixed_pelvis; locomotion balance is not simulated'
    report['validation_envelope'] = dict(position_span_m=settings.position_span_m,
        calibration_uncertainty_m=scene.uncertainty_m, target_margin_is_diagnostic=True)
    report['runtime'] = dict(mujoco_version=mujoco.__version__, physics_timestep_s=.002,
                             control_period_s=PERIOD, workers=workers,shutdown_reserve_s=reserve)
    t = time.perf_counter(); kin = build_scene_model(path, calibration, scene)
    report['stages_wall_s']['scene'] = time.perf_counter()-t
    conditions = uncertainty_scenarios(settings, scene); report['conditions'] = conditions
    offsets = [(0.,0.,0.)]+[v for v in product((-.4,0.,.4),(-.4,0.,.4),(-.6,0.,.6)) if v != (0.,0.,0.)]
    static_time = physics_time = 0.
    # spawn is safe even when the camera/SDK already own threads. Do not fork them.
    with ProcessPoolExecutor(max_workers=workers,
            mp_context=multiprocessing.get_context('spawn'), initializer=_init_worker,
            initargs=(str(path), calibration, scene)) as pool:
        for index in range(settings.max_candidates):
            monitor()
            if time.perf_counter() >= deadline or report['completed_rollouts']+report['aborted_rollouts']+len(conditions) > settings.max_rollouts:
                break
            original = kin.data.qpos.copy(); t = time.perf_counter()
            row = {'index': index}; report['candidates'].append(row)
            report['static_checks'] += 1
            try:
                waypoint = [(0.,0.),(-.04,.04),(.04,.04),(0.,.08)][index % 4]
                candidate = generate_candidate(kin, scene, index, offsets[index % len(offsets)], waypoint, settings, deadline)
            except (ValueError, TimeoutError) as exc:
                row['reason'] = str(exc)
                continue
            finally:
                static_time += time.perf_counter()-t
                kin.update(original)
            t = time.perf_counter()
            futures = [pool.submit(_worker_trial, scene, candidate, cond, deadline) for cond in conditions]
            trials = []
            for future in futures:
                while not future.done():
                    monitor()
                    time.sleep(.005)
                trials.append(future.result())
            physics_time += time.perf_counter()-t
            monitor()
            report['completed_rollouts'] += sum(t['completed'] for t in trials)
            report['aborted_rollouts'] += sum(not t['completed'] for t in trials)
            row.update(trials=trials, accepted=time.perf_counter() < deadline and all(
                t.get('completed',False) and t.get('success',False) for t in trials))
            if not row['accepted']:
                row['reason'] = ('deadline' if time.perf_counter() >= deadline else
                                 'Not every condition completed and passed')
                continue
            score = min(candidate.min_joint_margin_rad/.025,
                        min(t['min_joint_margin_rad'] for t in trials)/.025,
                        min(t['min_clearance_m'] for t in trials)/.003)
            row.update(score=score, duration_s=candidate.duration_s)
            result.candidate, result.score = candidate, score
            break
    report['stages_wall_s'].update(candidate_and_static=static_time, physics_and_scoring=physics_time)
    report['total_wall_s'] = time.perf_counter()-started
    report['target_1000_met'] = report['completed_rollouts'] >= 1000 and report['total_wall_s'] <= 10.
    if result.candidate is None:
        report['reason'] = 'No candidate completed and passed every required uncertainty condition'
    return result


def evaluate_holdout(path, calibration, plan, settings, *, seed=10000, count=20, budget_s=60.):
    """Independent post-selection scenarios. Never count these inside the search budget."""
    if plan.candidate is None:
        return dict(completed=0,successes=0,reason='no selected plan')
    started = time.perf_counter()
    if not 1 <= count <= 100:
        raise ValueError('holdout count must be 1..100')
    evaluation = replace(settings,seed=seed,scenarios=count+1,max_rollouts=max(count+1,settings.max_rollouts))
    conditions = uncertainty_scenarios(evaluation,plan.scene,include_corners=False,full_uncertainty=True)[1:]
    with ProcessPoolExecutor(max_workers=min(settings.workers,count),
            mp_context=multiprocessing.get_context('spawn'),initializer=_init_worker,
            initargs=(str(path),calibration,plan.scene)) as pool:
        trials = list(pool.map(_worker_trial,[plan.scene]*count,[plan.candidate]*count,
                              conditions,[started+budget_s]*count))
    return dict(seed=seed,conditions=conditions,trials=trials,completed=sum(t['completed'] for t in trials),
                successes=sum(t.get('success',False) for t in trials),wall_s=time.perf_counter()-started)
