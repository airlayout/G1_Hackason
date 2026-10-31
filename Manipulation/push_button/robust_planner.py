"""Bounded, observation-only trajectory search with complete physical rollouts."""
from dataclasses import dataclass, field, asdict, replace
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from itertools import product
import time
import math

import mujoco
import numpy as np
from scipy.interpolate import CubicHermiteSpline, PchipInterpolator
from mujoco import rollout

from robust_scene import SceneSnapshot, build_scene_model, clone_kin, contact_metrics
from trajectory import ARM_JOINTS, ARM_TORQUE_LIMITS_NM
from alignment import press_compensation_allowance, press_overtravel_allowance

PERIOD = .02


@dataclass(frozen=True)
class SearchSettings:
    budget_s: float = 10.
    max_rollouts: int = 1000
    scenarios: int = 20
    workers: int = 8
    max_candidates: int = 50
    seed: int = 0
    position_error_m: float = .001
    normal_error_deg: float = 1.
    tracking_error_rad: float = .001
    mass_range: tuple = (.28, .42)
    friction_range: tuple = (.24, .36)
    gravity_scale: float = 1.

    def __post_init__(self):
        counts = (self.max_rollouts,self.scenarios,self.workers,self.max_candidates,self.seed)
        if any(not isinstance(v,int) or isinstance(v,bool) for v in counts) or self.seed < 0:
            raise ValueError('Search counts and seed must be integers')
        if (not np.isfinite(self.budget_s) or not 0 < self.budget_s <= 300
                or not 1 <= self.scenarios <= self.max_rollouts <= 10000
                or not 1 <= self.workers <= 32 or not 1 <= self.max_candidates <= 100
                or not 0 <= self.gravity_scale <= 1
                or not 0 <= self.position_error_m <= .02
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
                press_end=c.press_end, retract_end=c.retract_end, duration_s=c.duration_s)
        return result


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
    target_points = [(raised, 'approach'), (face-(r+.05)*normal, 'approach'),
                     (face-(r+.005)*normal, 'close'), (face-r*normal, 'contact')]
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
                          allow_tip_contact=phase in ('contact', 'press', 'retract'))
        pose = kin.data.qpos.copy(); pose[kin.qadr] = solution.joint_angles; kin.update(pose)
        points.append(solution.joint_angles.copy()); labels.append(phase)
    points.append(home); labels.append('home')
    points = np.array(points)
    # One continuous interpolation; no holds or zero-velocity stops at approach waypoints.
    node_tcp = []
    for q in points:
        pose = original.copy(); pose[kin.qadr] = q; kin.update(pose); node_tcp.append(kin.tip_position())
    node_tcp = np.array(node_tcp)
    durations = []
    for i in range(1, len(points)):
        speed = (.004 if scene.task == 'bottle' else min(.003, scene.stroke/2)) if labels[i] == 'press' else (
                 .005 if labels[i] == 'contact' else .01 if labels[i] == 'close' else
                 .025 if labels[i] == 'retract' else .05)
        durations.append(max(.3, 1.6*np.linalg.norm(node_tcp[i]-node_tcp[i-1])/speed,
                             2*np.max(np.abs(points[i]-points[i-1]))/.8,
                             math.sqrt(8*np.max(np.abs(points[i]-points[i-1]))/2)))
    times = np.r_[0., np.cumsum(durations)]
    derivatives = PchipInterpolator(times, points, axis=0).derivative()(times)
    derivatives[0] = derivatives[-1] = 0.
    spline = CubicHermiteSpline(times, points, derivatives, axis=0)
    t = np.arange(0, times[-1]+PERIOD, PERIOD); t[-1] = times[-1]
    velocity, acceleration = spline(t, 1), spline(t, 2)
    scale = max(1., np.max(np.abs(velocity))/.8, np.sqrt(np.max(np.abs(acceleration))/2))
    times *= scale
    derivatives /= scale
    spline = CubicHermiteSpline(times, points, derivatives, axis=0)
    t = np.linspace(0, times[-1], math.ceil(times[-1]/PERIOD)+1)
    qs = spline(t)
    intervals = np.minimum(np.searchsorted(times, t, side='left'), len(labels)-1)
    phases = [labels[i] for i in intervals]
    limits = kin.model.jnt_range[[kin.model.joint(n).id for n in ARM_JOINTS]]
    margin = float(np.min(np.minimum(qs-limits[:, 0], limits[:, 1]-qs)))
    if margin < .025:
        raise ValueError('Interpolated joint limit')
    tcp, taus, clearance = [], [], .05
    for i, q in enumerate(qs):
        if time.perf_counter() >= deadline:
            raise TimeoutError('static validation budget')
        pose = original.copy(); pose[kin.qadr] = q; kin.update(pose)
        clearance = min(clearance, contact_metrics(kin, scene, contact=phases[i] in ('contact','press','retract')))
        tcp.append(kin.tip_position())
        taus.append(np.clip((kin.data.qfrc_bias[kin.dadr]+1.5*spline(t[i],1))*settings.gravity_scale,
                            -np.array(ARM_TORQUE_LIMITS_NM),ARM_TORQUE_LIMITS_NM))
    kin.update(original)
    if clearance < .003:
        raise ValueError('Static obstacle clearance is below 3mm')
    start = next(i for i, p in enumerate(phases) if p == 'contact')
    end = max(i for i, p in enumerate(phases) if p == 'press')
    retract = max(i for i, p in enumerate(phases) if p == 'retract')
    return Candidate(index, qs, np.array(tcp), phases, start, end, retract, len(qs)*PERIOD,
                     margin, clearance, np.array(taus))


def uncertainty_scenarios(settings, scene):
    rng = np.random.default_rng(settings.seed)
    rows = []
    for i in range(settings.scenarios):
        position = np.zeros(3) if i == 0 else rng.uniform(-1, 1, 3)*(settings.position_error_m+scene.uncertainty_m)
        angles = np.zeros(3) if i == 0 else rng.uniform(-1, 1, 3)*np.deg2rad(settings.normal_error_deg)
        tracking = np.zeros(7) if i == 0 else rng.uniform(-1, 1, 7)*settings.tracking_error_rad
        rows.append(dict(index=i, position=position.tolist(), angles=angles.tolist(), tracking=tracking.tolist(),
            mass=float(np.mean(settings.mass_range)) if i == 0 else float(rng.uniform(*settings.mass_range)),
            friction=float(np.mean(settings.friction_range)) if i == 0 else float(rng.uniform(*settings.friction_range))))
    return rows


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
    peak_force = 0.
    limits = m.jnt_range[[m.joint(name).id for name in ARM_JOINTS]]
    dynamic_joint_margin = float('inf')
    n = np.array(scene.normal)
    ctrl_spec = mujoco.mjtState.mjSTATE_CTRL | mujoco.mjtState.mjSTATE_QFRC_APPLIED
    state_spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
    state = np.empty(mujoco.mj_stateSize(m, state_spec))
    target_id = m.body('robust_target').id
    gains = m.actuator_gainprm[kin.actuator, 0]
    # Blocks are bounded to 0.2 simulated seconds, so a deadline never waits for a whole trajectory.
    for first in range(0, len(c.q), 10):
        if time.perf_counter() >= deadline:
            return dict(completed=False, reason='deadline', wall_s=time.perf_counter()-started)
        last = min(first+10, len(c.q))
        inputs = np.zeros((last-first, m.nu+m.nv))
        inputs[:, :m.nu] = scene.q
        inputs[:, kin.actuator] = c.q[first:last]+np.array(condition['tracking'])
        inputs[:, m.nu+kin.dadr] = c.tau[first:last]
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
                clearance = min(clearance, contact_metrics(kin, scene,
                    contact=c.phases[i] in ('contact','press','retract')))
            except ValueError as exc:
                return dict(completed=False, reason=str(exc), wall_s=time.perf_counter()-started)
            tracking_max = max(tracking_max, float(np.linalg.norm(kin.tip_position()-c.tcp[i])))
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
    success = (.9*scene.stroke <= peak_press <= scene.stroke+press_overtravel_allowance(scene.stroke)
               and peak_force <= 40. if scene.task == 'button' else
               abs(displacement[0]-scene.push_distance) <= .01 and abs(displacement[1]) <= .01)
    target_margin = scene.radius-scene.tip_radius-scene.uncertainty_m-lateral_max
    success = bool(success and clearance >= .003 and target_margin > .003
                   and tracking_max <= .01 and dynamic_joint_margin >= .02
                   and np.max(np.abs(d.qpos[kin.qadr]-scene.q[22:29])) <= np.deg2rad(2))
    return dict(completed=True, success=success, min_clearance_m=clearance, target_margin_m=target_margin,
        tracking_max_m=tracking_max, min_joint_margin_rad=dynamic_joint_margin,
        peak_press_m=peak_press, peak_force_n=peak_force if scene.task=='button' else None,
        displacement_m=displacement.tolist(),
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
    report['runtime'] = dict(mujoco_version=mujoco.__version__, physics_timestep_s=.002,
                             control_period_s=PERIOD, workers=settings.workers,shutdown_reserve_s=reserve)
    t = time.perf_counter(); kin = build_scene_model(path, calibration, scene)
    report['stages_wall_s']['scene'] = time.perf_counter()-t
    conditions = uncertainty_scenarios(settings, scene); report['conditions'] = conditions
    offsets = [(0.,0.,0.)]+[v for v in product((-.4,0.,.4),(-.4,0.,.4),(-.6,0.,.6)) if v != (0.,0.,0.)]
    best_score, best_duration = -np.inf, np.inf
    static_time = physics_time = 0.
    # spawn is safe even when the camera/SDK already own threads. Do not fork them.
    with ProcessPoolExecutor(max_workers=settings.workers,
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
            row.update(trials=trials, accepted=all(t.get('success',False) for t in trials))
            if not row['accepted']:
                continue
            score = min(candidate.min_joint_margin_rad/.025,
                        min(t['min_joint_margin_rad'] for t in trials)/.025,
                        min(t['min_clearance_m'] for t in trials)/.003,
                        min(t['target_margin_m'] for t in trials)/.003)
            row.update(score=score, duration_s=candidate.duration_s)
            if score > best_score+1e-6 or (abs(score-best_score) <= 1e-6 and candidate.duration_s < best_duration):
                result.candidate, result.score = candidate, score
                best_score, best_duration = score, candidate.duration_s
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
    conditions = uncertainty_scenarios(evaluation,plan.scene)[1:]  # exclude the reused nominal condition
    with ProcessPoolExecutor(max_workers=settings.workers,
            mp_context=multiprocessing.get_context('spawn'),initializer=_init_worker,
            initargs=(str(path),calibration,plan.scene)) as pool:
        trials = list(pool.map(_worker_trial,[plan.scene]*count,[plan.candidate]*count,
                              conditions,[started+budget_s]*count))
    return dict(seed=seed,conditions=conditions,trials=trials,completed=sum(t['completed'] for t in trials),
                successes=sum(t.get('success',False) for t in trials),wall_s=time.perf_counter()-started)
