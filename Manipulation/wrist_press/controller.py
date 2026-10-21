"""One pressing state machine for Yada and real DDS camera observations."""
from __future__ import annotations

from dataclasses import asdict
from itertools import product
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
from time import perf_counter
import numpy as np

from .collision import CollisionChecker
from .model import ARM, RobotModel
from .perception import LatestPerception, Localizer, StableTarget
from .planning import Planner, PlanningError, Trajectory
from .types import Command, Report
from .wrist_servo import WristTargetFilter, bounded_correction, tangent_error


class CommandLimiter:
    """Bound command velocity/acceleration; the final segment is collision checked."""
    def __init__(self, q, cfg, lower=None, upper=None):
        self.q, self.v = q.copy(), np.zeros(29)
        self.cfg = cfg
        self.lower,self.upper=lower,upper

    def next(self, goal, dt):
        delta = goal-self.q
        vmax, amax = self.cfg["joint_speed_rad_s"], self.cfg["joint_acceleration_rad_s2"]
        # Brake before the target. Transit is already jerk-limited by Trajectory.
        desired = np.sign(delta)*np.minimum(np.sqrt(2*amax*np.abs(delta)), vmax)
        desired = np.clip(delta/dt, -np.abs(desired), np.abs(desired))
        if self.lower is not None:
            # Include a control-period braking allowance before a hard joint bound.
            upper_distance=np.maximum(0,self.upper-self.q[ARM]-.0005)
            lower_distance=np.maximum(0,self.q[ARM]-self.lower-.0005)
            forward=np.maximum(0,np.sqrt((amax*dt)**2+2*amax*upper_distance)-amax*dt)
            backward=np.maximum(0,np.sqrt((amax*dt)**2+2*amax*lower_distance)-amax*dt)
            desired[ARM]=np.clip(desired[ARM],-backward,forward)
        velocity = self.v + np.clip(desired-self.v, -amax*dt, amax*dt)
        out = self.q + velocity*dt
        out[:22] = goal[:22]  # Waist/other arm are held at their measured initial pose.
        velocity[:22] = 0
        return out, velocity

    def commit(self, q, v):
        self.q, self.v = q.copy(), v.copy()


class PressController:
    def __init__(self, cfg, robot=None, collision=None, localizer=None):
        self.cfg = cfg
        self.robot = robot or RobotModel(cfg)
        self.collision = collision or CollisionChecker(self.robot, cfg)
        self.planner = Planner(self.robot, self.collision, cfg)
        self.localizer = localizer or Localizer(cfg)
        self.vision = LatestPerception(self.localizer, cfg["async_perception"])
        self.stable = StableTarget()
        self.report = Report(cfg["mode"], cfg["detector"])
        self.phase = "look"
        self.target = None
        self.reference_target=None
        self.home = self.last = self.last_t = self.limiter = None
        self.axis = self.trajectory = None
        self.started = self.phase_t = 0.0
        self.name = "up"
        self.kp = np.full(29, 60.0)
        self.kp[[19,20,21,26,27,28]] = 20
        self.baseline_brightness = None
        self.freeze_target = None
        self.contact_q = None
        self.press_goal = None
        self.hold_started = None
        self.press_force=0.0
        self.verified_started = None
        self.alignment_bias = np.zeros(3)
        self.last_accepted_image_t = -float("inf")
        self.plan_pool = ThreadPoolExecutor(max_workers=1) if cfg["async_perception"] else None
        self.plan_robot = RobotModel(cfg) if self.plan_pool else self.robot
        self.plan_collision = CollisionChecker(self.plan_robot,cfg) if self.plan_pool else self.collision
        self.plan_planner = Planner(self.plan_robot,self.plan_collision,cfg) if self.plan_pool else self.planner
        self.plan_future = None
        self.wrist_servo_enabled = bool(cfg.get("wrist_servo", {}).get("enabled", False)
                                       and cfg["use_wrist"] and cfg["mode"] == "button")
        servo = cfg.get("wrist_servo", {})
        self.wrist_filter = WristTargetFilter(servo.get("window", 3), servo.get("spread_m", .003),
                                             cfg["max_image_age_s"])
        self.wrist_completed_key = None
        self.wrist_lost_since = None
        self.wrist_settle_after = None
        self.wrist_confirmations = 0
        self.previous_wrist_step = None
        self.servo_diagnostic = {}

    def reset(self, name, kp=None):
        self.name = name if self.cfg["mode"] == "button" else "bottle"
        if self.name not in ("up", "down", "bottle"):
            raise ValueError("invalid requested target")
        if kp is not None:
            kp = np.asarray(kp)
            self.kp[12:] = kp
        self.report.phases.append({"phase": "look", "start_s": 0.0})

    def _phase(self, name, t):
        self.report.phases[-1]["duration_s"] = t-self.phase_t
        self.phase, self.phase_t = name, t
        self.report.phases.append({"phase": name, "start_s": t-self.started})

    def _start_path(self, path, t):
        self.trajectory = Trajectory(path, self.cfg)
        self.path_started = t

    def _fresh(self, frame, now):
        return frame is not None and -0.02 <= now-frame.captured_at <= self.cfg["max_image_age_s"]

    def _observe(self, obs):
        frame = obs.wrist if self.cfg["use_wrist"] and self.target is not None and self._fresh(obs.wrist, obs.t) else obs.frame
        if not self._fresh(frame, obs.t):
            return None
        target = self.vision.update(frame, self.name, self.target)
        def consistent(candidate):
            return candidate is None or self.reference_target is None or (
                np.linalg.norm(candidate.point-self.reference_target.point)<.015 and
                candidate.normal @ self.reference_target.normal>np.cos(np.radians(8)))
        if not consistent(target):
            target=None
        if (target is None and frame.camera == "wrist" and not self.cfg["async_perception"]
                and self._fresh(obs.frame,obs.t)):
            target = self.vision.update(obs.frame,self.name,self.target)
        if not consistent(target):
            return None
        if target is None or not -0.02 <= obs.t-target.captured_at <= self.cfg["max_image_age_s"]:
            return None
        if target.captured_at <= self.last_accepted_image_t + 1e-6:
            return None
        if target.name != self.name:
            return None
        if target.camera not in self.report.cameras:
            self.report.cameras.append(target.camera)
        self.last_accepted_image_t = target.captured_at
        return target

    def _reset_wrist_track(self):
        reset = getattr(self.localizer, "reset_track", None)
        if reset:
            reset("wrist")

    def _wrist_event(self, obs, event, **values):
        target = self.wrist_filter.latest
        tip = self.robot.tip(obs.q)
        self.servo_diagnostic = {
            "t": float(obs.t), "event": event, "status": self.wrist_filter.reason,
            "phase": self.phase, "alignment_camera": "wrist", "surface_camera": "head",
            "image_t": None if not self.wrist_filter.items else float(self.wrist_filter.items[-1].captured_at),
            "window_image_t": [float(item.captured_at) for item in self.wrist_filter.items],
            "tip_computed_m": tip.tolist(),
            "arm_speed_rad_s": float(np.max(np.abs(obs.dq[ARM]))),
            "target_m": None if target is None else target.point.tolist(),
            "error_vector_m": None if target is None else tangent_error(target, tip).tolist(),
            "correction_vector_m": [0., 0., 0.],
            "confirmations": self.wrist_confirmations, **values}
        self.report.alignment_diagnostics.append(deepcopy(self.servo_diagnostic))

    def _observe_wrist(self, obs):
        """Poll wrist only. A failed exposure is distinct from inference in flight."""
        old_reason = self.wrist_filter.reason
        frame = obs.wrist if self._fresh(obs.wrist, obs.t) and obs.wrist.camera == "wrist" else None
        target = self.vision.update(frame, self.name, self.target)
        key = self.vision.completed_key
        completed = key is not None and key != self.wrist_completed_key and key[0] == "wrist"
        stable = None
        if completed:
            self.wrist_completed_key = key
            if target is None:
                self.wrist_filter.reject(self.vision.error or "wrist target missing")
                self._reset_wrist_track()
            else:
                stable = self.wrist_filter.add(target, obs.t, self.reference_target)
                if stable is not None:
                    if "wrist" not in self.report.cameras:
                        self.report.cameras.append("wrist")
                elif self.wrist_filter.reason != "collecting wrist images":
                    self._reset_wrist_track()
        if frame is None:
            self.wrist_filter.reject("wrist image missing or stale")
        self.wrist_filter.fresh(obs.t)
        if self.wrist_filter.latest is None:
            self.wrist_settle_after = None
            self.wrist_confirmations = 0
        if completed or self.wrist_filter.reason != old_reason:
            self._wrist_event(obs, "observation", accepted=stable is not None,
                              raw_point_m=None if target is None else target.point.tolist(),
                              completed_image_t=None if not completed else float(key[1]))
        return stable if self.wrist_filter.latest is not None else None

    def _align_wrist(self, obs):
        if (obs.t-self.phase_t > self.cfg["alignment_timeout_s"] or
                self.report.corrections >= self.cfg["max_corrections"]):
            raise RuntimeError("wrist alignment did not converge within its budget: " + self.wrist_filter.reason)
        settled = np.max(np.abs(obs.dq[ARM])) < .2
        # Motion between image updates also invalidates the stopped-image count.
        # In async mode the returned exposure may predate the actual stop.
        if not settled:
            self.wrist_settle_after = None
            self.wrist_confirmations = 0
        target = self._observe_wrist(obs)
        if self.wrist_filter.latest is None:
            if self.wrist_lost_since is None:
                self.wrist_lost_since = obs.t
            if obs.t-self.wrist_lost_since >= self.cfg["wrist_servo"]["lost_timeout_s"]:
                raise RuntimeError("wrist reacquisition timeout: " + self.wrist_filter.reason)
            return self.alignment_goal
        self.wrist_lost_since = None
        if target is None:
            return self.alignment_goal  # Never correct/confirm twice from one exposure.
        self.target = target
        tip = self.robot.tip(obs.q)
        error_vector = tangent_error(target, tip)
        error = float(np.linalg.norm(error_vector))
        self.report.phases[-1]["last_error_m"] = error
        if error <= self.cfg["alignment_tolerance_m"]:
            if not settled:
                self.wrist_confirmations = 0
            elif self.wrist_settle_after is None:
                self.wrist_settle_after = obs.t
                self.wrist_confirmations = 0
            elif target.captured_at > self.wrist_settle_after:
                self.wrist_confirmations += 1
            else:
                self.wrist_confirmations = 0
            gap = float((target.point-tip) @ target.normal-self.cfg["tip_radius_m"])
            confirmed = settled and self.wrist_confirmations >= self.cfg["wrist_servo"]["confirm_frames"] and gap > .008
            self._wrist_event(obs, "confirmation", status="confirmed" if confirmed else "settling",
                              accepted=True, error_m=error)
            if confirmed:
                self.press_goal = self.robot.tip(self.alignment_goal).copy()
                self.freeze_target = deepcopy(target)
                self._phase("press", obs.t)
            return self.alignment_goal
        self.wrist_settle_after = None
        self.wrist_confirmations = 0
        servo = self.cfg["wrist_servo"]
        step = bounded_correction(error_vector, servo["gain"], servo["max_step_m"])
        # No integral bias, no advance into the button plane. The next image
        # updates this bounded position goal while CommandLimiter keeps it smooth.
        goal = self.robot.solve(tip+step, obs.q, self.axis)
        if not self.collision.edge(obs.q, goal, target):
            raise PlanningError("unsafe wrist visual correction: " + self.collision.last_reason)
        if self.previous_wrist_step is not None and step @ self.previous_wrist_step < 0:
            self.report.correction_reversals += 1
        self.previous_wrist_step = step.copy()
        self.alignment_goal = goal
        self.report.corrections += 1
        self._wrist_event(obs, "correction", status="correcting", accepted=True, error_m=error,
                          correction_vector_m=step.tolist())
        return goal

    def _plan_approach(self, obs):
        started = perf_counter()
        robot,collision,planner = self.plan_robot,self.plan_collision,self.plan_planner
        p = self.target.point - (self.cfg["tip_radius_m"]+self.cfg["approach_m"])*self.target.normal
        reasons = []
        seed = obs.q.copy()
        seed[ARM] = [-1.0,-0.5,0.3,0.3,0,0,0]
        for tilt,roll,arm_seed in product(self.cfg["finger_tilts_deg"],(0,-.8,.8,-1.6,1.6,3.14),self.cfg["ik_seeds"]):
            if perf_counter()-started > self.cfg["planning_timeout_s"]*3:
                break
            axis = robot.orientation(self.target.normal, tilt,roll)
            try:
                candidate_seed = seed.copy()
                candidate_seed[ARM] = arm_seed
                press_end = self.target.point + (self.cfg[self.target.kind]["max_command_penetration_m"]-.0008-
                                                 self.cfg["tip_radius_m"])*self.target.normal
                end_q = robot.solve(press_end,candidate_seed,axis)
                if not collision.valid(end_q,self.target,contact=True):
                    raise PlanningError(collision.last_reason)
                goal = robot.solve(p,end_q,axis)
                # Reserve room for PD/gravity compensation, not just nominal IK.
                compensated=robot.gravity_command(goal,self.kp) if self.cfg["gravity_mode"]=="position" else goal
                if not collision.valid(compensated,self.target):
                    raise PlanningError("compensated approach: "+collision.last_reason)
                # Select a finger orientation that clears the WHOLE hand through the press.
                planner.cartesian(goal,press_end,axis,self.target,contact=True)
                path = planner.transit(obs.q, goal, self.target)
                # Reserve the outward normal path before committing the approach.
                end = self.target.point-(self.cfg["tip_radius_m"]+self.cfg["retreat_m"])*self.target.normal
                planner.cartesian(goal, end, axis, self.target)
                return path,axis,1000*(perf_counter()-started)
            except (ValueError, PlanningError) as exc:
                reasons.append(str(exc))
        raise PlanningError("approach and retreat could not both be planned: " + "; ".join(dict.fromkeys(reasons)))

    def _apply_approach(self,result,t):
        path,self.axis,elapsed=result
        self.planner.paths=list(self.plan_planner.paths)
        self._start_path(path,t)
        self.report.planning_ms.append(elapsed)
        self._phase("transit",t)

    def _tracking_ok(self, obs, goal, tolerance=0.07):
        return np.max(np.abs(obs.q[ARM]-goal[ARM])) < tolerance and np.max(np.abs(obs.dq[ARM])) < 0.15

    def _path_goal(self, obs):
        return self.trajectory.at(obs.t-self.path_started)

    def _start_retreat(self, obs):
        p = self.target.point-(self.cfg["tip_radius_m"]+self.cfg["retreat_m"])*self.target.normal
        # Contact permitted ONLY until the fingertip has cleared the target.
        path = self.planner.cartesian(obs.q, p, self.axis, self.target, contact=True)
        self._start_path(path, obs.t)
        self._phase("retract", obs.t)

    def _desired(self, obs, dt):
        t = obs.t
        if self.phase == "look":
            target = self._observe(obs)
            if target is not None:
                stable = self.stable.add(target)
                if stable is not None:
                    self.target = stable
                    self.reference_target=deepcopy(stable)
                    self.baseline_brightness = stable.brightness
                    if self.plan_pool:
                        self.plan_future=self.plan_pool.submit(self._plan_approach,obs)
                        self._phase("planning",t)
                    else:
                        self._apply_approach(self._plan_approach(obs),t)
            if t-self.phase_t > self.cfg["observation_timeout_s"]:
                raise RuntimeError("target localization timeout: " + self.vision.error)
            return self.home
        if self.phase == "planning":
            if self.plan_future.done():
                self._apply_approach(self.plan_future.result(),t)
            return self.home
        if self.phase == "planning_return":
            if self.plan_future.done():
                self._start_path(self.plan_future.result(),t)
                self._phase("return",t)
            return self.return_hold
        if self.phase in ("transit", "retract", "return"):
            if self.phase=="transit":
                # Keep the image track alive while approaching; do not queue old views.
                if self.wrist_servo_enabled:
                    self._observe_wrist(obs)
                else:
                    observation=self._observe(obs)
                    if observation is not None and np.linalg.norm(observation.point-self.target.point)<.01:
                        self.target=observation
            goal = self._path_goal(obs)
            if t-self.path_started >= self.trajectory.duration and self._tracking_ok(obs, goal):
                if self.phase == "transit":
                    self._phase("align", t)
                    self.alignment_goal = goal.copy()
                    self.last_correction_t = t
                    if self.wrist_servo_enabled:
                        self.alignment_bias.fill(0)
                        self.wrist_settle_after = None
                        self.wrist_confirmations = 0
                        self.wrist_lost_since = t if self.wrist_filter.latest is None else None
                        self.report.alignment_camera = "wrist"
                        self.report.surface_camera = self.reference_target.camera
                elif self.phase == "retract":
                    # Replan from MEASURED pose; reversing an old path is not assumed safe.
                    if self.plan_pool:
                        self.return_hold=obs.q.copy()
                        self.plan_future=self.plan_pool.submit(self.plan_planner.transit,obs.q.copy(),self.home.copy(),self.target)
                        self._phase("planning_return",t)
                    else:
                        path = self.planner.transit(obs.q, self.home, self.target)
                        self._start_path(path, t)
                        self._phase("return", t)
                else:
                    self.report.returned = True
                    self._phase("verify", t)
            return goal
        if self.phase == "align":
            if self.wrist_servo_enabled:
                return self._align_wrist(obs)
            target = self._observe(obs)
            if target is not None:
                if np.linalg.norm(target.point-self.target.point) > 0.025 or target.normal @ self.target.normal < np.cos(np.radians(8)):
                    raise RuntimeError("target moved outside the local correction corridor")
                self.target = target
                goal_point = target.point-(self.cfg["tip_radius_m"]+self.cfg["approach_m"])*target.normal
                error_vector = goal_point-self.robot.tip(obs.q)
                # Alignment corrects the tangent plane. The press stage controls
                # remaining normal distance, so do not wind up normal correction here.
                error_vector -= (error_vector @ target.normal)*target.normal
                self.alignment_bias += 2.5*error_vector*min(.2,t-self.last_correction_t)
                self.last_correction_t = t
                self.alignment_bias = np.clip(self.alignment_bias,-.015,.015)
                goal = self.robot.solve(goal_point+self.alignment_bias, obs.q, self.axis)
                if not self.collision.edge(obs.q, goal, target):
                    raise PlanningError("unsafe visual correction: " + self.collision.last_reason)
                self.alignment_goal = goal
                self.report.corrections += 1
                error = np.linalg.norm(error_vector)
                self.report.phases[-1]["last_error_m"] = float(error)
                gap=float((target.point-self.robot.tip(obs.q))@target.normal-self.cfg["tip_radius_m"])
                if error < self.cfg["alignment_tolerance_m"] and gap>.008 and np.max(np.abs(obs.dq[ARM])) < .2:
                    self.press_goal = self.robot.tip(goal).copy()
                    self.freeze_target = target
                    self.report.alignment_camera = target.camera
                    self.report.surface_camera = target.camera
                    self._phase("press", t)
            if self.phase == "align" and (t-self.phase_t > self.cfg["alignment_timeout_s"] or
                                          self.report.corrections >= self.cfg["max_corrections"]):
                raise RuntimeError("alignment did not converge within its budget: " + self.vision.error)
            return self.alignment_goal
        if self.phase == "press":
            target, c = self.freeze_target, self.cfg[self.cfg["mode"]]
            tip = self.robot.tip(obs.q)
            penetration = float((tip-target.point) @ target.normal + self.cfg["tip_radius_m"])
            self.report.phases[-1]["max_measured_penetration_m"]=max(penetration,self.report.phases[-1].get("max_measured_penetration_m",-1))
            if penetration>-.002:
                self.press_force=min(c["feedforward_force_n"],self.press_force+dt*c["feedforward_force_n"]/.4)
            lateral = tip-target.point
            lateral -= (lateral @ target.normal)*target.normal
            if (penetration > c["max_measured_penetration_m"] or
                    (penetration > -.01 and np.linalg.norm(lateral) > target.radius-self.cfg["tip_radius_m"])):
                raise RuntimeError("measured fingertip left the permitted pressing region")
            command_pen = float((self.press_goal-target.point) @ target.normal + self.cfg["tip_radius_m"])
            lag = command_pen-penetration
            # This is a contact ESTIMATE from following error, not measured force.
            self.report.contact_estimated |= penetration > -0.001 and lag > 0.0015
            if penetration >= c["stroke_m"]-.0002 or (self.report.contact_estimated and command_pen >= c["max_command_penetration_m"]-.0009):
                self.hold_started = self.hold_started or t
                if t-self.hold_started >= self.cfg["hold_s"]:
                    self._start_retreat(obs)
                    return obs.q.copy()
            else:
                speed = self.cfg["approach_speed_m_s"] if penetration < -0.01 else c.get("close_speed_m_s",self.cfg["close_speed_m_s"]) if penetration < -0.002 else c.get("press_speed_m_s",self.cfg["press_speed_m_s"])
                advance = min(speed*dt, c["max_command_penetration_m"]-.0008-command_pen)
                if advance <= 1e-8 and t-self.phase_t > 5:
                    raise RuntimeError("press reached command limit without sufficient motion")
                self.press_goal += max(0, advance)*target.normal
            return self.robot.solve(self.press_goal, obs.q, self.axis)
        if self.phase == "verify":
            if self.cfg["mode"] == "button":
                observed = self._observe(obs)
                if observed is not None and self.baseline_brightness is not None and observed.brightness is not None:
                    self.report.press_confirmed = observed.brightness-self.baseline_brightness > 20
            # Bottle contact cannot be proved by RGB alone. Simulation scores physical contact separately.
            if self.report.press_confirmed or t-self.phase_t > 0.8:
                self._phase("done", t)
            return self.home
        return self.home

    def act(self, obs):
        if self.phase in ("done","failed"):
            return Command(self.last.copy(),True,self.phase,self.report.error)
        if self.home is None:
            self.home, self.last, self.last_t = obs.q.copy(), obs.q.copy(), obs.t
            self.limiter = CommandLimiter(obs.q, self.cfg,self.robot.lower,self.robot.upper)
            self.started = self.phase_t = obs.t
        dt = obs.t-self.last_t
        if dt == 0:
            return Command(self.last.copy(), phase=self.phase)
        try:
            if dt < 0:
                raise RuntimeError("observation/control clock discontinuity")
            if dt>.15:
                if self.phase in ("look","planning","planning_return","verify"):
                    # In these phases the command is a hold. Account for wall time
                    # in deadlines, but do not turn scheduler jitter into a large move.
                    dt=.02
                else:
                    raise RuntimeError("observation/control clock discontinuity")
            if obs.t-self.started > self.cfg["cycle_timeout_s"]:
                raise RuntimeError("cycle deadline exceeded")
            if np.max(np.abs(obs.q[ARM]-self.last[ARM])) > self.cfg["joint_tracking_limit_rad"]:
                raise RuntimeError("excessive arm tracking error")
            desired = self._desired(obs, dt)
            command = self.robot.gravity_command(desired, self.kp) if self.cfg["gravity_mode"]=="position" else desired.copy()
            # Waist and opposite arm must stay at the initial measured pose.
            command[:22] = self.home[:22]
            output, velocity = self.limiter.next(command, dt)
            contact = self.phase in ("press", "retract")
            if not self.collision.edge(self.last, output, self.target, contact):
                raise PlanningError("command interpolation: " + self.collision.last_reason)
            if not self.collision.edge(obs.q, output, self.target, contact):
                raise PlanningError("measured-to-command envelope: " + self.collision.last_reason)
            self.limiter.commit(output, velocity)
            self.last = output
        except (ValueError, RuntimeError) as exc:
            # Keep the last checked command; do not invent an unchecked emergency return.
            self.report.error = str(exc)
            self._phase("failed", obs.t)
        self.last_t = obs.t
        self.report.elapsed_s = obs.t-self.started
        self.report.perception_ms = self.localizer.timings
        tau=self.robot.gravity_torque(obs.q) if self.cfg["gravity_mode"]=="torque" else None
        if tau is not None and self.phase=="press":
            tau+=self.robot.pressing_torque(obs.q,self.target.normal,self.press_force)
            tau[ARM]=np.clip(tau[ARM],-self.robot.model.effortLimit[ARM],self.robot.model.effortLimit[ARM])
        return Command(self.last.copy(), self.phase in ("done", "failed"), self.phase, self.report.error,tau)

    def close(self):
        self.vision.close()
        if self.plan_pool:
            self.plan_pool.shutdown(wait=True,cancel_futures=True)

    def result(self):
        return asdict(self.report)
