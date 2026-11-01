"""Stream a verified reference with bounded visual correction; no capture/settle phases."""
import time
import numpy as np
import mujoco

from alignment import JointCommandFilter
from robust_scene import build_scene_model, update_robot, contact_metrics
from robust_planner import PERIOD, reference_torque
from robust_perception import CorrectionGate


def validate_start(scene, configuration, observed):
    q, imu, _, leg, gyro = configuration
    if (np.max(np.abs(np.array(q)-scene.q)) > .01
            or np.max(np.abs(np.array(imu)-scene.imu)) > np.deg2rad(.5)
            or leg > .15 or gyro > .10):
        raise RuntimeError('Robot moved while planning; discard snapshot')
    if np.linalg.norm(np.array(observed.geometry['face'])-scene.face) > max(.003, scene.uncertainty_m):
        raise RuntimeError('Target moved while planning; discard snapshot')
    if scene.task == 'bottle' and abs(observed.geometry.get('object_height',scene.object_height)-scene.object_height) > .01:
        raise RuntimeError('Bottle vertical extent changed while planning')


def execute(plan, model_path, calibration, motion, configuration, observe, settings,
            *, now=time.monotonic, set_torque=lambda tau: None, diagnostic=False):
    if plan.candidate is None:
        raise RuntimeError('No fully verified robust plan; no motion allowed')
    s, c = plan.scene, plan.candidate
    report = dict(schema='g1-robust-execution-v1', task=s.task, completed=False, motion_completed=False,
                  home_completed=False, release_completed=False, physical_push_success=None,
                  stopped_for_measurement=0, remeasurement_count=0, commands=[], correction_steps=[])
    report['limits'] = settings.motion_limits.to_dict()
    report['target_motion_s'] = settings.motion_duration_s
    check = build_scene_model(model_path, calibration, s)
    start = configuration()
    validate_start(s, start, observe())
    gate = CorrectionGate(s)
    command_filter = JointCommandFilter(c.q[0], settings.motion_limits)
    n = np.array(s.normal)
    started = now()
    enabled = False
    peak_speed = peak_acceleration = 0.
    try:
        set_torque(c.tau[0])
        enabled = True  # A partially failed enable still needs release.
        motion.enable()
        report['enable_s'] = now()-started
        # Enabling must not invalidate the observed state used to select the path.
        validate_start(s, configuration(), observe())
        last_tick = now()
        trajectory_started = now()
        end = c.press_start if diagnostic else len(c.q)
        for i in range(end):
            cycle = now()
            if cycle-last_tick > .1:
                raise RuntimeError('Control update exceeded 100ms')
            q, imu, stamp, leg, gyro = configuration()
            if not 0 <= now()-stamp <= .1 or leg > .15 or gyro > .10:
                raise RuntimeError('Stale or moving robot state')
            if np.max(np.abs(np.array(imu)-s.imu)) > np.deg2rad(3) or np.max(np.abs(np.array(q)[12:15]-np.array(s.q)[12:15])) > np.deg2rad(3):
                raise RuntimeError('Body posture changed')
            if np.max(np.abs(np.array(q)[:12]-np.array(s.q)[:12])) > .025:
                raise RuntimeError('Leg posture changed')
            observation = observe()
            progress = max(0., float((c.tcp[min(i,c.press_end)]-np.array(s.face))@n+s.tip_radius))
            freeze = s.task == 'button' and (c.tcp[i]-s.face)@n+s.tip_radius >= -.005
            offset = gate.update(observation, progress, freeze=freeze or i > c.press_end)
            pose = update_robot(check,q,imu)
            if s.task == 'bottle':
                adr = check.model.joint('robust_object_joint').qposadr[0]
                face = observation.geometry['face']
                pose[adr:adr+3] = (face[0]+s.radius,face[1],s.base_height+s.object_height/2)
                check.update(pose)
            if np.linalg.norm(check.tip_position()-c.tcp[i]) > .01:
                raise RuntimeError('Measured FK tracking error exceeds 10mm')
            # Compute bounded correction about the certified reference, not the last IK solution.
            ref_pose = pose.copy(); ref_pose[check.qadr] = c.q[i]; check.update(ref_pose)
            jac = np.zeros((3, check.model.nv))
            mujoco.mj_jacSite(check.model, check.data, jac, None, check.site_id)
            dq = np.linalg.lstsq(jac[:, check.dadr], offset, rcond=.01)[0]
            previous_velocity = command_filter.velocity.copy()
            goal = command_filter.next_goal(c.q[i]+dq, PERIOD)
            peak_speed = max(peak_speed, float(np.max(np.abs(command_filter.velocity))))
            peak_acceleration = max(peak_acceleration, float(np.max(np.abs(
                command_filter.velocity-previous_velocity)))/PERIOD)
            check.update(pose)
            # Revalidate the actual adjacent path, retaining the initial B wall.
            check.validate_path(goal, allow_tip_contact=c.phases[i] in ('contact','press','retract'),
                                allowed_tip_geoms={check.model.geom('robust_target_geom').id})
            commanded_pose = pose.copy(); commanded_pose[check.qadr] = goal; check.update(commanded_pose)
            if contact_metrics(check,s,contact=c.phases[i] in ('contact','press','retract')) < .003:
                raise RuntimeError('Corrected command has less than 3mm obstacle clearance')
            check.update(pose)
            if i == 0:
                motion.start_approach()
            tau = reference_torque(check, goal, command_filter.velocity, settings.gravity_scale)
            set_torque(tau)
            motion.stream(goal, velocity=command_filter.velocity)
            report['commands'].append(dict(time_s=now()-started, phase=c.phases[i], q=goal.tolist(),
                velocity_rad_s=command_filter.velocity.tolist(), tau=tau.tolist(),
                measured_q=np.array(q).tolist(), observation_frame_id=observation.frame_id,
                observation_captured_at=observation.captured_at))
            report['correction_steps'].append(dict(frame_id=observation.frame_id, offset_m=offset.tolist()))
            motion.wait(max(0., PERIOD-(now()-cycle)))
            last_tick = cycle
        if diagnostic:
            # Contact-free diagnostic returns over the checked approach reference.
            for goal in c.q[:end][::-1]:
                config = configuration(); pose = update_robot(check,config[0],config[1])
                goal = command_filter.next_goal(goal,PERIOD)
                check.validate_path(goal)
                set_torque(reference_torque(check,goal,command_filter.velocity,settings.gravity_scale))
                motion.stream(goal,velocity=command_filter.velocity); motion.wait(PERIOD)
        report['trajectory_s'] = now()-trajectory_started
        home_started = now()
        motion.wait(.3)
        home_confirmed = True
        for _ in range(3):
            error = np.max(np.abs(np.array(configuration()[0])[22:29]-np.array(s.q)[22:29]))
            home_confirmed &= error <= np.deg2rad(2)
            motion.wait(PERIOD)
        report.update(motion_completed=not diagnostic, diagnostic_completed=diagnostic,
                      home_completed=bool(home_confirmed), home_confirmation_s=now()-home_started)
        if not report['home_completed']:
            raise RuntimeError('Home return not confirmed')
        report['completed'] = True
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if enabled:
            release_started = now()
            try:
                motion.close()
            except Exception as exc:
                motion.release_error = str(exc)
            report['release_s'] = now()-release_started
            report['release_completed'] = not bool(getattr(motion,'release_error',None))
            if not report['release_completed']:
                report['completed'] = False
                report['release_error'] = motion.release_error
        report.update(duration_s=now()-started, motion_s=now()-started,
                      max_joint_speed_rad_s=peak_speed, max_joint_acceleration_rad_s2=peak_acceleration,
                      speed_limit_fraction=peak_speed/settings.joint_speed_rad_s,
                      acceleration_limit_fraction=peak_acceleration/settings.joint_acceleration_rad_s2)
    return report
