"""Stream a verified reference with bounded visual correction; no capture/settle phases."""
import time
import numpy as np
import mujoco

from alignment import JointCommandFilter
from robust_scene import build_scene_model, update_robot, contact_metrics
from robust_planner import PERIOD, reference_torque
from robust_perception import CorrectionGate
from robust_policy import execution_policy
from robust_control import (BottleFeedback, fade_return_offset, reference_command,
                            waist_support_torque, bottle_runtime_state)


# Only the right arm is commanded by this task. The held left arm may settle
# during the SDK handover; its measured pose is still used for collision checks.
START_MONITORED_JOINTS = tuple(range(15)) + tuple(range(22,29))


def button_fixed_interval(scene, candidate):
    """The verified path, rather than elapsed time, bounds contact occlusion."""
    depth = (np.asarray(candidate.tcp)-scene.face) @ np.asarray(scene.normal)+scene.tip_radius
    close = np.flatnonzero(depth[:candidate.press_end+1] >= -.005-1e-9)
    retreat = np.flatnonzero(depth[candidate.press_end+1:] <= -.05+1e-9)
    if not len(close) or not len(retreat):
        raise ValueError('Button trajectory must include final 5mm approach and 5cm retreat')
    return int(close[0]), int(candidate.press_end+1+retreat[0])


def validate_start(scene, configuration, observed, *, record_deviations=False):
    q, imu, _, leg, gyro = configuration
    policy = execution_policy(record_deviations)
    delta = np.abs(np.array(q)-scene.q)
    if leg > policy.leg_speed_rad_s or gyro > policy.gyro_rad_s:
        raise RuntimeError('Robot moved while planning; discard snapshot')
    warnings = []
    checks = [(np.max(delta[list(START_MONITORED_JOINTS)]) > .01
        or np.max(np.abs(np.array(imu)-scene.imu)) > np.deg2rad(.5),
        'Robot moved while planning; discard snapshot',True),
        (np.linalg.norm(np.array(observed.geometry['face'])-scene.face) > max(.003,scene.uncertainty_m),
        'Target moved while planning; discard snapshot',scene.task=='bottle'),
        (scene.task == 'bottle' and abs(observed.geometry.get('object_height',scene.object_height)-scene.object_height) > .01,
        'Bottle vertical extent changed while planning',True)]
    for changed,reason,recordable in checks:
        if changed:
            if not (record_deviations and recordable):
                raise RuntimeError(reason)
            warnings.append(reason)
    return warnings


def execute(plan, model_path, calibration, motion, configuration, observe, settings,
            *, now=time.monotonic, set_torque=lambda tau: None, diagnostic=False,
            record_deviations=False, set_waist_torque=None, allow_stale_observation=False,
            observe_held=None, feedback_state=None):
    if plan.candidate is None:
        raise RuntimeError('No fully verified robust plan; no motion allowed')
    s, c = plan.scene, plan.candidate
    if s.task == 'button' and ('button_feedback' in getattr(c,'motion_metrics',{})) != (settings.button_feedback == 'joint'):
        raise RuntimeError('Button feedback mode differs from the verified plan')
    if s.task == 'button' and settings.button_feedback == 'joint':
        from button_tracking_runtime import execute_feedback
        return execute_feedback(plan,model_path,calibration,motion,configuration,observe,settings,
            now=now,set_torque=set_torque,diagnostic=diagnostic,record_deviations=record_deviations,
            set_waist_torque=set_waist_torque,feedback_state=feedback_state)
    if allow_stale_observation and (s.task != 'bottle' or settings.bottle_feedback != 'off'):
        raise ValueError('Stale observations require bottle feedback off')
    if observe_held is not None and (s.task != 'button' or diagnostic):
        raise ValueError('Contact occlusion requires a monitored full button press')
    fixed_interval = button_fixed_interval(s,c) if observe_held is not None else None
    if s.task == 'bottle' and c.motion_metrics.get('bottle_feedback_mode', 'off') != settings.bottle_feedback:
        raise RuntimeError('Bottle feedback mode differs from the verified plan')
    report = dict(schema='g1-robust-execution-v1', task=s.task, completed=False, motion_completed=False,
                  home_completed=False, release_completed=False, physical_push_success=None,
                  stopped_for_measurement=0, remeasurement_count=0, commands=[], correction_steps=[])
    report['limits'] = settings.motion_limits.to_dict()
    policy = execution_policy(record_deviations)
    report['execution_policy'] = policy.to_dict()
    report['target_motion_s'] = settings.motion_duration_s
    report['bottle_feedback'] = settings.bottle_feedback
    report.update(allow_stale_observation=allow_stale_observation,stale_observation_commands=0)
    report.update(record_deviations=record_deviations,warnings=[],tracking_samples=[],max_fk_error_m=0.)
    if fixed_interval is not None:
        report['button_fixed_target'] = dict(start_index=fixed_interval[0],end_index=fixed_interval[1],
            interval_timeout_s=None,stale_observation_commands=0)
    def deviation(reason):
        if not record_deviations:
            raise RuntimeError(reason)
        if reason not in report['warnings']:
            report['warnings'].append(reason)
    check = build_scene_model(model_path, calibration, s)
    start = configuration()
    report['warnings'].extend(validate_start(s,start,observe(),record_deviations=record_deviations))
    gate = BottleFeedback(s) if s.task == 'bottle' else CorrectionGate(s)
    command_filter = JointCommandFilter(c.q[0], settings.motion_limits)
    n = np.array(s.normal)
    started = now()
    enabled = False
    peak_speed = peak_acceleration = 0.
    try:
        set_torque(c.tau[0])
        if set_waist_torque is not None:
            set_waist_torque(c.waist_tau[0] if c.waist_tau is not None else waist_support_torque(check,c.q[0]))
        enabled = True  # A partially failed enable still needs release.
        motion.enable()
        report['enable_s'] = now()-started
        # Enabling must not invalidate the observed state used to select the path.
        enabled_state = configuration()
        joint_delta = np.array(enabled_state[0])-np.array(s.q)
        peak_index = max(START_MONITORED_JOINTS,key=lambda index:abs(joint_delta[index]))
        report['post_enable_state'] = dict(motor_q_rad=list(map(float,enabled_state[0])),
            imu_rpy_rad=list(map(float,enabled_state[1])),state_age_s=now()-enabled_state[2],
            leg_speed_rad_s=float(enabled_state[3]),gyro_rad_s=float(enabled_state[4]))
        report['post_enable_delta'] = dict(max_joint_index=int(peak_index),
            max_joint_rad=float(abs(joint_delta[peak_index])),
            monitored_joint_indices=list(START_MONITORED_JOINTS),
            left_arm_delta_rad=joint_delta[15:22].tolist(),left_arm_excluded_from_start_check=True,
            imu_rpy_rad=(np.array(enabled_state[1])-np.array(s.imu)).tolist())
        report['warnings'].extend(validate_start(s,enabled_state,observe(),record_deviations=record_deviations))
        last_tick = now()
        trajectory_started = now()
        end = c.press_start if diagnostic else len(c.q)
        for i in range(end):
            cycle = now()
            max_age = policy.state_timeout_s
            if cycle-last_tick > max_age:
                raise RuntimeError(f'Control update exceeded {max_age*1000:.0f}ms')
            q, imu, stamp, leg, gyro = configuration()
            if (not 0 <= now()-stamp <= max_age or leg > policy.leg_speed_rad_s
                    or gyro > policy.gyro_rad_s):
                raise RuntimeError('Stale or moving robot state')
            if np.max(np.abs(np.array(imu)-s.imu)) > np.deg2rad(3) or np.max(np.abs(np.array(q)[12:15]-np.array(s.q)[12:15])) > np.deg2rad(3):
                deviation('Body posture changed')
            if np.max(np.abs(np.array(q)[:12]-np.array(s.q)[:12])) > .025:
                deviation('Leg posture changed')
            locked = fixed_interval is not None and fixed_interval[0] <= i <= fixed_interval[1]
            # Require a fresh measurement at lock entry. Later occlusion may
            # reuse the measured target, while camera-worker errors still raise.
            observation = (observe_held() if locked and i > fixed_interval[0] else observe())
            observation_age = now()-observation.captured_at
            if fixed_interval is not None and (observation_age < 0 or
                    ((not locked or i == fixed_interval[0]) and observation_age > .5)):
                raise RuntimeError('No fresh button observation outside contact interval')
            progress = max(0., float((c.tcp[min(i,c.press_end)]-np.array(s.face))@n+s.tip_radius))
            freeze = s.task == 'button' and (c.tcp[i]-s.face)@n+s.tip_radius >= -.005
            if s.task == 'bottle':
                face,progress,observation_age,predicted = bottle_runtime_state(
                    s,c,i,observation,now(),allow_stale=allow_stale_observation)
                if predicted:
                    # Feedback is off: follow the verified path. An old image
                    # cannot measure progress or prove that the bottle stopped.
                    offset = np.zeros(3)
                    report['stale_observation_commands'] += 1
                else:
                    if abs(observation.geometry.get('object_height',s.object_height)-s.object_height) > .01:
                        deviation('Bottle vertical extent changed')
                    displacement = face-s.face
                    offset = gate.update(displacement,progress,observation.frame_id,
                                         freeze=settings.bottle_feedback == 'off' or i < c.press_start or i > c.press_end)
                offset = fade_return_offset(offset,c,i)
            else:
                if locked and observation_age > .5:
                    offset = gate.offset.copy()
                    report['button_fixed_target']['stale_observation_commands'] += 1
                else:
                    offset = gate.update(observation, progress, freeze=locked or freeze or i > c.press_end)
                if fixed_interval is not None and i == fixed_interval[0]:
                    report['button_fixed_target'].update(face_m=list(observation.geometry['face']),
                        normal=list(observation.geometry['normal']),offset_m=offset.tolist(),
                        frame_id=observation.frame_id,captured_at_s=observation.captured_at)
                if fixed_interval is not None and i > fixed_interval[1]:
                    remaining = len(c.q)-1-i
                    offset = offset*max(0.,remaining/max(1,len(c.q)-1-fixed_interval[1]))
            pose = update_robot(check,q,imu)
            if s.task == 'bottle':
                adr = check.model.joint('robust_object_joint').qposadr[0]
                pose[adr:adr+3] = (face[0]+s.radius,face[1],s.base_height+s.object_height/2)
                check.update(pose)
            measured_tip = check.tip_position().copy()
            # Compute bounded correction about the certified reference, not the last IK solution.
            previous_velocity = command_filter.velocity.copy()
            if s.task == 'bottle':
                goal, tau, target_tcp = reference_command(check,c,i,offset,command_filter,settings.gravity_scale)
            else:
                ref_pose = pose.copy(); ref_pose[check.qadr] = c.q[i]; check.update(ref_pose)
                jac = np.zeros((3, check.model.nv))
                mujoco.mj_jacSite(check.model, check.data, jac, None, check.site_id)
                dq = np.linalg.lstsq(jac[:, check.dadr], offset, rcond=.01)[0]
                goal = command_filter.next_goal(c.q[i]+dq, PERIOD)
                target_tcp = c.tcp[i]
            command_tracking_error = float(np.linalg.norm(measured_tip-target_tcp))
            initial_target_tcp = c.tcp[i]+offset
            tracking_error = float(np.linalg.norm(measured_tip-initial_target_tcp))
            report['max_fk_error_m'] = max(report['max_fk_error_m'],tracking_error)
            report['tracking_samples'].append(dict(time_s=now()-started,
                state_received_at_s=float(stamp),measured_imu_rpy_rad=list(map(float,imu)),
                error_m=tracking_error,measured_xyz_m=measured_tip.tolist(),reference_xyz_m=initial_target_tcp.tolist(),
                command_reference_xyz_m=target_tcp.tolist(),command_tracking_error_m=command_tracking_error,
                waist_q_rad=list(map(float,np.array(q)[12:15])),leg_speed_rad_s=float(leg),gyro_rad_s=float(gyro)))
            if tracking_error > .01:
                deviation('Measured FK tracking error exceeds 10mm')
            peak_speed = max(peak_speed, float(np.max(np.abs(command_filter.velocity))))
            peak_acceleration = max(peak_acceleration, float(np.max(np.abs(
                command_filter.velocity-previous_velocity)))/PERIOD)
            check.update(pose)
            # Revalidate the actual adjacent path, retaining the initial B wall.
            check.validate_path(goal, allow_tip_contact=not diagnostic,
                                allowed_tip_geoms={check.model.geom('robust_target_geom').id})
            commanded_pose = pose.copy(); commanded_pose[check.qadr] = goal; check.update(commanded_pose)
            if contact_metrics(check,s,contact=not diagnostic) < .003:
                raise RuntimeError('Corrected command has less than 3mm obstacle clearance')
            check.update(pose)
            if i == 0:
                motion.start_approach()
            if s.task != 'bottle':
                tau = reference_torque(check, goal, command_filter.velocity, settings.gravity_scale)
            set_torque(tau)
            waist_tau = c.waist_tau[i] if c.waist_tau is not None else waist_support_torque(check,goal)
            if set_waist_torque is not None:
                set_waist_torque(waist_tau)
            motion.stream(goal, velocity=command_filter.velocity)
            report['commands'].append(dict(time_s=now()-started, phase=c.phases[i], q=goal.tolist(),
                velocity_rad_s=command_filter.velocity.tolist(), tau=tau.tolist(),
                measured_q=np.array(q).tolist(), observation_frame_id=observation.frame_id,
                waist_tau_nm=waist_tau.tolist(),
                observation_captured_at=observation.captured_at))
            if s.task == 'bottle':
                report['commands'][-1].update(observation_age_s=float(observation_age),
                    bottle_pose_source='reference_prediction' if predicted else 'observation',
                    bottle_face_m=face.tolist())
            elif fixed_interval is not None:
                report['commands'][-1].update(button_target_locked=locked,
                    observation_age_s=float(observation_age),
                    button_observation_source='held' if observation_age > .5 else 'fresh')
            report['correction_steps'].append(dict(frame_id=observation.frame_id, offset_m=offset.tolist()))
            motion.wait(max(0., PERIOD-(now()-cycle)))
            last_tick = cycle
        if diagnostic:
            # Contact-free diagnostic returns over the checked approach reference.
            for goal in c.q[:end][::-1]:
                config = configuration(); pose = update_robot(check,config[0],config[1])
                goal = command_filter.next_goal(goal,PERIOD)
                check.validate_path(goal)
                commanded_pose = pose.copy(); commanded_pose[check.qadr] = goal; check.update(commanded_pose)
                if contact_metrics(check,s,contact=False) < .003:
                    raise RuntimeError('Diagnostic return has less than 3mm obstacle clearance')
                check.update(pose)
                set_torque(reference_torque(check,goal,command_filter.velocity,settings.gravity_scale))
                if set_waist_torque is not None:
                    set_waist_torque(waist_support_torque(check,goal))
                motion.stream(goal,velocity=command_filter.velocity); motion.wait(PERIOD)
        if hasattr(motion,'finish_stream'):
            motion.finish_stream()
        report['trajectory_s'] = now()-trajectory_started
        home_started = now()
        motion.wait(.3)
        home_confirmed = True
        home_errors = []
        for _ in range(3):
            error = np.max(np.abs(np.array(configuration()[0])[22:29]-np.array(s.q)[22:29]))
            home_errors.append(float(error))
            home_confirmed &= error <= np.deg2rad(2)
            motion.wait(PERIOD)
        report.update(motion_completed=not diagnostic, diagnostic_completed=diagnostic,
                      home_completed=bool(home_confirmed), home_confirmation_s=now()-home_started,
                      home_errors_rad=home_errors,home_max_error_rad=max(home_errors),
                      home_tolerance_rad=float(np.deg2rad(2)))
        if not report['home_completed']:
            deviation('Home return not confirmed')
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
