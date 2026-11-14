"""Adapters for the shared measured-joint controller. No SDK import in simulation."""
import time

import mujoco
import numpy as np

from button_tracking import ButtonTrackingController, PERIOD
from robust_scene import build_scene_model, clone_kin, contact_metrics
from robust_control import waist_support_torque
from robust_policy import execution_policy


def execute_feedback(plan, model_path, calibration, motion, configuration, observe, settings,
                     *, now=time.monotonic, set_torque=lambda tau: None, diagnostic=False,
                     record_deviations=False, set_waist_torque=None, feedback_state=None,
                     lamp_baseline=None, lamp_min_rise=30., observe_lamp=None, **unused):
    from robust_execution import validate_start
    from button_collection import CollectionCorrectionGate
    from robust_perception import CorrectionGate
    scene, candidate = plan.scene, plan.candidate
    kin = build_scene_model(model_path, calibration, scene)
    controller = ButtonTrackingController(kin, scene, candidate, settings, diagnostic=diagnostic)
    gate = CollectionCorrectionGate(scene) if settings.button_policy == 'collection' else CorrectionGate(scene)
    policy = execution_policy(record_deviations)
    report = dict(schema='g1-robust-execution-v1',task='button',completed=False,
        motion_completed=False,diagnostic_completed=False,home_completed=False,release_completed=False,
        physical_push_success=None,button_feedback='joint',button_policy=settings.button_policy,
        limits=settings.motion_limits.to_dict(),execution_policy=policy.to_dict(),
        feedback_controller=candidate.motion_metrics['button_feedback'],
        stopped_for_measurement=0,remeasurement_count=0,commands=[],tracking_samples=[],
        correction_steps=[],warnings=[],max_fk_error_m=0.)
    report['warnings'].extend(validate_start(scene,configuration(),observe(),record_deviations=record_deviations))
    started = now()
    enabled = False
    lamp_monitor = None
    try:
        set_torque(candidate.tau[0])
        if set_waist_torque:
            set_waist_torque(waist_support_torque(kin,candidate.q[0]))
        enabled = True
        motion.enable()
        report['enable_s'] = now()-started
        enabled_state = configuration()
        controller.start_from_command(motion.current_target() if hasattr(motion,'current_target') else candidate.q[0])
        report['post_enable_state'] = dict(motor_q_rad=list(map(float,enabled_state[0])),
            imu_rpy_rad=list(map(float,enabled_state[1])),state_received_at_s=enabled_state[2])
        motion.start_approach()
        trajectory_started = last_tick = now()
        if not diagnostic and lamp_baseline is not None:
            from button_feedback import PressLampMonitor
            lamp_monitor = PressLampMonitor(lamp_baseline, lamp_min_rise, trajectory_started)
        while not controller.done:
            cycle = now()
            config = configuration()
            q, imu, stamp, leg, gyro = config
            if (cycle-last_tick > policy.state_timeout_s or not 0 <= now()-stamp <= policy.state_timeout_s
                    or leg > policy.leg_speed_rad_s or gyro > policy.gyro_rad_s):
                raise RuntimeError('Stale or moving robot state')
            observation = observe()  # Camera-worker errors still surface with held frames.
            if lamp_monitor is not None and not controller.retreat:
                lamp_observation = observe_lamp() if observe_lamp is not None else observation
                trigger_index = controller.index
                if (lamp_observation is not None and lamp_monitor.observe(lamp_observation, cycle)
                        and controller.return_on_lamp(cycle-trajectory_started)):
                    report['lamp_early_return'] = dict(baseline=float(lamp_baseline),
                        min_rise=float(lamp_min_rise), frames=list(lamp_monitor.frames),
                        confirmed_at_s=cycle, reference_index=int(trigger_index),
                        return_method=('optimized_shortcut' if getattr(controller,'lamp_return_shortcut',False)
                                       else 'checked_reference'))
                    print('点灯を検出しました。押し込みを止め、追従と制限を監視しながら速く戻ります。',flush=True)
            depth = float((controller.tcps[controller.index]-scene.face)@controller.normal+scene.tip_radius)
            # Geometry does not follow the button cylinder as it is depressed.
            # Freeze visual retargeting at contact; joint/body feedback continues.
            offset = (gate.offset.copy() if controller.retreat or depth >= -.005
                      else gate.update(observation,0.))
            if controller.retreat:
                offset *= np.clip((len(controller.qs)-1-controller.index)/max(1,len(controller.qs)-1-controller.press_end),0.,1.)
            row = controller.step(q,imu,stamp,cycle-trajectory_started,offset)
            if controller.retreat and 'retreat_started_at_s' not in report:
                report['retreat_started_at_s'] = cycle
            set_torque(row['tau'])
            waist_tau = waist_support_torque(kin,row['q'])
            if set_waist_torque:
                set_waist_torque(waist_tau)
            motion.stream(row['q'],velocity=row['velocity'])
            report['commands'].append(dict(time_s=now()-started,phase=row['phase'],
                reference_index=row['index'],reference_held=row['reference_held'],
                q=row['q'].tolist(),velocity_rad_s=row['velocity'].tolist(),
                tau=row['tau'].tolist(),waist_tau_nm=waist_tau.tolist(),
                torque_bias_nm=row['torque_bias'].tolist(),measured_q=list(map(float,q)),
                observation_frame_id=observation.frame_id,observation_age_s=now()-observation.captured_at,
                state_received_at_s=stamp,reference_depth_m=row['reference_depth_m'],
                measured_fk_depth_m=row['actual_depth_m'],guard_holds=row['guard_holds']))
            if feedback_state is not None:
                report['commands'][-1]['joint_feedback'] = feedback_state()
            report['tracking_samples'].append(dict(time_s=now()-started,state_received_at_s=stamp,
                error_m=row['error_m'],measured_xyz_m=row['measured_tip'].tolist(),
                reference_xyz_m=row['target'].tolist(),measured_imu_rpy_rad=list(map(float,imu)),
                waist_q_rad=list(map(float,q[12:15]))))
            report['max_fk_error_m'] = max(report['max_fk_error_m'],row['error_m'])
            report['correction_steps'].append(dict(frame_id=observation.frame_id,offset_m=offset.tolist()))
            motion.wait(max(0., PERIOD-(now()-cycle)))
            last_tick = cycle
        if hasattr(motion,'finish_stream'):
            motion.finish_stream()
        report['trajectory_s'] = now()-trajectory_started
        home = configuration()
        error = float(np.max(np.abs(np.asarray(home[0])[22:29]-candidate.q[0])))
        report.update(home_completed=bool(error <= np.deg2rad(2)),home_max_error_rad=error,
            motion_completed=not diagnostic and controller.error is None,
            diagnostic_completed=diagnostic and controller.error is None,completed=controller.error is None)
        if report.get('lamp_early_return') and not report['home_completed']:
            report.update(completed=False,error='点灯後の開始姿勢への復帰誤差が2度を超えました')
        if controller.error:
            report.update(error=controller.error,recovery_completed=True)
    except KeyboardInterrupt:
        report.update(error='操作者による中断',interrupted=True)
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if enabled:
            try:
                motion.close()
                report['release_completed'] = not bool(getattr(motion,'release_error',None))
            except Exception as exc:
                report['release_error'] = str(exc)
        if not report['release_completed']:
            report['completed'] = False
        report.update(duration_s=now()-started,motion_s=now()-started,guard_holds=controller.guard_holds,
            feedback_normal_deviations=getattr(gate,'normal_deviations',[]),
            target_position_stop_enabled=settings.button_policy!='collection',
            target_position_changes=getattr(gate,'position_changes',[]))
        controller.integral.bias[:] = 0.
    return report


def physical_feedback_trial(base_kin, kin, scene, candidate, condition, deadline, started):
    """Use measured simulated state on every tick, with independent FK data.

    Pelvis is fixed as in the existing planner; waist, left arm and right arm
    remain dynamic under the SDK gains. This does not validate standing balance.
    """
    from robust_planner import SearchSettings, _trial_failure_reasons
    from trajectory import ARM_JOINTS
    settings = SearchSettings(**candidate.motion_metrics['button_tracking_settings'])
    controller = ButtonTrackingController(clone_kin(base_kin),scene,candidate,settings)
    m,d = kin.model,kin.data
    clearance = .05
    peak_press = peak_force = tracking_max = 0.
    dynamic_margin = float('inf')
    limits = m.jnt_range[[m.joint(name).id for name in ARM_JOINTS]]
    waist_dadr = np.array([m.joint(i).dofadr[0] for i in (12,13,14)])
    object_adr = m.joint('robust_object_joint').qposadr[0]
    target = m.geom('robust_target_geom').id
    elapsed = 0.
    reason = None
    lamp_trigger_index = condition.get('_lamp_trigger_index')
    lamp_triggered = False
    try:
        while not controller.done:
            if time.perf_counter() >= deadline:
                return dict(completed=False,reason='deadline',wall_s=time.perf_counter()-started)
            if (lamp_trigger_index is not None and not controller.retreat
                    and controller.index >= lamp_trigger_index):
                lamp_triggered = controller.return_on_lamp(elapsed)
            row = controller.step(d.qpos[:29].copy(),scene.imu,elapsed,elapsed)
            d.ctrl[:] = scene.q
            d.ctrl[kin.actuator] = row['q']+np.asarray(condition['tracking'])
            waist_tau = waist_support_torque(controller.kin,row['q'])
            for _ in range(round(PERIOD/m.opt.timestep)):
                d.qfrc_applied[:] = 0.
                d.qfrc_applied[kin.dadr] = row['tau']*condition.get('torque_effectiveness',1.)
                d.qfrc_applied[waist_dadr] = waist_tau
                mujoco.mj_step(m,d)
                mujoco.mj_forward(m,d)
                clearance = min(clearance,contact_metrics(kin,scene,contact=True))
                peak_press = max(peak_press,float(d.qpos[object_adr]))
                for ci, contact in enumerate(d.contact):
                    if {int(contact.geom1),int(contact.geom2)} == {kin.tip_id,target}:
                        force = np.zeros(6); mujoco.mj_contactForce(m,d,ci,force)
                        peak_force = max(peak_force,float(np.linalg.norm(force[:3])))
                if peak_press > scene.stroke+.001 or peak_force > 40.:
                    raise RuntimeError('Button overtravel or contact force limit')
            tracking_max = max(tracking_max,row['error_m'])
            dynamic_margin = min(dynamic_margin,float(np.minimum(
                d.qpos[kin.qadr]-limits[:,0],limits[:,1]-d.qpos[kin.qadr]).min()))
            elapsed += PERIOD
        reason = controller.error
    except (ValueError,RuntimeError) as exc:
        return dict(completed=False,reason=str(exc),wall_s=time.perf_counter()-started)
    home_error = float(np.max(np.abs(d.qpos[kin.qadr]-scene.q[22:29])))
    diagnostics = _trial_failure_reasons(scene,displacement=np.zeros(3),peak_press=peak_press,
        peak_force=peak_force,clearance=clearance,tracking_max=tracking_max,
        dynamic_joint_margin=dynamic_margin,home_error=home_error,condition=condition,include_diagnostics=True)
    failures = [v for v in diagnostics if settings.button_policy != 'collection'
                or v not in ('press_depth','tracking_error','home_return')]
    if lamp_trigger_index is not None:
        failures = [v for v in failures if v != 'press_depth']
        if not lamp_triggered:
            failures.append('lamp return not triggered')
        if home_error > np.deg2rad(2.):
            failures.append('lamp return did not reach home')
    if reason:
        failures.append(reason)
    return dict(completed=True,success=not failures,failure_reasons=failures,
        min_clearance_m=clearance,home_error_rad=home_error,tracking_max_m=tracking_max,
        min_joint_margin_rad=dynamic_margin,peak_press_m=peak_press,peak_force_n=peak_force,
        condition_position_m=list(condition['position']),condition_name=condition.get('name'),
        displacement_m=[0.,0.,0.],duration_s=elapsed,guard_holds=controller.guard_holds,
        button_feedback='joint',lamp_return_triggered=lamp_triggered,
        diagnostic_failure_reasons=diagnostics,wall_s=time.perf_counter()-started)
