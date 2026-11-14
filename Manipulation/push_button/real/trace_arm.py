#!/usr/bin/env python3
"""Prepare and optionally execute a no-camera trace of a recorded arm approach."""
import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import sys
import time

import mujoco
import numpy as np
from scipy.interpolate import UnivariateSpline

REAL = Path(__file__).resolve().parent
ROOT = REAL.parents[2]
sys.path.insert(0, str(REAL.parent))
from kinematics import ArmKinematics, load_robot
from trajectory import ARM_TORQUE_LIMITS_NM
from upper_body_control import UPPER_BODY_JOINTS, UPPER_BODY_KP, UPPER_BODY_KD
from motion_limits import JointMotionLimits

PERIOD = .02


def smooth(u):
    return u**3 * (10 - 15*u + 6*u*u)


def make_trace(rows, current, multiplier=1.5):
    if not math.isfinite(multiplier) or not 0.1 <= multiplier <= 1.5:
        raise ValueError('trace multiplier must be between 0.1 and 1.5')
    t = np.asarray([r['time_s'] for r in rows], dtype=float)
    q = np.asarray([r['q'] for r in rows], dtype=float)
    current = np.asarray(current, dtype=float)
    if (len(t) < 5 or q.shape != (len(t), 7) or current.shape != (29,)
            or not np.all(np.isfinite(t)) or not np.all(np.diff(t) > 0)
            or not np.all(np.isfinite(q)) or not np.all(np.isfinite(current))):
        raise ValueError('Invalid recorded approach or current pose')
    t -= t[0]
    splines = [UnivariateSpline(t, q[:, i], s=len(t)*.003**2) for i in range(7)]
    fit = np.column_stack([s(t) for s in splines])
    fit_error = float(np.max(abs(q-fit)))
    if fit_error > .02:
        raise ValueError('Recorded motion cannot be smoothed within 0.02 rad')
    if np.max(abs(current[22:29]-fit[0])) > .15:
        raise ValueError('Recorded starting pose differs from current right arm by more than 0.15 rad')
    for duration in np.arange(max(.5,t[-1]/multiplier), t[-1]*10+.02, .02):
        n = max(2, math.ceil(duration/PERIOD))
        x = smooth(np.linspace(0.,1.,n+1))*t[-1]
        path = np.column_stack([s(x) for s in splines])
        blend = smooth(np.linspace(0.,1.,101))[:,None]
        lead = current[22:29]+blend*(path[0]-current[22:29])
        trajectory = np.vstack([lead,path[1:],path[-2::-1],lead[-2::-1]])
        velocity = np.diff(trajectory,axis=0)/PERIOD
        acceleration = np.diff(velocity,axis=0)/PERIOD
        if np.max(abs(velocity)) <= .96 and np.max(abs(acceleration)) <= 3.2:
            return trajectory, dict(reference_speedup=float(t[-1]/(n*PERIOD)),
                smoothing_max_error_rad=fit_error, max_speed_rad_s=float(np.max(abs(velocity))),
                max_acceleration_rad_s2=float(np.max(abs(acceleration))))
    raise ValueError('Unable to satisfy joint speed and acceleration limits')


def build_kin(model_path, calibration, q, imu):
    model, _ = load_robot(Path(model_path), tuple(calibration['tip_offset_m']),
                          calibration['pelvis_height_m'], calibration['tip_radius_m'])
    model.opt.timestep = .002
    kin = ArmKinematics(model)
    kin.update_fixed_base(np.asarray(q),tuple(imu))
    return kin


def preflight(kin, path, baseline):
    for target in path:
        kin.validate_path(target)
        pose = kin.data.qpos.copy(); pose[kin.qadr] = target; kin.update(pose)
    m,d = kin.model,kin.data
    actuators = np.array([m.actuator(name).id for name in UPPER_BODY_JOINTS])
    m.actuator_gainprm[actuators,0] = UPPER_BODY_KP
    m.actuator_biasprm[actuators,1] = -np.asarray(UPPER_BODY_KP)
    m.actuator_biasprm[actuators,2] = -np.asarray(UPPER_BODY_KD)
    d.qpos[:29] = baseline
    d.qvel[:] = 0.
    mujoco.mj_forward(m,d)
    peak = 0.
    for i,target in enumerate(path):
        d.ctrl[:] = baseline
        d.ctrl[kin.actuator] = target
        for _ in range(10):
            d.qfrc_applied[:] = 0.
            d.qfrc_applied[kin.dadr] = np.clip(d.qfrc_bias[kin.dadr],-np.asarray(ARM_TORQUE_LIMITS_NM),ARM_TORQUE_LIMITS_NM)
            mujoco.mj_step(m,d); mujoco.mj_forward(m,d)
            for contact in d.contact:
                if contact.dist < -.0005 and {int(contact.geom1),int(contact.geom2)} & kin.arm_geoms:
                    raise RuntimeError(f'Simulated arm collision at sample {i}')
        peak = max(peak,float(np.max(abs(d.qpos[kin.qadr]-target))))
        if peak > .10:
            raise RuntimeError(f'Simulated tracking error exceeds 0.10 rad at sample {i}: {peak:.3f}')
    return dict(simulated_max_tracking_rad=peak,simulated_home_error_rad=float(np.max(abs(d.qpos[kin.qadr]-baseline[22:29]))))


def run(args):
    from push_button import ArmSdk
    from align_button import ArmMotion, validate_stationary_pose
    from check_arm_connection import validate_connection
    from check_robust_readiness import check_other_controller, validate_readiness_state
    from robust_policy import COLLECTION_POLICY
    from robust_planner import reference_torque
    from robust_control import waist_support_torque
    raw = json.loads(Path(args.trace).read_text())
    calibration = raw['calibration']
    output = Path(args.output)
    output.parent.mkdir(parents=True,exist_ok=True)
    report = dict(completed=False,release_completed=False,commands_sent=0,source_error=raw['source_error'])
    arm = motion = None
    lockpath = ROOT/'_local/robust_push/runs/robot.lock'
    lockpath.parent.mkdir(parents=True,exist_ok=True)
    with lockpath.open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            report['other_controller'] = check_other_controller(args.ssh_host)
            arm = ArmSdk(args.interface)
            validate_connection(arm.connection_info())
            samples=[]
            for _ in range(50):
                state=arm.current_configuration()
                validate_readiness_state(state,calibration,time.monotonic(),policy=COLLECTION_POLICY)
                samples.append(np.asarray(state[0])); time.sleep(PERIOD)
            if np.max(np.ptp(np.asarray(samples)[:,12:29],axis=0)) > .02:
                raise RuntimeError('Upper body moved while checking readiness')
            state=arm.current_configuration(); baseline=np.asarray(state[0]); imu=tuple(state[1])
            path, metrics=make_trace(raw['samples'],baseline,args.multiplier)
            kin=build_kin(args.model,calibration,baseline,imu)
            report.update(metrics); print('静的な経路と固定骨盤シミュレーションを検証します。',flush=True)
            report.update(preflight(kin,path,baseline))
            print(json.dumps(report,ensure_ascii=False),flush=True)
            if not args.execute:
                report['preflight_passed']=True
                return report
            state=arm.current_configuration()
            validate_readiness_state(state,calibration,time.monotonic(),policy=COLLECTION_POLICY)
            if (np.max(abs(np.asarray(state[0])[12:29]-baseline[12:29])) > .02
                    or np.max(abs(np.asarray(state[1])[:2]-np.asarray(imu)[:2])) > .02):
                raise RuntimeError('Robot pose changed while the trajectory was verified')
            kin.update_fixed_base(np.asarray(state[0]),state[1])
            arm.stop_walking()
            state=arm.current_configuration()
            if np.max(abs(np.asarray(state[0])[12:29]-baseline[12:29])) > .02:
                raise RuntimeError('Robot pose changed before enabling arm control')
            torque={'right':np.zeros(7),'waist':np.zeros(3)}
            motion=ArmMotion(arm,calibration,gravity=lambda config,velocity=None: torque['right'],
                waist_gravity=lambda config: torque['waist'],limits=JointMotionLimits(1.2,4.),
                allow_pose_drift=True,synchronous_stream=True)
            # A tiny handover difference otherwise appears as an instantaneous
            # first-tick velocity and acceleration. Abort before enabling.
            if np.max(abs(np.asarray(motion.home)-path[0])) > .0001:
                raise RuntimeError('Arm moved before control handover')
            torque['right']=reference_torque(kin,path[0],np.zeros(7),1.)
            torque['waist']=waist_support_torque(kin,path[0])
            print('検証済みの腕の往復を開始します。Ctrl+Cで中断できます。',flush=True)
            motion.enable(); motion.start_approach()
            prev=motion.current_target()
            last_cycle=time.monotonic()
            for i,target in enumerate(path):
                cycle=time.monotonic()
                if cycle-last_cycle > .4:
                    raise RuntimeError('Arm control update delayed by over 0.4 seconds')
                state=arm.current_configuration()
                validate_stationary_pose(state,calibration,allow_pose_drift=True)
                measured=np.asarray(state[0])
                if np.max(abs(measured[22:29]-prev)) > .10:
                    raise RuntimeError('Measured arm tracking error exceeded 0.10 rad')
                pose=kin.data.qpos.copy(); pose[:29]=measured; kin.update_fixed_base(pose,state[1])
                kin.validate_path(target)
                velocity=(target-prev)/PERIOD
                torque['right']=reference_torque(kin,target,velocity,1.)
                torque['waist']=waist_support_torque(kin,target)
                motion.stream(target,velocity=velocity)
                report['commands_sent'] += 1
                prev=target
                motion.wait(max(0., PERIOD-(time.monotonic()-cycle)))
                last_cycle=cycle
            motion.finish_stream(); motion.wait(.5)
            final=arm.current_configuration()
            report['home_max_error_rad']=float(np.max(abs(np.asarray(final[0])[22:29]-baseline[22:29])))
            report['completed']=report['home_max_error_rad'] <= math.radians(2)
            if not report['completed']:
                report['error']='Return error exceeds 2 degrees'
        except BaseException as exc:
            report['error']=str(exc) or type(exc).__name__
            report['interrupted']=isinstance(exc,KeyboardInterrupt)
        finally:
            if motion is not None:
                motion.close()
                report['release_completed']=motion.release_error is None
                if motion.release_error:
                    report['release_error']=str(motion.release_error)
                    report['completed']=False
            if arm is not None:
                arm.close()
            output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',required=True)
    parser.add_argument('--model',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--interface',default='eth0')
    parser.add_argument('--ssh-host',default='g1')
    parser.add_argument('--multiplier',type=float,default=1.5)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    result=run(args)
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return 0 if result.get('completed') or result.get('preflight_passed') else 1

if __name__=='__main__':
    sys.exit(main())
