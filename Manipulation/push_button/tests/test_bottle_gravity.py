"""トルク指令の境界、解除、重力下の保持とIK失敗配置の回帰。"""

import os
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'real')]
import push_button
from trajectory import validate_arm_tau


def sdk_packet():
    arm = push_button.ArmSdk.__new__(push_button.ArmSdk)
    arm._cmd = SimpleNamespace(motor_cmd=[SimpleNamespace(q=0.) for _ in range(35)], crc=0)
    arm._crc = SimpleNamespace(Crc=lambda _: 0)
    packets = []
    def write(cmd):
        packets.append((cmd.motor_cmd[29].q, tuple(cmd.motor_cmd[i].tau for i in range(22, 29))))
    arm._publisher = SimpleNamespace(Write=write)
    arm._hold, arm._last_right, arm._last_weight = [0.]*14, None, 0.
    return arm, packets


@pytest.mark.parametrize('tau', [[0.]*6, [float('nan')]*7, [26.]*7, [0.]*6+[5.01]])
def test_invalid_torque_is_never_published(tau):
    arm, packets = sdk_packet()
    with pytest.raises(ValueError):
        arm.publish((0.,)*7, 1., require_fresh_state=False, right_tau=tau)
    assert not packets


def test_right_torque_and_release_ramp_leave_other_motors_untouched(monkeypatch):
    arm, packets = sdk_packet()
    tau = (1., -2., .5, -.5, .1, .2, -.2)
    arm.publish((0.,)*7, 1., require_fresh_state=False, right_tau=tau)
    assert packets[-1][1] == tau
    assert all(not hasattr(arm._cmd.motor_cmd[i], 'tau') for i in range(15))
    assert all(arm._cmd.motor_cmd[i].tau == 0 for i in range(15,22))
    monkeypatch.setattr(push_button, 'time', SimpleNamespace(sleep=lambda _: None))
    arm.release()
    assert packets[-1] == (0., (0.,)*7)
    assert all(a[0] >= b[0] for a,b in zip(packets, packets[1:]))
    assert all(abs(a[1][1]) >= abs(b[1][1]) for a,b in zip(packets, packets[1:]))
    # A following legacy call must overwrite every previous feedforward value.
    arm.publish((0.,)*7, 1., require_fresh_state=False)
    assert packets[-1][1] == (0.,)*7


def test_release_write_failure_is_not_reported_as_success(monkeypatch):
    arm, _ = sdk_packet()
    arm.publish((0.,)*7, 1., require_fresh_state=False, right_tau=(1.,)*7)
    arm._publisher.Write = lambda _: False
    monkeypatch.setattr(push_button, 'time', SimpleNamespace(sleep=lambda _: None))
    with pytest.raises(RuntimeError, match="制御解除"):
        arm.release()


def model_path():
    path = os.environ.get('G1_MUJOCO_MODEL')
    if not path:
        pytest.skip('G1_MUJOCO_MODELを指定してください')
    return Path(path)


def test_gravity_compensation_holds_arm_without_changing_pd():
    import mujoco
    from gravity import GravityCompensation
    from kinematics import load_robot, ArmKinematics
    calibration = {'tip_offset_m': [.11,-.003,0], 'tip_radius_m': .015, 'pelvis_height_m': .79}
    compensation = GravityCompensation(model_path(), calibration, 1.)
    errors = []
    for enabled in (False, True):
        model, rest = load_robot(model_path(), (.11,-.003,0), .79)
        kin, data = ArmKinematics(model), mujoco.MjData(model)
        data.qpos[:], data.ctrl[:] = rest, rest
        kp = np.array([40.,40.,40.,40.,15.,15.,15.])
        model.actuator_gainprm[kin.actuator,0] = kp
        model.actuator_biasprm[kin.actuator,1] = -kp
        model.actuator_biasprm[kin.actuator,2] = -1.5
        kin.update(rest); target = kin.tip_position()
        while data.time < 2.:
            if enabled:
                data.qfrc_applied[kin.dadr] = compensation((data.qpos.copy(), (0.,0.,0.)))
            mujoco.mj_step(model, data)
        kin.update(data.qpos.copy())
        errors.append(np.linalg.norm(kin.tip_position()-target))
    assert errors[0] > .005
    assert errors[1] < .002


def test_nominal_dynamics_feedforward_reduces_moving_joint_lag():
    import mujoco
    from gravity import GravityCompensation
    from kinematics import load_robot,ArmKinematics
    calibration={'tip_offset_m':[.11,-.003,0],'tip_radius_m':.015,'pelvis_height_m':.79}
    errors=[]
    for dynamics in (False,True):
        model,rest=load_robot(model_path(),(.11,-.003,0),.79)
        kin,data=ArmKinematics(model),mujoco.MjData(model)
        data.qpos[:],data.ctrl[:]=rest,rest
        kp=np.array([40.,40.,40.,40.,15.,15.,15.])
        model.actuator_gainprm[kin.actuator,0]=kp
        model.actuator_biasprm[kin.actuator,1]=-kp
        model.actuator_biasprm[kin.actuator,2]=-1.5
        feedforward=GravityCompensation(model_path(),calibration,1.,dynamics=dynamics)
        samples=[]
        while data.time<3.:
            target=rest[25]+.1*np.sin(np.pi*data.time)
            velocity=np.zeros(7);velocity[3]=.1*np.pi*np.cos(np.pi*data.time)
            data.ctrl[kin.actuator[3]]=target
            data.qfrc_applied[kin.dadr]=feedforward((data.qpos.copy(),(0.,0.,0.)),velocity=velocity)
            mujoco.mj_step(model,data)
            if data.time>1.:samples.append(data.qpos[kin.qadr[3]]-target)
        errors.append(float(np.sqrt(np.mean(np.array(samples)**2))))
    assert errors[1] < errors[0]*.4
    assert errors[1] < .005


def test_cartesian_joint_controller_tracks_without_exceeding_joint_rates():
    from alignment import SmoothNormalSettings
    from cartesian import CartesianJointController
    from kinematics import load_robot,ArmKinematics
    model,q=load_robot(model_path(),(.11,-.003,0),.79)
    kin=ArmKinematics(model);kin.update(q)
    target=kin.tip_position()+[.015,0.,.01]
    normal=kin.data.site_xmat[kin.site_id].reshape(3,3)[:,0].copy()
    controller=CartesianJointController(q[kin.qadr],SmoothNormalSettings())
    previous_velocity=np.zeros(7)
    for i in range(150):
        kin.update_fixed_base(q,(0.,0.,0.),base_offset=(.002*np.sin(i*.02*2*np.pi*.3),0.,0.))
        goal=controller.next_goal(kin,target,normal,.02)
        assert np.max(np.abs(controller.velocity)) <= .8+1e-8
        assert np.max(np.abs(controller.velocity-previous_velocity)) <= 2.*.02+1e-8
        previous_velocity=controller.velocity.copy()
        q[kin.qadr]=goal
    kin.update(q)
    assert np.linalg.norm(kin.tip_position()-target) < .003


@pytest.mark.parametrize('face', [(0.307245198,-0.276890231,.83), (.297601500,-.289377376,.83)])
def test_failed_preview_targets_pass_with_strict_ik_and_subdivision(face):
    from bottle import BottlePushRoutine, BottleSettings, BottleTarget, BottleObservation, add_table
    from kinematics import load_robot, ArmKinematics
    settings = BottleSettings(.75)
    model, rest = load_robot(model_path(), (.11,-.003,0), .79,
                             scene_builder=lambda root: add_table(root,settings))
    kin = ArmKinematics(model); kin.update(rest)
    target = BottleTarget((face[0]+.034,face[1]), .034, face, 1.,1.,(0,0,50,100))
    routine = BottlePushRoutine(kin,None,None,lambda:None,lambda:0.,settings,.015)
    report = routine.preview(BottleObservation(target,0.,1))
    assert 'push' in report['poses']
    assert max(row['position_error_m'] for row in routine.report['ik_checks']) < .0001
    assert max(row['axis_error_deg'] for row in routine.report['ik_checks']) < 10
