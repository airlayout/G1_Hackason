"""Shared bottle/button startup limits, without SDK or robot IO."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'real'),str(ROOT)]
import check_robust_readiness as readiness
from robust_policy import COLLECTION_POLICY,STRICT_POLICY


CAL=dict(reference_waist_q_rad=[0.,0.,0.],reference_imu_rpy_rad=[0.,0.,0.])


def configuration(*,age=0.,leg=0.,gyro=0.,waist=0.,tilt=0.):
    q=[0.]*29;q[12]=waist
    return (q,(0.,tilt,0.),-age,leg,gyro)


def test_collection_readiness_uses_bottle_execution_limits_and_records_pose_drift():
    value=configuration(age=.3,leg=.4,gyro=.2,waist=.04,tilt=.06)
    warnings=readiness.validate_readiness_state(value,CAL,0.,policy=COLLECTION_POLICY)
    assert len(warnings)==2 and '腰' in warnings[0] and 'IMU' in warnings[1]
    with pytest.raises(RuntimeError):
        readiness.validate_readiness_state(value,CAL,0.,policy=STRICT_POLICY)


@pytest.mark.parametrize('value',[
    dict(age=.4001),dict(age=-.01),dict(leg=.5001),dict(gyro=.3001),dict(tilt=.1501)])
def test_collection_still_rejects_expired_moving_or_tilted_state(value):
    with pytest.raises(RuntimeError):
        readiness.validate_readiness_state(configuration(**value),CAL,0.,policy=COLLECTION_POLICY)


def test_collection_boundaries_and_malformed_joint_values():
    assert readiness.validate_readiness_state(configuration(age=.4,leg=.5,gyro=.3),CAL,0.,policy=COLLECTION_POLICY)==[]
    value=configuration();value[0][23]=math.nan
    with pytest.raises(ValueError,match='状態データ'):
        readiness.validate_readiness_state(value,CAL,0.,policy=COLLECTION_POLICY)


@pytest.mark.parametrize('value',[dict(age=.1001),dict(waist=.0251),dict(tilt=math.radians(2.01))])
def test_explicit_strict_readiness_keeps_previous_limits(value):
    with pytest.raises(RuntimeError):
        readiness.validate_readiness_state(configuration(**value),CAL,0.)


@pytest.mark.parametrize('policy,expected',[('collection',0),('strict',1)])
def test_readiness_entrypoint_passes_policy_and_closes_read_only_sdk(tmp_path,monkeypatch,policy,expected):
    cal=tmp_path/'cal.json';cal.write_text(json.dumps(CAL))
    trial=tmp_path/'trial.json';trial.write_text(json.dumps(dict(base_transform=str(cal),network_interface='unused')))
    output=tmp_path/'result'
    closed=[]
    class Arm:
        def __init__(self,*a): pass
        def connection_info(self): return dict(mode_machine=5,motor_count=35,fsm_id=501)
        def current_configuration(self): return configuration(age=.3,leg=.4,gyro=.2,waist=.04,tilt=.06)
        def close(self): closed.append(True)
    monkeypatch.setattr(readiness,'ArmSdk',Arm)
    monkeypatch.setattr(readiness,'check_other_controller',lambda host:dict(vla_process_absent=True))
    monkeypatch.setattr(readiness.time,'monotonic',lambda:0.)
    monkeypatch.setattr(readiness.time,'sleep',lambda _:None)
    monkeypatch.setattr(sys,'argv',['check','--trial-config',str(trial),'--output-dir',str(output),'--policy',policy])
    assert readiness.main()==expected
    report=json.loads((output/'result.json').read_text())
    assert report['readiness_policy']==policy and report['robot_commands_sent'] is False and closed
    if expected==0:
        assert len(report['samples'])==50 and len(report['warnings'])==2
        assert report['execution_policy']==COLLECTION_POLICY.to_dict()
    else:
        assert not (output/'state.json').exists()


@pytest.mark.parametrize('script',['run_robust_button.sh','run_robust_bottle.sh'])
def test_both_bash_entrypoints_select_shared_collection_readiness(tmp_path,script):
    python=tmp_path/'python3'
    python.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n');python.chmod(0o755)
    result=subprocess.run(['bash',str(ROOT/'real'/script),'check'],check=True,text=True,capture_output=True,
        env=dict(os.environ,PATH=str(tmp_path)+os.pathsep+os.environ['PATH']))
    args=result.stdout.splitlines()
    assert args[args.index('--readiness-policy')+1]=='collection'
