"""Offline checks for bounded no-camera trace preparation and handover."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('mujoco')
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'real')]
import trace_arm


def test_trace_round_trip_starts_at_current_and_obeys_limits():
    rows=[dict(time_s=i*.1,q=[.1*np.sin(i*.05)]*7) for i in range(101)]
    current=np.zeros(29)
    path, metrics=trace_arm.make_trace(rows,current,1.5)
    np.testing.assert_array_equal(path[0],current[22:29])
    np.testing.assert_array_equal(path[-1],current[22:29])
    assert metrics['max_speed_rad_s'] <= .96
    assert metrics['max_acceleration_rad_s2'] <= 3.2
    assert metrics['reference_speedup'] <= 1.5
    with pytest.raises(ValueError):
        trace_arm.make_trace(rows,current,5)
    current[22]=1.
    with pytest.raises(ValueError,match='starting pose'):
        trace_arm.make_trace(rows,current,1.5)


def test_pose_change_at_handover_aborts_before_enabling(tmp_path,monkeypatch):
    import push_button, align_button, check_arm_connection, check_robust_readiness
    baseline=np.zeros(29)
    fake_state=(baseline,[0.,0.,0.],1.,0.,0.)
    calls=[]
    class Arm:
        def __init__(self,interface): pass
        def connection_info(self): return {}
        def current_configuration(self): return fake_state
        def stop_walking(self): calls.append('stop')
        def close(self): calls.append('close')
    class Motion:
        def __init__(self,*args,**kwargs): self.home=np.ones(7)*.001; self.release_error=None
        def enable(self): calls.append('enable')
        def close(self): calls.append('release')
    monkeypatch.setattr(trace_arm,'ROOT',tmp_path)
    monkeypatch.setattr(trace_arm.time,'sleep',lambda _:None)
    monkeypatch.setattr(push_button,'ArmSdk',Arm)
    monkeypatch.setattr(align_button,'ArmMotion',Motion)
    monkeypatch.setattr(align_button,'validate_stationary_pose',lambda *a,**k:None)
    monkeypatch.setattr(check_arm_connection,'validate_connection',lambda *a:None)
    monkeypatch.setattr(check_robust_readiness,'validate_readiness_state',lambda *a,**k:None)
    monkeypatch.setattr(check_robust_readiness,'check_other_controller',lambda *a:{})
    monkeypatch.setattr(trace_arm,'make_trace',lambda *a:(np.zeros((2,7)),{}))
    monkeypatch.setattr(trace_arm,'build_kin',lambda *a:SimpleNamespace(update_fixed_base=lambda *a:None))
    monkeypatch.setattr(trace_arm,'preflight',lambda *a:{})
    source=tmp_path/'source.json'; source.write_text(json.dumps(dict(calibration={},samples=[],source_error='old')))
    args=SimpleNamespace(trace=source,output=tmp_path/'out.json',ssh_host='g1',interface='eth0',model='unused',multiplier=1.5,execute=True)
    result=trace_arm.run(args)
    assert 'handover' in result['error']
    assert result['commands_sent']==0 and 'enable' not in calls
    assert calls[-2:] == ['release','close']


def test_stream_error_releases_control(tmp_path,monkeypatch):
    import push_button, align_button, check_arm_connection, check_robust_readiness
    import robust_planner, robust_control
    baseline=np.zeros(29)
    state=(baseline,[0.,0.,0.],1.,0.,0.)
    calls=[]
    class Arm:
        def __init__(self,interface): pass
        def connection_info(self): return {}
        def current_configuration(self): return state
        def stop_walking(self): pass
        def close(self): calls.append('close')
    class Motion:
        def __init__(self,*a,**k): self.home=np.zeros(7); self.release_error=None
        def enable(self): calls.append('enable')
        def start_approach(self): pass
        def current_target(self): return np.zeros(7)
        def stream(self,*a,**k): raise RuntimeError('simulated stream failure')
        def close(self): calls.append('release')
    kin=SimpleNamespace(data=SimpleNamespace(qpos=np.zeros(29)),update_fixed_base=lambda *a:None,
                        validate_path=lambda *a:None)
    monkeypatch.setattr(trace_arm,'ROOT',tmp_path)
    monkeypatch.setattr(trace_arm.time,'sleep',lambda _:None)
    monkeypatch.setattr(push_button,'ArmSdk',Arm)
    monkeypatch.setattr(align_button,'ArmMotion',Motion)
    monkeypatch.setattr(align_button,'validate_stationary_pose',lambda *a,**k:None)
    monkeypatch.setattr(check_arm_connection,'validate_connection',lambda *a:None)
    monkeypatch.setattr(check_robust_readiness,'validate_readiness_state',lambda *a,**k:None)
    monkeypatch.setattr(check_robust_readiness,'check_other_controller',lambda *a:{})
    monkeypatch.setattr(robust_planner,'reference_torque',lambda *a:np.zeros(7))
    monkeypatch.setattr(robust_control,'waist_support_torque',lambda *a:np.zeros(3))
    monkeypatch.setattr(trace_arm,'make_trace',lambda *a:(np.zeros((2,7)),{}))
    monkeypatch.setattr(trace_arm,'build_kin',lambda *a:kin)
    monkeypatch.setattr(trace_arm,'preflight',lambda *a:{})
    source=tmp_path/'source.json'; source.write_text(json.dumps(dict(calibration={},samples=[],source_error='old')))
    args=SimpleNamespace(trace=source,output=tmp_path/'out.json',ssh_host='g1',interface='eth0',model='unused',multiplier=1.5,execute=True)
    result=trace_arm.run(args)
    assert result['error']=='simulated stream failure'
    assert result['commands_sent']==0 and result['release_completed']
    assert calls==['enable','release','close']
