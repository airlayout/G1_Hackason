"""Single-command orchestration and non-actuating output checks; no robot IO."""
from contextlib import contextmanager
from copy import deepcopy
import json
from pathlib import Path
import socket
import sys
from types import SimpleNamespace

import pytest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]/'real'),str(Path(__file__).resolve().parents[1])]
import robust_session as session
from check_arm_output import matches_zero_weight
from upper_body_control import control_contract


def preview():
    return dict(planning_completed=True,plan=dict(accepted=True,completed_rollouts=3,
        scene=dict(face=[.375,-.22,.8],radius=.035),trajectory=dict(q=[[0.]*7])))


def test_venv_python_symlink_is_preserved(tmp_path):
    interpreter = tmp_path/'venv/bin/python'
    interpreter.parent.mkdir(parents=True)
    interpreter.symlink_to(sys.executable)
    assert session.local_path(interpreter,resolve_symlinks=False) == interpreter
    assert session.local_path(interpreter) == Path(sys.executable).resolve()


def test_failed_search_prints_ik_collisions_and_observed_position(tmp_path,capsys):
    (tmp_path/'result.json').write_text(json.dumps(dict(plan=dict(accepted=False,
        reason='No candidate completed and passed every required uncertainty condition',
        candidates_attempted=3,physically_evaluated_candidates=0,completed_rollouts=0,
        scene=dict(face=[.56,-.09,.87]),candidates=[
            dict(reason='手先のIKが収束しません (位置誤差 0.098 m)'),
            dict(reason='手先のIKが収束しません (位置誤差 0.075 m)'),
            dict(reason='腕の経路に衝突があります (geoms=[36, 61])')]))))
    session.print_execution_failure(tmp_path)
    output = capsys.readouterr().out
    assert '物理評価: 0 / 完走: 0' in output
    assert '(2件): 手先のIKが収束しません' in output
    assert 'geoms=[36, 61]' in output
    assert '[0.56, -0.09, 0.87]' in output
    assert '物理シミュレーション開始前' in output


def test_failed_rollouts_print_physical_failures_instead_of_generic_reason(tmp_path,capsys):
    (tmp_path/'preview.json').write_text(json.dumps(dict(plan=dict(accepted=False,
        physically_evaluated_candidates=1,completed_rollouts=1,candidates=[dict(
            reason='Not every condition completed and passed',trials=[
                dict(completed=True,failure_reasons=['press_depth','press_force']),
                dict(completed=False,reason='deadline')])]))))
    session.print_execution_failure(tmp_path)
    output = capsys.readouterr().out
    assert 'press_depth' in output and 'press_force' in output and 'deadline' in output
    assert '物理シミュレーション開始前' not in output


def test_output_probe_rejects_active_weight_torque_changed_target_and_bad_crc():
    message = SimpleNamespace(motor_cmd=[SimpleNamespace(q=0.,dq=0.,tau=0.,kp=0.,kd=0.) for _ in range(35)],crc=123)
    hold = dict(**control_contract(),held_upper_body_q_rad=[0.]*17)
    for index,kp,kd in zip(hold['joint_indices'],hold['kp'],hold['kd']):
        message.motor_cmd[index].kp=kp
        message.motor_cmd[index].kd=kd
    crc = SimpleNamespace(Crc=lambda value:123)
    assert matches_zero_weight(message,[0.]*7,crc,hold)
    for index,attribute,value in [(29,'q',.01),(22,'tau',.01),(15,'tau',.01),(22,'q',.1),
            (14,'kp',0.),(14,'tau',.01)]:
        changed = deepcopy(message)
        setattr(changed.motor_cmd[index],attribute,value)
        assert not matches_zero_weight(changed,[0.]*7,crc,hold)
    message.crc = 124
    assert not matches_zero_weight(message,[0.]*7,crc,hold)


def test_preview_requires_accepted_plan_and_current_measured_position():
    assert session.validate_preview(preview(),[.41,-.22],.015)['completed_conditions'] == 3
    changed = preview(); changed['plan']['accepted'] = False
    with pytest.raises(RuntimeError,match='採用'):
        session.validate_preview(changed,[.41,-.22],.015)
    with pytest.raises(RuntimeError,match='実測値'):
        session.validate_preview(preview(),[.41,.22],.015)


def test_socket_path_stays_short_even_with_long_log_path(tmp_path,monkeypatch):
    run = tmp_path/('x'*150); run.mkdir()
    class Process:
        def __init__(self,command,**kwargs):
            self.path = Path(command[command.index('--socket')+1])
            self.socket = socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
            self.socket.bind(str(self.path))
        def poll(self): return None
        def wait(self,timeout):
            self.socket.close()
            return 0
    monkeypatch.setattr(session.subprocess,'Popen',Process)
    with session.sdk_bridge('unused','lo',run,'test',{}) as path:
        assert len(str(path).encode()) < 108 and path.parent.parent == Path('/tmp')
        assert path.exists()
    assert not path.exists()


@pytest.mark.parametrize('mode,failure', [('check',None),('precontact',None),
    ('push',None),('precontact','state'),('push','state'),('check','preview'),('check','output')])
def test_one_command_orders_checks_and_never_arms_after_failed_preparation(tmp_path,monkeypatch,mode,failure):
    trial = tmp_path/'trial.json'; calibration = tmp_path/'calibration.json'
    model = tmp_path/'model.xml'; weights = tmp_path/'weights.pt'
    model.touch(); weights.touch()
    calibration.write_text(json.dumps(dict(position_uncertainty_m=.015)))
    trial.write_text(json.dumps(dict(model=str(model),weights=str(weights),base_transform=str(calibration),
        network_interface='lo',push_distance=.02)))
    setup = tmp_path/'setup.json'
    setup.write_text(json.dumps(dict(sdk_python=sys.executable,vision_python=sys.executable,
        trial_config=str(trial),ssh_host='g1',remote_camera_root='unused',measured_bottle_xy_m=[.41,-.22],
        remote_sdk_python='python3',remote_network_interface='eth0')))
    monkeypatch.setattr(session,'ROOT',tmp_path)
    real = tmp_path/'Manipulation/push_button/real'; real.mkdir(parents=True)
    monkeypatch.setattr(session,'REAL',real)
    monkeypatch.setattr(session,'ensure_camera',lambda *args:dict(already_listening=True))
    @contextmanager
    def bridge(*args): yield Path('/tmp/fake_arm.sock')
    monkeypatch.setattr(session,'sdk_bridge',bridge)
    calls = []
    def execute(command,log,env,*,input_text=None):
        calls.append((command,input_text))
        log.write_text('fake child log')
        if '--trial-config' in command:
            assert json.loads(Path(command[command.index('--trial-config')+1]).read_text())['push_distance']==.05
        if any('check_robust_readiness.py' in value for value in command):
            return 1 if failure == 'state' else 0
        if any('check_arm_output.py' in value for value in command):
            return 1 if failure == 'output' else 0
        if '--execute' not in command:
            if failure == 'preview': return 1
            Path(command[command.index('--output')+1]).write_text(json.dumps(preview()))
        else:
            Path(command[command.index('--output')+1]).write_text(json.dumps(dict(completed=True,
                execution=dict(diagnostic_completed=True,release_completed=True))))
        return 0
    monkeypatch.setattr(session,'execute_child',execute)
    result = session.main([mode,'--setup',str(setup),'--push-distance','.05'])
    assert json.loads(trial.read_text())['push_distance']==.02
    assert calls, 'Preparation must reach the subprocess phases before testing their failure paths'
    armed = [(command,text) for command,text in calls if '--execute' in command]
    if mode == 'check' or failure:
        assert not armed
    else:
        assert len(armed) == 1
        assert armed[0][1] is None
        assert '--execute-after-plan' in armed[0][0]
        assert ('--stop-before-contact' in armed[0][0]) == (mode=='precontact')
        assert ('--static-target' in armed[0][0]) == (mode=='precontact')
        assert ('--allow-stale-observation' in armed[0][0]) == (mode=='push')
    assert result == (1 if failure else 0)


def test_full_push_does_not_inherit_static_observation_mode():
    command = session.motion_arguments('push','trial','out',socket_path='sock')
    assert '--execute' in command
    assert '--static-target' not in command and '--stop-before-contact' not in command
    assert '--allow-stale-observation' in command
    assert '--execute-after-plan' in command and command[command.index('--planning-scenarios')+1]=='3'
