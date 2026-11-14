#!/usr/bin/env python3
"""Read-only controller ownership, standing-state and calibration posture checks."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'Manipulation/push_button/real'), str(ROOT/'Manipulation/push_button')]
from check_arm_connection import check_state, validate_connection
from push_button import ArmSdk
from robust_policy import STRICT_POLICY, execution_policy


def validate_readiness_state(configuration, calibration, now, *, policy=STRICT_POLICY):
    """Share bottle execution limits while retaining explicit strict diagnostics."""
    check_state(configuration,now,policy=policy)
    q,imu,_,_,_ = configuration
    waist_delta = [a-b for a,b in zip(q[12:15],calibration['reference_waist_q_rad'])]
    tilt_delta = [math.atan2(math.sin(a-b),math.cos(a-b))
        for a,b in zip(imu[:2],calibration['reference_imu_rpy_rad'][:2])]
    warnings = []
    for changed,reason in ((max(map(abs,waist_delta)) > .025,'腰姿勢が校正時から0.025radを超えて変化しました'),
            (max(map(abs,tilt_delta)) > math.radians(2),'IMUの傾きが校正時から2度を超えて変化しました')):
        if changed:
            if not policy.record_deviations:
                raise RuntimeError(reason)
            warnings.append(reason)
    return warnings


def check_other_controller(host):
    command = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',host]
    script = ('python3 -c \'import urllib.request; '
        'print(urllib.request.urlopen("http://127.0.0.1:8766/api/status",timeout=2).read().decode())\'')
    result = subprocess.run(command+[script],capture_output=True,text=True,timeout=9)
    if result.returncode == 0:
        status = json.loads(result.stdout)
        if status.get('execute') is not False:
            raise RuntimeError('VLAの腕制御が有効です。現在の操作者が停止してから再実行してください')
        return {key:status.get(key) for key in ('execute','phase','weight','state_age_s')}
    probe = subprocess.run(command+[
        'ps -eo args | grep -F "python3 -m g1_e2e.vla.robot" | grep -v grep'],
        capture_output=True,text=True,timeout=9)
    if probe.returncode != 1:
        raise RuntimeError('他の腕制御プロセスの状態を確認できません: '+result.stderr[-500:])
    return dict(vla_process_absent=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial-config',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--ssh-host',default='g1')
    parser.add_argument('--policy',choices=('strict','collection'),default='strict')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=False)
    trial = json.loads(args.trial_config.read_text())
    calibration = json.loads(Path(trial['base_transform']).read_text())
    policy = execution_policy(args.policy=='collection')
    report = dict(robot_commands_sent=False,checked_at_epoch_s=time.time(),samples=[],warnings=[],
        readiness_policy=args.policy,execution_policy=policy.to_dict())
    arm = None
    try:
        report['other_controller'] = check_other_controller(args.ssh_host)
        arm = ArmSdk(trial['network_interface'])
        report['connection'] = arm.connection_info()
        validate_connection(report['connection'])
        for _ in range(50):
            config = arm.current_configuration()
            for warning in validate_readiness_state(config,calibration,time.monotonic(),policy=policy):
                if warning not in report['warnings']:
                    report['warnings'].append(warning)
            q,imu,stamp,leg,gyro = config
            report['samples'].append(dict(motor_q_rad=q,imu_rpy_rad=imu,
                state_age_s=time.monotonic()-stamp,leg_speed_rad_s=leg,gyro_rad_s=gyro))
            time.sleep(.02)
        report['passed'] = True
        (args.output_dir/'state.json').write_text(json.dumps(dict(schema='g1-button-state-v1',
            motor_q_rad=q,imu_rpy_rad=imu),indent=2)+'\n')
        print(f'FSM 501・29DoF・50状態サンプル・{args.policy}監視を確認しました。'
              f' 状態期限{policy.state_timeout_s*1000:g}ms、脚{policy.leg_speed_rad_s:g}rad/s、IMU{policy.gyro_rad_s:g}rad/s')
        return 0
    except Exception as exc:
        report.update(passed=False,error=str(exc))
        print(str(exc),file=sys.stderr)
        return 1
    finally:
        if arm is not None:
            arm.close()
        (args.output_dir/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
