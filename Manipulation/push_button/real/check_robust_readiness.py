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
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=False)
    trial = json.loads(args.trial_config.read_text())
    calibration = json.loads(Path(trial['base_transform']).read_text())
    report = dict(robot_commands_sent=False,checked_at_epoch_s=time.time(),samples=[])
    arm = None
    try:
        report['other_controller'] = check_other_controller(args.ssh_host)
        arm = ArmSdk(trial['network_interface'])
        report['connection'] = arm.connection_info()
        validate_connection(report['connection'])
        for _ in range(50):
            config = arm.current_configuration()
            check_state(config,time.monotonic())
            q,imu,stamp,leg,gyro = config
            if not 0 <= time.monotonic()-stamp <= .1:
                raise RuntimeError('Robot state older than 100ms')
            if max(abs(a-b) for a,b in zip(q[12:15],calibration['reference_waist_q_rad'])) > .025:
                raise RuntimeError('腰姿勢が校正時から0.025radを超えて変化しました')
            tilt = [abs(math.atan2(math.sin(a-b),math.cos(a-b)))
                for a,b in zip(imu[:2],calibration['reference_imu_rpy_rad'][:2])]
            if max(tilt) > math.radians(2):
                raise RuntimeError('IMUの傾きが校正時から2度を超えて変化しました')
            report['samples'].append(dict(motor_q_rad=q,imu_rpy_rad=imu,
                state_age_s=time.monotonic()-stamp,leg_speed_rad_s=leg,gyro_rad_s=gyro))
            time.sleep(.02)
        report['passed'] = True
        (args.output_dir/'state.json').write_text(json.dumps(dict(schema='g1-button-state-v1',
            motor_q_rad=q,imu_rpy_rad=imu),indent=2)+'\n')
        print('FSM 501・29DoF・新しい50状態サンプル・校正姿勢を確認しました。')
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
