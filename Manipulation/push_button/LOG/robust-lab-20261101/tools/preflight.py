#!/usr/bin/env python3
"""Read robot state and compare with calibration. Never publish motion commands."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'Manipulation/push_button/real'),
                str(ROOT / 'Manipulation/push_button')]
from push_button import ArmSdk
from check_arm_connection import validate_connection, check_state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial-config', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--base-transform')
    parser.add_argument('--network-interface')
    args, _ = parser.parse_known_args()
    trial = json.loads(Path(args.trial_config).read_text())
    calibration = json.loads(Path(args.base_transform or trial['base_transform']).read_text())
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    report = dict(robot_commands_sent=False, checked_at_epoch_s=time.time(), samples=[])
    arm = None
    try:
        # The VLA process observed during setup exposes this read-only status URL.
        command = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'g1',
                   'python3 -c \'import urllib.request; '
                   'print(urllib.request.urlopen("http://127.0.0.1:8766/api/status",timeout=2).read().decode())\'']
        result = subprocess.run(command, capture_output=True, text=True, timeout=9)
        if result.returncode == 0:
            status = json.loads(result.stdout)
            report['vla'] = {k: status.get(k) for k in ('execute', 'phase', 'weight', 'state_age_s')}
            # In dry-run mode weight is simulated; the SDK publisher is not created.
            if status.get('execute') is not False:
                raise RuntimeError('VLA arm controller enabled; stop it before the bottle experiment')
        else:
            report['vla_status_unavailable'] = result.stderr[-1000:]
            probe = subprocess.run(
                ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'g1',
                 'ps -eo args | grep -F "python3 -m g1_e2e.vla.robot" | grep -v grep'],
                capture_output=True, text=True, timeout=9)
            if probe.returncode != 1:
                raise RuntimeError('VLA status unavailable while a controller may be running; recheck ownership')
            report['vla_process_absent'] = True
        arm = ArmSdk(args.network_interface or trial['network_interface'])
        arm.current_arm()
        report['connection'] = arm.connection_info()
        validate_connection(report['connection'])
        for _ in range(50):
            config = arm.current_configuration()
            check_state(config, time.monotonic())
            q, imu, stamp, leg, gyro = config
            if time.monotonic() - stamp > .1:
                raise RuntimeError('Robot state older than 100ms')
            if max(abs(a-b) for a,b in zip(q[12:15], calibration['reference_waist_q_rad'])) > .025:
                raise RuntimeError('Waist differs from the previous calibration by more than 0.025rad')
            delta = [abs(math.atan2(math.sin(a-b), math.cos(a-b)))
                     for a,b in zip(imu[:2], calibration['reference_imu_rpy_rad'][:2])]
            if max(delta) > math.radians(2):
                raise RuntimeError('IMU differs from the previous calibration by more than 2deg')
            report['samples'].append(dict(motor_q_rad=q, imu_rpy_rad=imu,
                                         state_age_s=time.monotonic()-stamp,
                                         leg_speed_rad_s=leg, gyro_rad_s=gyro))
            time.sleep(.02)
        report['state_checks_passed'] = True
        state = dict(schema='g1-button-state-v1', motor_q_rad=q, imu_rpy_rad=imu)
        (output/'state.json').write_text(json.dumps(state, indent=2)+'\n')
        print('Read-only state checks passed: 29DoF, FSM 501, 50 fresh samples, calibration posture within limits.')
        return 0
    except Exception as exc:
        report['error'] = str(exc)
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if arm is not None:
            arm.close()
        (output/'preflight.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
