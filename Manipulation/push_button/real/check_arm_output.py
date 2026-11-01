#!/usr/bin/env python3
"""Verify Unix RPC -> SDK -> rt/arm_sdk DDS using a zero-weight current-pose packet."""
import argparse
import json
import inspect
from pathlib import Path
import select
import shlex
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from arm_bridge import ArmProxy
from check_arm_connection import check_state, validate_connection
from motion_limits import JointMotionLimits
from trajectory import validate_arm_q
from upper_body_control import control_contract


def matches_zero_weight(message, expected, crc, hold):
    targets = hold['held_upper_body_q_rad'][:10]+list(expected)
    return (abs(float(message.motor_cmd[29].q)) < 1e-9
        and all(abs(float(message.motor_cmd[i].q)-q) < 1e-6
            and abs(float(message.motor_cmd[i].kp)-kp) < 1e-6
            and abs(float(message.motor_cmd[i].kd)-kd) < 1e-6
            and abs(float(message.motor_cmd[i].dq)) < 1e-9
            and abs(float(message.motor_cmd[i].tau)) < 1e-9
            for i,q,kp,kd in zip(hold['joint_indices'],targets,hold['kp'],hold['kd']))
        and int(message.crc) == crc.Crc(message))


def remote_listener(host,python,interface,expected,hold):
    """Read the output on G1 itself, independently of the sender's local loopback."""
    code = '''import json,sys,threading
from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC
'''+inspect.getsource(matches_zero_weight)+f'''
expected={expected!r}
hold={hold!r}
ChannelFactoryInitialize(0,{interface!r})
event=threading.Event()
crc=CRC()
result={{'received':False}}
def callback(message):
    if matches_zero_weight(message,expected,crc,hold):
        result.update(received=True,crc=int(message.crc),control_weight=float(message.motor_cmd[29].q),
            right_q_rad=[float(message.motor_cmd[i].q) for i in range(22,29)],
            upper_body_q_rad=[float(message.motor_cmd[i].q) for i in range(12,29)],
            upper_body_kp=[float(message.motor_cmd[i].kp) for i in range(12,29)],
            upper_body_kd=[float(message.motor_cmd[i].kd) for i in range(12,29)])
        event.set()
subscriber=ChannelSubscriber('rt/arm_sdk',LowCmd_)
subscriber.Init(callback,10)
print('READY',flush=True)
event.wait(6)
subscriber.Close()
print(json.dumps(result),flush=True)
sys.exit(0 if result['received'] else 1)
'''
    process = subprocess.Popen(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',host,
        shlex.join([python,'-u','-c',code])],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,text=True)
    deadline = time.monotonic()+10
    while time.monotonic()<deadline:
        ready,_,_ = select.select([process.stdout],[],[],.1)
        if ready and process.stdout.readline().strip() == 'READY':
            return process
        if process.poll() is not None:
            break
    process.terminate()
    _,error = process.communicate(timeout=3)
    raise RuntimeError('G1側のDDS受信確認を起動できません: '+error[-500:])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket',required=True)
    parser.add_argument('--network-interface',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--ssh-host')
    parser.add_argument('--remote-python')
    parser.add_argument('--remote-interface',default='eth0')
    args = parser.parse_args()
    report = dict(passed=False,control_weight=0.,motion_control_enabled=False,
        topic='rt/arm_sdk',packets_sent=0,loopback_received=False)
    proxy = subscriber = remote = None
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        ChannelFactoryInitialize(0,args.network_interface)
        received = threading.Event()
        expected = []
        hold = {}
        crc = CRC()
        def callback(message):
            if expected and hold and matches_zero_weight(message,expected,crc,hold):
                report['loopback_received'] = True
                report['received_right_q_rad'] = [float(message.motor_cmd[i].q) for i in range(22,29)]
                report['received_crc'] = int(message.crc)
                received.set()
        subscriber = ChannelSubscriber('rt/arm_sdk',LowCmd_)
        subscriber.Init(callback,10)
        proxy = ArmProxy(args.socket)
        report['connection'] = proxy.connection_info()
        validate_connection(report['connection'])
        report['limits'] = proxy.configure_motion_limits(JointMotionLimits(1.2,4.))
        config = proxy.current_configuration()
        check_state(config,time.monotonic())
        current = tuple(config[0][15:29])
        plan = json.loads(args.plan.read_text())
        if not plan.get('plan',{}).get('accepted'):
            raise RuntimeError('採用された軌道がありません')
        expected[:] = validate_arm_q(plan['plan']['trajectory']['q'][0])
        if max(abs(a-b) for a,b in zip(expected,current[7:])) > .025:
            raise RuntimeError('方策の初回指令と現在の腕姿勢が一致しません')
        hold.update(proxy.set_hold(current))
        for key,value in control_contract().items():
            if hold.get(key) != value:
                raise RuntimeError('SDK側の上半身保持設定がシミュレーションと一致しません')
        report['upper_body_hold'] = hold
        if args.ssh_host:
            if not args.remote_python:
                raise RuntimeError('--remote-python is required for G1 DDS readback')
            remote = remote_listener(args.ssh_host,args.remote_python,args.remote_interface,list(expected),hold)
        # Only the initial policy sample is transmitted, with control weight
        # zero. DDS discovery occurs before the single non-actuating packet.
        time.sleep(1.)
        proxy.publish(tuple(expected),0.,right_tau=(0.,)*7,waist_tau=(0.,)*3)
        report['packets_sent'] = 1
        if not received.wait(3.):
            raise RuntimeError('重み0のSDK出力をDDSで受信できませんでした')
        if remote is not None:
            stdout,stderr = remote.communicate(timeout=7)
            report['robot_readback'] = json.loads(stdout.strip().splitlines()[-1])
            if remote.returncode or not report['robot_readback'].get('received'):
                raise RuntimeError('G1側で重み0の方策出力を受信できません: '+stderr[-300:])
        report.update(passed=True,command_right_q_rad=list(expected),crc_verified=True,
                      policy_report=str(args.plan),policy_sample_index=0)
        print('方策の初回指令→Unix RPC→SDK→rt/arm_sdkを確認しました（トルク0・制御重み0）。')
        if remote is not None:
            print('G1上で腰を含む17軸の目標角・保持ゲイン・重み0・CRC一致も確認しました。')
        return 0
    except Exception as exc:
        report['error'] = str(exc)
        print(str(exc),file=sys.stderr)
        return 1
    finally:
        if proxy is not None:
            proxy.close()
        if subscriber is not None:
            subscriber.Close()
        if remote is not None and remote.poll() is None:
            remote.terminate()
            remote.communicate(timeout=3)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
