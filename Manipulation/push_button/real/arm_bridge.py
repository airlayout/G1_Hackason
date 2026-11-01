#!/usr/bin/env python3
"""SDKのPython環境とRGB-D/IK環境を、ローカルUnixソケットで接続する。"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import sys
import threading
import time
from pathlib import Path

from push_button import ArmSdk
from trajectory import validate_arm_q, validate_arm_tau
from motion_limits import JointMotionLimits
from upper_body_control import validate_waist_tau


class ArmProxy:
    """ArmMotion互換。状態取得と制御指令を別ソケットで送受信する。"""

    def __init__(self, path, *, separate_state=True):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(3.0)
        self.socket.connect(path)
        self.buffer = b""
        self.lock = threading.Lock()
        self.weight = 0.0
        self._state_channel = None
        if separate_state and Path(str(path)+'.state').exists():
            self._state_channel = ArmProxy(str(path)+'.state',separate_state=False)

    def _rpc(self, op, **args):
        with self.lock:
            self.socket.sendall((json.dumps(dict(op=op, **args), allow_nan=False)+"\n").encode())
            while b"\n" not in self.buffer:
                chunk = self.socket.recv(4096)
                if not chunk:
                    raise RuntimeError("腕SDKブリッジとの接続が途絶しました")
                self.buffer += chunk
                if len(self.buffer) > 16384:
                    raise RuntimeError("腕SDKブリッジの応答が大きすぎます")
            line, self.buffer = self.buffer.split(b"\n", 1)
            reply = json.loads(line)
            if "error" in reply:
                raise RuntimeError(reply["error"])
            return reply.get("result")

    def current_configuration(self):
        channel = getattr(self,'_state_channel',None) or self
        values = channel._rpc("state")
        return tuple(values[0]), tuple(values[1]), *values[2:]

    def current_arm(self):
        return tuple(self.current_configuration()[0][15:29])

    def connection_info(self):
        return self._rpc("info")

    def stop_walking(self):
        self._rpc("stop")

    def set_hold(self, current):
        return self._rpc("hold", joints=list(current))

    def configure_motion_limits(self, limits):
        result = self._rpc('configure_limits', limits=limits.to_dict())
        if result != limits.to_dict():
            raise RuntimeError('腕ブリッジが関節制限の設定に未対応です。SDK側も更新してください')
        return result

    def publish(self, right, weight, require_fresh_state=True, *, right_tau=None, waist_tau=None):
        kwargs = {} if right_tau is None else {"right_tau": list(validate_arm_tau(right_tau))}
        if waist_tau is not None:
            kwargs['waist_tau'] = list(validate_waist_tau(waist_tau))
        self.weight = weight
        result = self._rpc("publish", joints=list(right), weight=weight, **kwargs)
        if right_tau is not None and (not isinstance(result,dict) or result.get('torque_supported') is not True):
            self.release()
            raise RuntimeError("腕ブリッジがトルク送信に未対応です。SDK側も更新してください")
        if waist_tau is not None and (not isinstance(result,dict) or result.get('waist_torque_supported') is not True):
            self.release()
            raise RuntimeError('腕ブリッジが腰の補助トルク送信に未対応です')

    def release(self):
        if self.weight > 0:
            self._rpc("release")
            self.weight = 0.0

    def close(self):
        try:
            self.release()
        finally:
            self.socket.close()
            state = getattr(self,'_state_channel',None)
            if state is not None:
                state.close()


def serve_state_connections(server, arm, stop):
    """Read-only state RPC never waits behind a publish or a weight release."""
    server.settimeout(.1)
    while not stop.is_set():
        try:
            connection,_ = server.accept()
        except socket.timeout:
            continue
        except OSError:
            return
        with connection:
            connection.settimeout(.1)
            buffer = b''
            while not stop.is_set():
                try:
                    chunk = connection.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buffer += chunk
                if len(buffer)>16384:
                    break
                while b'\n' in buffer:
                    line,buffer = buffer.split(b'\n',1)
                    try:
                        if json.loads(line)['op'] != 'state':
                            raise ValueError('This socket is read-only')
                        reply = dict(result=arm.current_configuration())
                    except Exception as error:
                        reply = dict(error=str(error))
                    try:
                        connection.sendall((json.dumps(reply)+'\n').encode())
                    except OSError:
                        break


def serve_connection(connection, arm, timeout_s=0.5, *, status=None):
    """送信指令が途絶したらSDK側で解除する。推論プロセスの生存に依存しない。"""
    connection.settimeout(0.05)
    buffer = b""
    last_publish = None
    last_target = None
    active = False
    limits = JointMotionLimits()
    status = {} if status is None else status
    fault = None
    try:
        while True:
            try:
                chunk = connection.recv(4096)
                if not chunk:
                    return
                buffer += chunk
                if len(buffer) > 16384:
                    raise ValueError("腕指令のサイズが上限を超えました")
            except socket.timeout:
                pass
            if active and time.monotonic()-last_publish > timeout_s:
                fault = '腕指令が途絶しました。制御重みを解除します'
                status['error'] = fault
                print(fault,file=sys.stderr,flush=True)
                arm.release()
                status['release_completed'] = True
                active = False
                last_target = last_publish = None
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                try:
                    request = json.loads(line)
                    op = request["op"]
                    result = None
                    if op == "state":
                        result = arm.current_configuration()
                    elif op == "info":
                        if active:
                            raise ValueError("腕制御中は接続情報を照会できません")
                        result = arm.connection_info()
                    elif op == "stop":
                        if active:
                            raise ValueError("腕制御中は停止要求を再実行できません")
                        arm.stop_walking()
                    elif op == "hold":
                        if active:
                            raise ValueError("腕制御中に保持姿勢を変更できません")
                        current = arm.current_arm()
                        joints = request["joints"]
                        if (len(joints) != 14 or not all(math.isfinite(v) for v in joints)
                                or max(abs(a-b) for a, b in zip(joints, current)) > 0.025):
                            raise ValueError("保持姿勢が実機の現在角と一致しません")
                        result = arm.set_hold(current)
                        status['upper_body_hold'] = result
                    elif op == 'configure_limits':
                        if active:
                            raise ValueError('腕制御中は関節制限を変更できません')
                        limits = JointMotionLimits(**request['limits'])
                        result = limits.to_dict()
                    elif op == "publish":
                        joints = validate_arm_q(request["joints"])
                        weight = float(request["weight"])
                        if not 0 <= weight <= 1:
                            raise ValueError("腕の制御重みが不正です")
                        if fault is not None and weight>0:
                            raise RuntimeError('SDK bridge fault: '+fault)
                        now = time.monotonic()
                        if last_target is None and max(abs(a-b) for a, b in zip(joints, arm.current_arm()[7:])) > 0.025:
                            raise ValueError("初回の腕指令が現在姿勢と一致しません")
                        if last_target is not None:
                            allowed = limits.joint_speed_rad_s * (now-last_publish) + 0.005
                            if max(abs(a-b) for a, b in zip(joints, last_target)) > allowed:
                                raise ValueError("腕指令の変化速度が上限を超えました")
                        kwargs = {}
                        if 'right_tau' in request:
                            kwargs['right_tau'] = validate_arm_tau(request['right_tau'])
                        if 'waist_tau' in request:
                            kwargs['waist_tau'] = validate_waist_tau(request['waist_tau'])
                        arm.publish(joints,weight,**kwargs)
                        if kwargs:
                            result = dict(torque_supported=True,waist_torque_supported=True)
                        last_publish, last_target = now, joints
                        active = weight > 0
                        status.update(control_ever_enabled=status.get('control_ever_enabled',False) or active,
                            last_weight=weight,release_completed=not active)
                    elif op == "release":
                        arm.release()
                        active = False
                        last_target = last_publish = None
                        limits = JointMotionLimits()
                        status.update(release_completed=True,last_weight=0.)
                        result = dict(released=True)
                    else:
                        raise ValueError("未対応の腕指令です")
                    reply = {"result": result}
                except Exception as error:
                    fault = str(error)
                    status['error'] = fault
                    print('SDK bridge request failed: '+fault,file=sys.stderr,flush=True)
                    # Tell the client the actual reason before the two-second
                    # release. Keep the connection for the release acknowledgement.
                    connection.sendall((json.dumps(dict(error=fault))+'\n').encode())
                    arm.release()
                    active = False
                    last_target = last_publish = None
                    status.update(release_completed=True,last_weight=0.)
                    continue
                connection.sendall((json.dumps(reply, allow_nan=False)+"\n").encode())
    finally:
        arm.release()
        status.update(release_completed=True,last_weight=0.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default="/tmp/g1_button_arm.sock")
    parser.add_argument("--network-interface", default="enp3s0")
    parser.add_argument("--arm", action="store_true", help="実機SDKへの接続を有効にする")
    parser.add_argument("--snapshot-out", help="関節角とIMUだけを読み、JSONを保存して終了する")
    parser.add_argument('--status-out',help='SDK側の解除結果を保存するJSON')
    args = parser.parse_args()
    if args.snapshot_out:
        arm = ArmSdk(args.network_interface)
        joints, rpy, _, _, _ = arm.current_configuration()
        output = Path(args.snapshot_out)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({"schema": "g1-button-state-v1", "motor_q_rad": joints,
                                      "imu_rpy_rad": rpy}, indent=2)+"\n")
        print(f"状態のみ保存: {output}。腕・歩行の指令は送っていません。")
        return
    if not args.arm:
        parser.error("SDK側の接続には --arm を明示してください")
    path = Path(args.socket)
    if path.exists():
        parser.error("ソケットが既に存在します。既存の制御プロセスを確認してください")
    arm = ArmSdk(args.network_interface)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    state_server = socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    state_path = Path(str(path)+'.state')
    stop = threading.Event()
    state_thread = None
    status = dict(control_ever_enabled=False,last_weight=0.,release_completed=True)
    def interrupt(_signal,_frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    try:
        state_server.bind(str(state_path))
        os.chmod(state_path,0o600)
        state_server.listen(1)
        state_thread = threading.Thread(target=serve_state_connections,args=(state_server,arm,stop),daemon=True)
        state_thread.start()
        server.bind(str(path))
        os.chmod(path, 0o600)
        server.listen(1)
        print(f"腕SDKブリッジ待機: {path}", flush=True)
        connection, _ = server.accept()
        with connection:
            serve_connection(connection, arm,status=status)
    except KeyboardInterrupt:
        status['interrupted'] = True
    except Exception as error:
        status['error'] = str(error)
        raise
    finally:
        try:
            arm.close()
            status.update(last_weight=float(arm._last_weight),release_completed=arm._last_weight<=0)
        except Exception as error:
            status.update(release_completed=False,release_error=str(error))
        stop.set()
        state_server.close()
        if state_thread is not None:
            state_thread.join(timeout=1.)
        server.close()
        if path.exists():
            path.unlink()
        if state_path.exists():
            state_path.unlink()
        if args.status_out:
            output = Path(args.status_out)
            output.parent.mkdir(parents=True,exist_ok=True)
            output.write_text(json.dumps(status,ensure_ascii=False,indent=2)+'\n')


if __name__ == "__main__":
    main()
