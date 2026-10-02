#!/usr/bin/env python3
"""SDKのPython環境とRGB-D/IK環境を、ローカルUnixソケットで接続する。"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import threading
import time
from pathlib import Path

from push_button import ArmSdk
from trajectory import validate_arm_q


class ArmProxy:
    """ArmMotionと同じインターフェース。カメラ・送信スレッド間でRPCを直列化する。"""

    def __init__(self, path):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(3.0)
        self.socket.connect(path)
        self.buffer = b""
        self.lock = threading.Lock()
        self.weight = 0.0

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
        values = self._rpc("state")
        return tuple(values[0]), tuple(values[1]), *values[2:]

    def current_arm(self):
        return tuple(self.current_configuration()[0][15:29])

    def stop_walking(self):
        self._rpc("stop")

    def set_hold(self, current):
        self._rpc("hold", joints=list(current))

    def publish(self, right, weight, require_fresh_state=True):
        self._rpc("publish", joints=list(right), weight=weight)
        self.weight = weight

    def release(self):
        if self.weight > 0:
            self._rpc("release")
            self.weight = 0.0

    def close(self):
        try:
            self.release()
        finally:
            self.socket.close()


def serve_connection(connection, arm, timeout_s=0.5):
    """送信指令が途絶したらSDK側で解除する。推論プロセスの生存に依存しない。"""
    connection.settimeout(0.05)
    buffer = b""
    last_publish = None
    last_target = None
    active = False
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
                raise RuntimeError("腕指令が途絶しました。制御重みを解除します")
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                try:
                    request = json.loads(line)
                    op = request["op"]
                    result = None
                    if op == "state":
                        result = arm.current_configuration()
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
                        arm.set_hold(current)
                    elif op == "publish":
                        joints = validate_arm_q(request["joints"])
                        weight = float(request["weight"])
                        if not 0 <= weight <= 1:
                            raise ValueError("腕の制御重みが不正です")
                        now = time.monotonic()
                        if last_target is None and max(abs(a-b) for a, b in zip(joints, arm.current_arm()[7:])) > 0.025:
                            raise ValueError("初回の腕指令が現在姿勢と一致しません")
                        if last_target is not None:
                            allowed = 0.8 * (now-last_publish) + 0.005
                            if max(abs(a-b) for a, b in zip(joints, last_target)) > allowed:
                                raise ValueError("腕指令の変化速度が上限を超えました")
                        arm.publish(joints, weight)
                        last_publish, last_target = now, joints
                        active = weight > 0
                    elif op == "release":
                        arm.release()
                        active = False
                        last_target = last_publish = None
                    else:
                        raise ValueError("未対応の腕指令です")
                    reply = {"result": result}
                except Exception as error:
                    arm.release()
                    active = False
                    reply = {"error": str(error)}
                    connection.sendall((json.dumps(reply)+"\n").encode())
                    return
                connection.sendall((json.dumps(reply, allow_nan=False)+"\n").encode())
    finally:
        arm.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default="/tmp/g1_button_arm.sock")
    parser.add_argument("--network-interface", default="enp3s0")
    parser.add_argument("--arm", action="store_true", help="実機SDKへの接続を有効にする")
    parser.add_argument("--snapshot-out", help="関節角とIMUだけを読み、JSONを保存して終了する")
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
    try:
        server.bind(str(path))
        os.chmod(path, 0o600)
        server.listen(1)
        print(f"腕SDKブリッジ待機: {path}", flush=True)
        connection, _ = server.accept()
        with connection:
            serve_connection(connection, arm)
    finally:
        arm.release()
        server.close()
        if path.exists():
            path.unlink()


if __name__ == "__main__":
    main()
