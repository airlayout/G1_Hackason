"""ROS側プロセスとSDK2側プロセスの間で交換するIPCペイロードの定義。

Planning.md D-05 の決定に基づく:
- 固定長構造体 + magic + シーケンス番号 + CLOCK_MONOTONIC送信タイムスタンプ
- cmdとstateでペイロードを分離する

CLOCK_MONOTONIC を使うのは、システム時計の変更（NTP補正等）に影響されず、
「送信からどれだけ時間が経過したか」だけを単調増加の時刻差で判定するため。
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from enum import IntEnum

MAGIC = 0x47314E56  # "G1NV" をASCIIコードとして4バイトに詰めた値


class BridgeStatus(IntEnum):
    """SDK側プロセスが報告する状態(仕様書7章の状態機械のうちSDK側が把握できる範囲)。"""

    DISCONNECTED = 0
    STANDBY = 1
    READY = 2
    NAVIGATING = 3
    FAULT = 4


def monotonic_ns() -> int:
    return time.monotonic_ns()


# --- Cmd: ROS側 -> SDK側 ---------------------------------------------------

_CMD_FORMAT = "<IQQddd"
_CMD_SIZE = struct.calcsize(_CMD_FORMAT)


@dataclass(frozen=True)
class CmdPacket:
    """速度指令。ROS側のSafety Managerが最終決定した安全な速度指令(/cmd_vel_safe相当)。"""

    seq: int
    timestamp_ns: int
    vx: float
    vy: float
    omega: float

    def encode(self) -> bytes:
        return struct.pack(_CMD_FORMAT, MAGIC, self.seq, self.timestamp_ns, self.vx, self.vy, self.omega)

    @staticmethod
    def decode(raw: bytes) -> "CmdPacket":
        if len(raw) != _CMD_SIZE:
            raise ValueError(f"CmdPacket: 想定サイズ {_CMD_SIZE} バイトに対し {len(raw)} バイトを受信した")
        magic, seq, ts, vx, vy, omega = struct.unpack(_CMD_FORMAT, raw)
        if magic != MAGIC:
            raise ValueError(f"CmdPacket: magic不一致 (0x{magic:08X})。異なるプロトコルのデータが混入している")
        return CmdPacket(seq=seq, timestamp_ns=ts, vx=vx, vy=vy, omega=omega)

    def age_seconds(self, now_ns: int | None = None) -> float:
        """このパケットが作られてからの経過時間(秒)。stale判定に使う。"""
        now_ns = monotonic_ns() if now_ns is None else now_ns
        return max(0, now_ns - self.timestamp_ns) / 1e9


def make_cmd(seq: int, vx: float, vy: float, omega: float) -> CmdPacket:
    return CmdPacket(seq=seq, timestamp_ns=monotonic_ns(), vx=vx, vy=vy, omega=omega)


ZERO_CMD_TEMPLATE = (0.0, 0.0, 0.0)


# --- State: SDK側 -> ROS側 --------------------------------------------------

_STATE_FORMAT = "<IQQBxxxddddddI"  # magic,seq,ts,status,pad3, x,y,yaw,vx,vy,omega(6個), err
_STATE_SIZE = struct.calcsize(_STATE_FORMAT)


@dataclass(frozen=True)
class StatePacket:
    """G1の状態。odometry相当(x,y,yaw,vx,vy,omega)とbridgeの健全性情報。

    x, y, yaw は G1 起動時点を原点とする odom フレーム相当(累積ドリフトを持ちうる)。
    実機ではSDK側プロセスがG1 stateから取得した値をそのまま詰める。
    """

    seq: int
    timestamp_ns: int
    status: BridgeStatus
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    omega: float
    sdk_error_count: int

    def encode(self) -> bytes:
        return struct.pack(
            _STATE_FORMAT,
            MAGIC,
            self.seq,
            self.timestamp_ns,
            int(self.status),
            self.x,
            self.y,
            self.yaw,
            self.vx,
            self.vy,
            self.omega,
            self.sdk_error_count,
        )

    @staticmethod
    def decode(raw: bytes) -> "StatePacket":
        if len(raw) != _STATE_SIZE:
            raise ValueError(f"StatePacket: 想定サイズ {_STATE_SIZE} バイトに対し {len(raw)} バイトを受信した")
        magic, seq, ts, status, x, y, yaw, vx, vy, omega, err = struct.unpack(_STATE_FORMAT, raw)
        if magic != MAGIC:
            raise ValueError(f"StatePacket: magic不一致 (0x{magic:08X})")
        return StatePacket(
            seq=seq,
            timestamp_ns=ts,
            status=BridgeStatus(status),
            x=x,
            y=y,
            yaw=yaw,
            vx=vx,
            vy=vy,
            omega=omega,
            sdk_error_count=err,
        )

    def age_seconds(self, now_ns: int | None = None) -> float:
        now_ns = monotonic_ns() if now_ns is None else now_ns
        return max(0, now_ns - self.timestamp_ns) / 1e9
