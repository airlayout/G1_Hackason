"""G1 内蔵マイクの PCM を、有線側 UDP multicast から常時受信する。"""

from __future__ import annotations

import fcntl
import socket
import struct

import numpy as np

MIC_GROUP = "239.168.123.161"
MIC_PORT = 5555


def interface_ip(interface: str) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        packed = fcntl.ioctl(probe.fileno(), 0x8915, struct.pack("256s", interface[:15].encode()))
    return socket.inet_ntoa(packed[20:24])


def open_mic(local_ip: str, group: str = MIC_GROUP, port: int = MIC_PORT) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", port))
        membership = struct.pack("=4s4s", socket.inet_aton(group), socket.inet_aton(local_ip))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        sock.settimeout(0.2)
        return sock
    except Exception:
        sock.close()
        raise


def decode_pcm(data: bytes) -> np.ndarray:
    if not data or len(data) % 2:
        raise ValueError("PCM は空でない 16 bit little-endian の列が必要です")
    return np.frombuffer(data, dtype="<i2").astype(np.float32) / 32768.0
