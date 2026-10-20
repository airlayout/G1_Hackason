"""DDSスレッドは指令の受領だけを担当し、物理状態を変更しない。"""

from collections import Counter
import threading
import time

import numpy as np

from .codec import MotorTargets


class CommandMailbox:
    def __init__(self, codec, timeout_s=0.1):
        if not np.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("watchdogは有限の正数です")
        self.codec, self.timeout_s = codec, timeout_s
        self._lock = threading.Lock()
        self._latest = None
        self.accepted = 0
        self.rejected = Counter()

    def receive(self, message, *, now=None):
        try:
            targets = self.codec.decode_command(message)
        except (ValueError, TypeError, OverflowError) as error:
            with self._lock:
                self.rejected[str(error)] += 1
            return False
        received = time.monotonic() if now is None else now
        with self._lock:
            self._latest = targets, received
            self.accepted += 1
        return True

    def current(self, *, now=None):
        current_time = time.monotonic() if now is None else now
        with self._lock:
            latest = self._latest
        if latest is None or current_time - latest[1] > self.timeout_s:
            return None
        return latest[0]

    @staticmethod
    def damping():
        zero = np.zeros(29, dtype=np.float32)
        return MotorTargets(zero, zero, zero, zero, np.full(29, 2, dtype=np.float32))

    def stats(self):
        with self._lock:
            return {"accepted": self.accepted, "rejected": dict(self.rejected)}


class DdsBridge:
    def __init__(self, codec, *, domain=42, interface="lo", timeout_s=0.1):
        self.mailbox = CommandMailbox(codec, timeout_s)
        sdk = codec.sdk
        sdk.initialize(domain, interface)
        self.publisher = sdk.Publisher("rt/lowstate", sdk.LowState)
        self.publisher.Init()
        self.secondary_publisher = sdk.Publisher("rt/secondary_imu", sdk.IMUState)
        self.secondary_publisher.Init()
        self.subscriber = sdk.Subscriber("rt/lowcmd", sdk.LowCmd)
        self.subscriber.Init(self.mailbox.receive)
        self.published = 0
        self.secondary_published = 0

    def publish(self, message, *, secondary_imu=None):
        if not self.publisher.Write(message):
            raise RuntimeError("LowStateのDDS送信に失敗しました")
        self.published += 1
        if secondary_imu is not None:
            if not self.secondary_publisher.Write(secondary_imu):
                raise RuntimeError("secondary_imuのDDS送信に失敗しました")
            self.secondary_published += 1

    def close(self):
        self.subscriber.Close()
        self.publisher.Close()
        self.secondary_publisher.Close()
