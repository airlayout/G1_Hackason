"""SDK側プロセス(systemd常駐想定)のロジックをROS 2なしで検証するためのプロトタイプ。

Planning.md A-5 の結論により `unitree_mujoco` は高レベルAPI非対応と確定した。
つまり「速度指令→実際の歩行」は実機でしか検証できないが、
**SDK側プロセス自身のロジック(watchdog・状態機械・起動時ゼロ速度・エラーカウント)は
実機なしで検証できる**。本モジュールはそのためのプロトタイプ。

実際のUnitree呼び出しは `MoveBackend` として抽象化してあり、実機投入時は
`RealMoveBackend`(unitree_sdk2pyのLocoClientを叩く)に差し替えるだけでよい設計にしてある。

Planning.md の対応する決定事項:
- D-09: 20Hz周期送信とwatchdogはSDK側プロセスに置く
- D-11: 起動直後に必ずゼロ速度を送信する
- D-13: 異常時はゼロ速度を優先し、自動でDamp/ZeroTorqueへ遷移させない
- D-27: SetVelocity()を直接呼び、durationを明示指定する。continous_moveは使わない
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Protocol

from ipc_transport import PeerClosed, SeqPacketEndpoint, SeqPacketServer
from protocol import BridgeStatus, CmdPacket, StatePacket, monotonic_ns

CMD_MAX_PAYLOAD = 64
STATE_MAX_PAYLOAD = 96


class MoveBackend(Protocol):
    """G1への実移動指令の抽象。実機では unitree_sdk2py の LocoClient.SetVelocity() を叩く。"""

    def set_velocity(self, vx: float, vy: float, omega: float, duration: float) -> None:
        """成功時は何も返さない。SDK呼び出し失敗時は例外を送出する。"""
        ...


@dataclass
class MockMoveBackend:
    """テスト用。呼び出し履歴を記録し、意図的に失敗させることもできる。"""

    calls: list[tuple[float, float, float, float, int]] = field(default_factory=list)
    _fail_remaining: int = 0

    def set_velocity(self, vx: float, vy: float, omega: float, duration: float) -> None:
        self.calls.append((vx, vy, omega, duration, monotonic_ns()))
        if self._fail_remaining > 0:
            self._fail_remaining -= 1
            raise RuntimeError("mock SDK call failure (test injected)")

    def fail_next(self, n: int) -> None:
        self._fail_remaining = n

    def last_call(self) -> Optional[tuple[float, float, float, float, int]]:
        return self.calls[-1] if self.calls else None


@dataclass
class SdkBridgeConfig:
    cmd_sock_path: str
    state_sock_path: str
    cmd_rate_hz: float = 20.0
    cmd_timeout_s: float = 0.30  # 仕様書8章 cmd_timeout。この値以下にduration(下記)を収める
    sdk_command_duration_s: float = 0.20  # D-27。送信周期(1/cmd_rate_hz)より長く、cmd_timeout以下
    max_sdk_errors: int = 3
    poll_interval_s: float = 0.005

    def __post_init__(self) -> None:
        send_period = 1.0 / self.cmd_rate_hz
        if not (send_period < self.sdk_command_duration_s <= self.cmd_timeout_s):
            raise ValueError(
                "sdk_command_duration_s は 送信周期(1/cmd_rate_hz) より長く、"
                "cmd_timeout_s 以下でなければならない(D-27の原則)。"
                f" send_period={send_period}, duration={self.sdk_command_duration_s},"
                f" cmd_timeout={self.cmd_timeout_s}"
            )


class SdkBridgeProcess:
    """systemd常駐を想定したSDK側プロセスのロジック本体。"""

    def __init__(self, config: SdkBridgeConfig, move_backend: MoveBackend):
        self._cfg = config
        self._move = move_backend

        self._cmd_server = SeqPacketServer(config.cmd_sock_path, CMD_MAX_PAYLOAD)
        self._state_server = SeqPacketServer(config.state_sock_path, STATE_MAX_PAYLOAD)
        self._cmd_endpoint: Optional[SeqPacketEndpoint] = None
        self._state_endpoint: Optional[SeqPacketEndpoint] = None

        self._lock = threading.Lock()
        self._last_cmd: Optional[CmdPacket] = None
        self._status = BridgeStatus.DISCONNECTED
        self._sdk_error_count = 0
        self._state_seq = 0
        self._pose = (0.0, 0.0, 0.0)  # x, y, yaw (デモ用の簡易積分。実機ではLIOが供給する)

        self._running = threading.Event()
        self._threads: list[threading.Thread] = []

    # --- ライフサイクル -------------------------------------------------

    def start(self) -> None:
        # D-11: 起動直後に必ずゼロ速度を送信する。クラッシュ後の再起動でも前回速度を残さない
        self._move.set_velocity(0.0, 0.0, 0.0, self._cfg.sdk_command_duration_s)

        self._running.set()
        self._threads = [
            threading.Thread(target=self._accept_loop, daemon=True, name="g1bridge-accept"),
            threading.Thread(target=self._cmd_recv_loop, daemon=True, name="g1bridge-cmdrecv"),
            threading.Thread(target=self._periodic_loop, daemon=True, name="g1bridge-periodic"),
        ]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._running.clear()
        for t in self._threads:
            t.join(timeout=1.0)
        if self._cmd_endpoint:
            self._cmd_endpoint.close()
        if self._state_endpoint:
            self._state_endpoint.close()
        self._cmd_server.close()
        self._state_server.close()

    # --- 外部からの操作(仕様書5.2相当) -----------------------------------

    def clear_fault(self) -> bool:
        """/g1/clear_fault 相当。原因解消・安全確認後に操作者が呼ぶ(D-13: 自動復帰はしない)。"""
        with self._lock:
            if self._status != BridgeStatus.FAULT:
                return False
            self._status = BridgeStatus.STANDBY
            self._sdk_error_count = 0
            return True

    @property
    def status(self) -> BridgeStatus:
        with self._lock:
            return self._status

    @property
    def sdk_error_count(self) -> int:
        with self._lock:
            return self._sdk_error_count

    # --- 内部ループ ------------------------------------------------------

    def _accept_loop(self) -> None:
        while self._running.is_set():
            if self._cmd_endpoint is None:
                ep = self._cmd_server.accept_if_pending()
                if ep is not None:
                    self._cmd_endpoint = ep
                    with self._lock:
                        self._last_cmd = None  # 新規接続では新しい指令を待つ(古い指令を引き継がない)
                        if self._status == BridgeStatus.DISCONNECTED:
                            self._status = BridgeStatus.STANDBY
            if self._state_endpoint is None:
                ep = self._state_server.accept_if_pending()
                if ep is not None:
                    self._state_endpoint = ep
            time.sleep(self._cfg.poll_interval_s)

    def _cmd_recv_loop(self) -> None:
        while self._running.is_set():
            ep = self._cmd_endpoint
            if ep is not None:
                try:
                    raw = ep.recv_latest()
                except PeerClosed:
                    ep.close()
                    self._cmd_endpoint = None
                    with self._lock:
                        if self._status != BridgeStatus.FAULT:
                            self._status = BridgeStatus.DISCONNECTED
                    raw = None
                if raw is not None:
                    try:
                        pkt = CmdPacket.decode(raw)
                    except ValueError:
                        pkt = None
                    if pkt is not None:
                        with self._lock:
                            self._last_cmd = pkt
            time.sleep(self._cfg.poll_interval_s)

    def _periodic_loop(self) -> None:
        period = 1.0 / self._cfg.cmd_rate_hz
        next_tick = time.monotonic()
        while self._running.is_set():
            self._tick(period)
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()  # 遅延が蓄積した場合は仕切り直す

    def _tick(self, dt: float) -> None:
        with self._lock:
            cmd = self._last_cmd
            status = self._status
            cmd_connected = self._cmd_endpoint is not None

        if not cmd_connected:
            effective = (0.0, 0.0, 0.0)
            new_status = BridgeStatus.DISCONNECTED
        elif status == BridgeStatus.FAULT:
            # D-13: FAULT中は自動復帰せずゼロ速度を送り続ける。解除はclear_fault()のみ
            effective = (0.0, 0.0, 0.0)
            new_status = BridgeStatus.FAULT
        elif cmd is None or cmd.age_seconds() > self._cfg.cmd_timeout_s:
            # SDK側watchdog(D-09): 指令が無い、または古すぎる場合はゼロ速度
            effective = (0.0, 0.0, 0.0)
            new_status = BridgeStatus.READY
        else:
            effective = (cmd.vx, cmd.vy, cmd.omega)
            new_status = BridgeStatus.NAVIGATING if any(effective) else BridgeStatus.READY

        error_count = self._apply_move(effective)
        if error_count >= self._cfg.max_sdk_errors:
            new_status = BridgeStatus.FAULT
            effective = (0.0, 0.0, 0.0)  # FAULT遷移した回のstateは安全側に倒す

        with self._lock:
            self._status = new_status
            self._pose = _integrate_pose(self._pose, effective, dt)
            pose = self._pose
            err = self._sdk_error_count

        self._publish_state(new_status, pose, effective, err)

    def _apply_move(self, effective: tuple[float, float, float]) -> int:
        vx, vy, omega = effective
        try:
            self._move.set_velocity(vx, vy, omega, self._cfg.sdk_command_duration_s)
        except Exception:
            with self._lock:
                self._sdk_error_count += 1
                return self._sdk_error_count
        else:
            with self._lock:
                self._sdk_error_count = 0  # 「連続」失敗回数なので成功でリセット(仕様書8章)
                return self._sdk_error_count

    def _publish_state(
        self,
        status: BridgeStatus,
        pose: tuple[float, float, float],
        effective: tuple[float, float, float],
        err: int,
    ) -> None:
        ep = self._state_endpoint
        if ep is None:
            return
        self._state_seq += 1
        pkt = StatePacket(
            seq=self._state_seq,
            timestamp_ns=monotonic_ns(),
            status=status,
            x=pose[0],
            y=pose[1],
            yaw=pose[2],
            vx=effective[0],
            vy=effective[1],
            omega=effective[2],
            sdk_error_count=err,
        )
        try:
            ep.send_latest(pkt.encode())
        except PeerClosed:
            ep.close()
            self._state_endpoint = None


def _integrate_pose(pose: tuple[float, float, float], vel: tuple[float, float, float], dt: float):
    """デモ用の簡易積分。実機ではodometryはLIOから供給される(D-19)ため、この積分自体は本番では使わない。"""
    x, y, yaw = pose
    vx, vy, omega = vel
    x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
    y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
    yaw += omega * dt
    return (x, y, yaw)
