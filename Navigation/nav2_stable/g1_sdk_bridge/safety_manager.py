"""ROS側 Safety Manager / Command Router のロジック本体(仕様書7章・8章に対応)。

ROS 2 が未インストールの環境で先行開発するため、ROS 2 ノードとしてではなく
「入力(Twist相当) -> 出力(IPC送信)」の純粋なロジックとして実装する。
実際の rclpy ノードは、これをラップしてトピック/サービスに接続するだけになる想定。

Planning.md の対応する決定事項:
- D-10: watchdogは二重化する。ROS側は「明示的ゼロ送信」+「送信停止」の両方を行う
- D-13: 異常時はゼロ速度を優先し、Damp/ZeroTorqueへ自動遷移させない
- D-14: 速度デッドバンド(min_vx / min_wz)を追加する
- D-15: MVPはvy=0(横移動無効)から開始する
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Optional


class NavState(IntEnum):
    """仕様書7章の状態機械。"""

    DISCONNECTED = 0
    STANDBY = 1
    READY = 2
    NAVIGATING = 3
    FAULT = 4
    E_STOP = 5


@dataclass
class SafetyLimits:
    """仕様書8章「初期安全パラメータ」に対応。値そのものは実測前の仮値。"""

    max_vx: float = 0.20
    max_vy: float = 0.0  # D-15: MVPでは横移動無効
    max_wz: float = 0.30
    max_ax: float = 0.20
    max_ay: float = 0.15
    max_awz: float = 0.40
    # D-14: これ未満はデッドバンドでゼロに丸める(歩容が成立しない速度域)。
    # 既定値は0(無効)。2026-09-09のNav2統合dry-runで、Nav2のrotate-to-heading起動フェーズの
    # 微小角速度をデッドバンドが常時ゼロに切り捨て、ロボットが永久に動き出せなくなる不具合を
    # 発見した(QUESTIONS.md Q8で選択肢を整理し、ユーザーが(d)を選択: Phase 1のU-12実測で
    # 実際の閾値が判明するまで無効化しておく)。apply_deadband自体のロジックは維持しているため、
    # Phase 1で実測した値をここに設定すれば有効化できる。
    min_vx: float = 0.0
    min_wz: float = 0.0
    cmd_timeout_s: float = 0.30
    max_sdk_errors: int = 3


Vel = tuple[float, float, float]


def clamp(v: Vel, limits: SafetyLimits) -> Vel:
    vx, vy, wz = v
    vx = max(-limits.max_vx, min(limits.max_vx, vx))
    vy = max(-limits.max_vy, min(limits.max_vy, vy))
    wz = max(-limits.max_wz, min(limits.max_wz, wz))
    return (vx, vy, wz)


def apply_deadband(v: Vel, limits: SafetyLimits) -> Vel:
    """D-14: 微小速度をゼロに丸める。

    Goal到達直前にNav2が出す微小速度を素通しすると、二足は歩容が成立せず
    足踏みし続けて到達判定(速度閾値以下を1秒継続)に入れなくなる。
    """
    vx, vy, wz = v
    if abs(vx) < limits.min_vx:
        vx = 0.0
    if abs(wz) < limits.min_wz:
        wz = 0.0
    # vy はMVPで無効(D-15)なので同じ扱いにはしない。呼び出し側でmax_vy=0によりclampで既に0になる
    return (vx, vy, wz)


def accel_limit(prev: Vel, target: Vel, dt: float, limits: SafetyLimits) -> Vel:
    """1制御周期あたりの変化量を加速度上限で制限する(仕様書8章 max_ax/max_ay/max_awz)。"""
    if dt <= 0:
        return prev

    def step(p: float, t: float, max_rate: float) -> float:
        max_delta = max_rate * dt
        delta = t - p
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta
        return p + delta

    vx = step(prev[0], target[0], limits.max_ax)
    vy = step(prev[1], target[1], limits.max_ay)
    wz = step(prev[2], target[2], limits.max_awz)
    return (vx, vy, wz)


IpcSend = Callable[[float, float, float], None]


class SafetyManager:
    """優先順位(仕様書8章)を実装する。

    E-stop > 通信/SDK/TF/センサー異常 > Nav2指令 の順で、下位の入力を上書きする。
    NAVIGATING状態のときだけ非ゼロ速度がIPCへ送られる(仕様書8章の状態別許可表)。
    """

    def __init__(self, limits: SafetyLimits, ipc_send: IpcSend, now_fn: Callable[[], float] | None = None):
        self.limits = limits
        self._ipc_send = ipc_send
        self._now = now_fn or _default_now
        self.state = NavState.DISCONNECTED
        self._last_nav_cmd_time: Optional[float] = None
        self._last_output: Vel = (0.0, 0.0, 0.0)
        self._last_tick_time: Optional[float] = None
        self.fault_reason: Optional[str] = None

    # --- Bridge接続状態 ------------------------------------------------

    def on_bridge_connected(self) -> None:
        if self.state == NavState.DISCONNECTED:
            self.state = NavState.STANDBY

    def on_bridge_disconnected(self) -> None:
        self.state = NavState.DISCONNECTED
        self._send_zero()

    # --- Navigation Enable/Disable (仕様書5.2 /g1/enable_navigation) ----

    def enable_navigation(self, enable: bool) -> bool:
        if enable:
            if self.state != NavState.READY:
                return False
            self.state = NavState.NAVIGATING
            now = self._now()
            # _last_nav_cmd_time はここでは設定しない。Nav2はGoal計画に数百ms〜数秒かかることが
            # あり(実測: 合成マップでのdry-runで約1秒)、enable直後にcmd_timeoutのカウントを
            # 始めると最初の指令が届く前にFAULTへ誤って遷移してしまう(2026-09-09発見)。
            # 最初の指令を受け取るまではtick()のタイムアウト判定を待機させる(下記tick参照)。
            # 物理的な安全性はSDK側watchdog(D-09)が別途担保するため、この猶予は安全上問題ない。
            self._last_nav_cmd_time = None
            self._last_tick_time = now  # 有効化した瞬間を基準にaccel_limitのdtを測り始める
            self._last_output = (0.0, 0.0, 0.0)
            return True
        else:
            if self.state == NavState.NAVIGATING:
                self.state = NavState.READY
                self._send_zero()
            return True

    def mark_ready(self) -> None:
        """歩行可能・センサー正常が確認できたときにSTANDBY->READYへ(仕様書7章)。"""
        if self.state == NavState.STANDBY:
            self.state = NavState.READY

    # --- Nav2からの速度指令(仕様書5.1 /cmd_vel_smoothed相当の入口) -------

    def on_nav_twist(self, vx: float, vy: float, omega: float) -> Vel:
        """Nav2からのTwistを受け、安全処理後にIPCへ送る。戻り値は実際に送った値(テスト用)。"""
        now = self._now()
        self._last_nav_cmd_time = now

        if self.state != NavState.NAVIGATING:
            # 仕様書8章の状態別許可表: NAVIGATING以外では非ゼロ速度を送らない
            return self._send_zero()

        target = clamp((vx, vy, omega), self.limits)
        target = apply_deadband(target, self.limits)
        # _last_tick_time は enable_navigation(True) で必ず設定済み(NAVIGATING以外ではここに来ない)。
        # accel_limit自身がdt<=0を「変化なし」として扱うため、ここで特別扱いはしない。
        dt = max(0.0, now - self._last_tick_time) if self._last_tick_time is not None else 0.0
        output = accel_limit(self._last_output, target, dt, self.limits)
        self._last_output = output
        self._last_tick_time = now
        self._ipc_send(*output)
        return output

    # --- 定期watchdog(D-10: ROS側の明示的ゼロ送信 + 送信停止) -----------

    def tick(self) -> None:
        """タイマーで周期呼び出しする(ROS 2ではWallTimer相当)。

        NAVIGATING中に指令が途絶えたら、SDK側watchdog(D-09)を待たずに
        ROS側からも即座にゼロを送り、FAULTへ遷移する。
        """
        if self.state != NavState.NAVIGATING:
            return
        if self._last_nav_cmd_time is None:
            return  # まだ最初の指令を受けていない。Nav2の計画時間を待つ(enable_navigation参照)
        now = self._now()
        if now - self._last_nav_cmd_time > self.limits.cmd_timeout_s:
            self._transition_fault("cmd_timeout")

    # --- 異常系(仕様書8章の停止条件) ------------------------------------

    def e_stop(self) -> None:
        self.state = NavState.E_STOP
        self._send_zero()

    def clear_e_stop(self) -> bool:
        """手動解除+安全確認(仕様書7章 E_STOP -> STANDBY)。呼び出し側が安全確認済みであること。"""
        if self.state != NavState.E_STOP:
            return False
        self.state = NavState.STANDBY
        self.fault_reason = None
        return True

    def on_tf_stale(self) -> None:
        self._transition_fault("tf_stale")

    def on_sensor_stale(self) -> None:
        self._transition_fault("sensor_stale")

    def on_bridge_error(self) -> None:
        self._transition_fault("sdk_bridge_error")

    def on_collision_stop(self) -> None:
        """Collision Monitorからの停止指令。FAULTにはせず、その場でゼロにするだけ(再開可能)。"""
        self._send_zero()

    def clear_fault(self) -> bool:
        """仕様書5.2 /g1/clear_fault。原因解消・安全確認後に呼ぶ。"""
        if self.state != NavState.FAULT:
            return False
        self.state = NavState.STANDBY
        self.fault_reason = None
        return True

    def _transition_fault(self, reason: str) -> None:
        if self.state == NavState.E_STOP:
            return  # E-stopが最優先。FAULTで上書きしない
        self.state = NavState.FAULT
        self.fault_reason = reason
        self._send_zero()

    def _send_zero(self) -> Vel:
        self._last_output = (0.0, 0.0, 0.0)
        self._ipc_send(0.0, 0.0, 0.0)
        return self._last_output


def _default_now() -> float:
    import time

    return time.monotonic()
