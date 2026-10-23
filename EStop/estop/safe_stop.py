"""安全停止の状態遷移。副作用を持たない純粋なロジック（入力→状態と指令）。

トルクをゼロにすると立位の G1 は倒れる。これは「停止」ではなく「転倒」なので、
**動きを止めて立ったまま静止する** ことを目標にする。
プロジェクト決定 D-13（異常時に自動で Damp/ZeroTorque へ遷移させない）に従い、
静止できなくても自動ではダンピングしない。ゼロ速度を保って警報(UNSTABLE)を出し、
ダンピングは操作者の明示操作(`request_damp`)でだけ入る。
"""

from __future__ import annotations

import enum
from dataclasses import dataclass


class State(enum.Enum):
    ARMED = "armed"          # 通常運転。上位層の指令を通す
    STOPPING = "stopping"    # 停止要求済み。減速して静止待ち
    SAFE_HOLD = "safe_hold"  # 立位で静止。保持のみ
    UNSTABLE = "unstable"    # 静止できない/傾いた。ゼロ速度を保ち警報。操作者の判断待ち
    DAMPED = "damped"        # 操作者が明示的にダンピングを指示した


@dataclass(frozen=True)
class Command:
    """各部位への指令。毎周期同じ値を返す（冪等）。"""

    locomotion: str  # "run"=上位に任せる / "stop_move"=速度0で静止 / "damp"=ダンピング
    arms: str        # "run" / "hold"=現在の関節角を保持
    hands: str       # "run" / "hold"=握りを現状維持（開かない・強めない）


@dataclass(frozen=True)
class Observation:
    """不明(None)は「静止を確認できない」扱い。静止判定は満たさない。"""

    tilt_deg: float | None        # 鉛直からの傾き
    speed_mps: float | None       # 胴体の水平速度
    ang_speed_rps: float | None   # 胴体の角速度の大きさ


# 以下はすべて**未実測の仮置き**。実機で転倒直前の傾きと停止時間を測って決める。
@dataclass(frozen=True)
class SafeStopConfig:
    fall_tilt_deg: float = 35.0     # これ以上傾いたら UNSTABLE
    stable_tilt_deg: float = 10.0   # 静止判定で許す傾き
    still_speed_mps: float = 0.05
    still_ang_speed_rps: float = 0.10
    settle_s: float = 0.5           # 静止条件を連続で満たす時間
    stop_timeout_s: float = 3.0     # この時間で静止できなければ UNSTABLE
    rearm_s: float = 2.0            # 復帰前にボタンが連続で健全であるべき時間


_RUN = Command("run", "run", "run")
_HOLD = Command("stop_move", "hold", "hold")
_DAMP = Command("damp", "hold", "hold")


class SafeStop:
    """`step()` を一定周期で呼ぶ。停止は即時ラッチし、自動では復帰しない。"""

    def __init__(self, config: SafeStopConfig | None = None) -> None:
        self._cfg = config or SafeStopConfig()
        self._state = State.ARMED
        self._t_trigger = 0.0
        self._t_still_since: float | None = None
        self._t_button_ok_since: float | None = None
        self._events: list[tuple[float, State, str]] = []

    @property
    def state(self) -> State:
        return self._state

    @property
    def permits_motion(self) -> bool:
        """上位層（歩行・腕・ナビ）が指令を出してよいか。"""
        return self._state is State.ARMED

    @property
    def alarm(self) -> bool:
        """操作者の判断が要る状態。"""
        return self._state is State.UNSTABLE

    def pop_events(self) -> list[tuple[float, State, str]]:
        """前回以降の遷移 (時刻, 遷移先, 理由)。停止イベント記録(incident log)用。"""
        out, self._events = self._events, []
        return out

    def step(self, now: float, button_ok: bool, obs: Observation) -> Command:
        self._track_button(now, button_ok)
        if self._state is State.ARMED and not button_ok:
            self._t_trigger = now
            self._t_still_since = None
            self._go(now, State.STOPPING, "ボタン開/断線/USB抜け")
        if self._state in (State.STOPPING, State.UNSTABLE):
            self._step_unsettled(now, obs)
        elif self._state is State.SAFE_HOLD and self._fallen(obs):
            self._go(now, State.UNSTABLE, f"静止後に傾き{obs.tilt_deg:.0f}°")
        return self._command()

    def request_damp(self, now: float) -> bool:
        """操作者の明示操作でダンピングへ。ARMED では受け付けない（先に停止させる）。"""
        if self._state in (State.ARMED, State.DAMPED):
            return False
        self._go(now, State.DAMPED, "操作者がダンピングを指示")
        return True

    def reset(self, now: float, operator_ack: bool) -> bool:
        """復帰。ボタンが rearm_s 連続で健全、かつ人の明示確認があるときだけ。

        DAMPED/UNSTABLE から復帰しても ARMED に戻るだけ。立たせ直しは上位の手順で行う。
        """
        ok_for = (
            self._t_button_ok_since is not None
            and now - self._t_button_ok_since >= self._cfg.rearm_s
        )
        if self._state is State.ARMED or not (operator_ack and ok_for):
            return False
        self._go(now, State.ARMED, "操作者が復帰")
        return True

    def _go(self, now: float, state: State, reason: str) -> None:
        self._state = state
        self._events.append((now, state, reason))

    def _track_button(self, now: float, button_ok: bool) -> None:
        if not button_ok:
            self._t_button_ok_since = None
        elif self._t_button_ok_since is None:
            self._t_button_ok_since = now

    def _step_unsettled(self, now: float, obs: Observation) -> None:
        """STOPPING と UNSTABLE 共通: 静止を連続で確認できたら SAFE_HOLD。"""
        if self._fallen(obs):
            if self._state is not State.UNSTABLE:
                self._go(now, State.UNSTABLE, f"傾き{obs.tilt_deg:.0f}°")
            self._t_still_since = None
            return
        if self._still(obs):
            if self._t_still_since is None:
                self._t_still_since = now
            if now - self._t_still_since >= self._cfg.settle_s:
                self._go(now, State.SAFE_HOLD, "静止を確認")
                return
        else:
            self._t_still_since = None
        timed_out = now - self._t_trigger >= self._cfg.stop_timeout_s
        if self._state is State.STOPPING and timed_out:
            self._go(now, State.UNSTABLE, "停止要求から時間内に静止できない")

    def _fallen(self, obs: Observation) -> bool:
        return obs.tilt_deg is not None and obs.tilt_deg >= self._cfg.fall_tilt_deg

    def _still(self, obs: Observation) -> bool:
        c = self._cfg
        if obs.tilt_deg is None or obs.ang_speed_rps is None:
            return False
        speed_ok = obs.speed_mps is None or obs.speed_mps <= c.still_speed_mps
        return (
            obs.tilt_deg <= c.stable_tilt_deg
            and obs.ang_speed_rps <= c.still_ang_speed_rps
            and speed_ok
        )

    def _command(self) -> Command:
        if self._state is State.ARMED:
            return _RUN
        if self._state is State.DAMPED:
            return _DAMP
        return _HOLD
