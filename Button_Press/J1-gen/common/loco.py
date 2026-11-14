"""立つ位置まで寄るときに、G1 の純正の歩行コントローラで「1 回分だけ」動かす（LocoClient.SetVelocity）。

    mover = LocoMover(SdkLocoBackend(), LegStillness(reader, cfg["still"]), cfg, go=True)
    pulse = mover.plan("forward", 0.20)       # 20 cm 進みたい → 速度と時間（小さすぎれば None）
    result = mover.pulse(pulse)               # 送って、脚が止まるまで待つ
    mover.stop()                              # 止める（with 文の終わりでも呼ぶ）

なぜ「1 回分」か（Navigation 班の実機の記録。Navigation/nav2_stable/Planning.md の U-07・U-12、2026-09-09）:
- 遅い指令では歩き出さない: vx 0.1 は関節が少し動くだけ、0.2 は歩くが進む向きが定まらない（+98.6° ずれた）、
  0.3 で前に進む（1 秒で約 23 cm、向きは −18〜−20° ずれる）。角速度は 0.02 で動かず、0.3 で回る
- 指令の時間（duration）が終わると自分で止まるが、約 1.9 秒は歩き続ける（完全に止まるまで 3.4 秒）
そのため「数 cm だけ動く」はできない見込み。下限より速い速度を短い時間だけ送り、止まるのを待ってから頭カメラで
測り直す（どれだけ動いたかは C の寄る流れで観測する）。

安全の作り:
- SetVelocity に duration を明示する（max_duration_s 以下）。continous_move（duration 10 日）は使わない（Navigation の D-27 と同じ）。
  このプロセスが落ちても、duration が終われば G1 が自分で止まる
- 1 回に動かす軸は 1 つだけ。下限より遅い・上限より速い指令は送らない（ValueError）。後ろへは既定で禁止
  （頭のセンサの後ろ上が死角。Navigation の loco_driver.py と同じ）
- 回数と、前に進んだ合計に上限（寄る流れが間違って回り続けても止まる）
- go=False（既定）なら SDK を呼ばず、送るはずの指令を表示するだけ
- 脚が止まらなければ StopMove を送り、止まらなかったと返す（呼ぶ側は中止する）
- ⚠️ 腕は下ろしておくこと（arm_sdk の weight 0）。このモジュールは腕を見ない
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np

# 動かす向きと、SetVelocity のどの成分をどの符号で使うか
AXES: dict[str, tuple[int, float]] = {
    "forward": (0, 1.0),
    "backward": (0, -1.0),
    "left": (1, 1.0),
    "right": (1, -1.0),
    "turn_left": (2, 1.0),
    "turn_right": (2, -1.0),
}
# AXES の成分の番号 → 設定の速度の名前
COMPONENTS = ("vx", "vy", "wz")
LEG_IDX: tuple[int, ...] = tuple(range(12))


class LocoError(RuntimeError):
    """足を動かせない（設定の上限・FSM・SDK の失敗など。理由は文で持つ）。"""


@dataclass
class Pulse:
    """1 回分の指令。速度は SetVelocity と同じ（vx 前 [m/s]、vy 左 [m/s]、wz 左回り [rad/s]）。"""

    vx: float
    vy: float
    wz: float
    duration_s: float
    axis: str = ""
    amount: float = 0.0  # 動きたかった量（m か rad。記録用）

    def summary(self) -> str:
        return (f"{self.axis or '?'}: vx {self.vx:+.2f} vy {self.vy:+.2f} wz {self.wz:+.2f} を {self.duration_s:.2f} 秒"
                f"（{self.amount * (100 if not self.axis.startswith('turn') else 180 / np.pi):+.1f}"
                f"{'°' if self.axis.startswith('turn') else ' cm'} 動くつもり）")


@dataclass
class PulseResult:
    pulse: Pulse
    sent: bool        # SDK に送った（go=False なら False）
    stopped: bool     # 脚が止まったのを確かめた（go=False なら True）
    settle_s: float   # 送ってから止まったと分かるまで [s]
    note: str = ""


class LocoBackend(Protocol):
    def set_velocity(self, vx: float, vy: float, wz: float, duration_s: float) -> None: ...

    def stop(self) -> None: ...

    def fsm_id(self) -> int | None: ...


class Stillness(Protocol):
    def is_still(self) -> bool: ...


class SdkLocoBackend:
    """実機（unitree_sdk2py の LocoClient、RPC の "sport" サービス）。DDS は先に初期化しておく（common/dds.py の init_dds）。"""

    def __init__(self, timeout_s: float = 3.0) -> None:
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

        self._client = LocoClient()
        self._client.SetTimeout(float(timeout_s))
        self._client.Init()

    def set_velocity(self, vx: float, vy: float, wz: float, duration_s: float) -> None:
        code = self._client.SetVelocity(float(vx), float(vy), float(wz), float(duration_s))
        if code != 0:
            raise LocoError(f"SetVelocity が失敗した（code {code}）。歩ける状態（FSM）か、sport サービスが動いているかを確かめる")

    def stop(self) -> None:
        self._client.StopMove()

    def fsm_id(self) -> int | None:
        code, data = self._client.GetFsmId()
        return int(data) if code == 0 and data is not None else None


class PrintLocoBackend:
    """SDK を呼ばず、送るはずの指令を表示するだけ（go=False のとき LocoMover が使う）。"""

    def __init__(self, log: Callable[[str], None] = print) -> None:
        self.log = log
        self.sent: list[tuple[float, float, float, float]] = []

    def set_velocity(self, vx: float, vy: float, wz: float, duration_s: float) -> None:
        self.sent.append((vx, vy, wz, duration_s))
        self.log(f"[loco] （送らない）SetVelocity({vx:+.2f}, {vy:+.2f}, {wz:+.2f}, {duration_s:.2f})")

    def stop(self) -> None:
        self.log("[loco] （送らない）StopMove()")

    def fsm_id(self) -> int | None:
        return None


class LegStillness:
    """lowstate の脚 12 関節の角速度で、止まったかを見る（hold_s のあいだ続けて max_leg_dq_rad_s 以下なら止まった）。

    max_leg_dq_rad_s を超えても、spike_s より短く、spike_max_dq_rad_s 以下で跳ねた値は動いたとみなさない（数え直さない）。
    11/14 の実機: 止まって立っているのに、0.3〜0.5 秒おきに 1 回だけ 0.2〜0.3 rad/s が跳ね、そのたびに数え直して
    寄る流れが timeout_s（6 秒）に届いて中止した。歩いている間は 1〜5 rad/s が続くので、跳ねとは区別できる。
    """

    def __init__(self, reader: Any, cfg: dict[str, Any], now: Callable[[], float] = time.monotonic) -> None:
        self.reader = reader
        self.max_dq = float(cfg["max_leg_dq_rad_s"])
        self.hold_s = float(cfg["hold_s"])
        self.max_age_s = float(cfg.get("max_state_age_s", 0.2))
        self.spike_s = float(cfg.get("spike_s", 0.0))
        self.spike_max_dq = float(cfg.get("spike_max_dq_rad_s", self.max_dq))
        self.now = now
        self._since: float | None = None
        self._over_since: float | None = None

    def is_still(self) -> bool:
        st = self.reader.latest()
        t = self.now()
        if st is None or t - float(st.stamp) > self.max_age_s:
            # lowstate が古い（届いていない）ときは、止まったとみなさない
            self._since = None
            self._over_since = None
            return False
        dq = float(np.abs(np.asarray(st.dq)[list(LEG_IDX)]).max())
        if dq > self.max_dq:
            if self._over_since is None:
                self._over_since = t
            if t - self._over_since >= self.spike_s or dq > self.spike_max_dq:
                # spike_s 以上続けて超えた（spike_s が 0 なら、1 回でも超えた）か、跳ねにしては大きい: 動いている。数え直す
                self._since = None
            return False
        self._over_since = None
        if self._since is None:
            self._since = t
        return t - self._since >= self.hold_s


class LocoMover:
    def __init__(self, backend: LocoBackend | None, stillness: Stillness | None, cfg: dict[str, Any], go: bool = False,
                 sleep: Callable[[float], None] = time.sleep, now: Callable[[], float] = time.monotonic,
                 log: Callable[[str], None] = print) -> None:
        """go=False なら backend は使わず（None でよい）、表示するだけ。stillness は go=True のとき要る。"""
        if go and (backend is None or stillness is None):
            raise ValueError("go=True には backend と stillness が要る")
        self.go = bool(go)
        self.backend: LocoBackend = backend if go else PrintLocoBackend(log)
        self.stillness = stillness
        self.cfg = cfg
        self.sleep, self.now, self.log = sleep, now, log
        self.n_pulses = 0
        self.forward_total_m = 0.0
        # 横へ動いた見込みの合計（+ が左）。左右に行ったり来たりした分は打ち消す
        self.side_total_m = 0.0
        self._fsm_checked = False
        self._moving = False

    # ---- 計画 ------------------------------------------------------------------

    def plan(self, axis: str, amount: float) -> Pulse | None:
        """axis の向きに amount（m か rad。正の値）動く 1 回分。1 回で動ける最小より十分小さければ None（直さない）。

        動く量 = rate × 時間 とみなす（rate は実機で測った「1 秒あたりに動いた量」。歩き終わりの分も含めた値）。
        時間は min_duration_s〜max_duration_s に収める（大きければ 1 回では足りず、呼ぶ側がもう一度測って続ける）。
        """
        if axis not in AXES:
            raise ValueError(f"知らない向き: {axis}（{', '.join(AXES)}）")
        a = self.cfg["axes"][axis]
        if not a.get("enabled", True):
            raise LocoError(f"{axis} は設定で使わないことになっている（approach.yaml の loco.axes.{axis}.enabled）")
        amount = abs(float(amount))
        rate = float(a["rate"])
        t_min, t_max = float(a["min_duration_s"]), float(a["max_duration_s"])
        if amount < rate * t_min * float(self.cfg["min_fraction"]):
            return None
        t = float(np.clip(amount / rate, t_min, t_max))
        v = [0.0, 0.0, 0.0]
        k, sign = AXES[axis]
        v[k] = sign * float(a["speed"])
        return Pulse(v[0], v[1], v[2], t, axis, amount)

    def min_step(self, axis: str) -> float:
        """axis の向きに 1 回で動く最小の量（m か rad）。"""
        a = self.cfg["axes"][axis]
        return float(a["rate"]) * float(a["min_duration_s"])

    def check(self, p: Pulse) -> None:
        """送ってよい指令か（だめなら ValueError / LocoError）。"""
        v = np.array([p.vx, p.vy, p.wz], dtype=float)
        nz = np.flatnonzero(v != 0.0)
        if len(nz) != 1:
            raise ValueError(f"1 回に動かす軸は 1 つだけ（{p.summary()}）")
        k = int(nz[0])
        name = COMPONENTS[k]
        lo, hi = float(self.cfg["min_speed"][name]), float(self.cfg["max_speed"][name])
        if not lo <= abs(v[k]) <= hi:
            raise ValueError(f"{name} {v[k]:+.2f} が {lo:.2f}〜{hi:.2f} の外（遅いと歩き出さない・向きが定まらない）")
        if not 0.0 < p.duration_s <= float(self.cfg["max_duration_s"]):
            raise ValueError(f"時間 {p.duration_s:.2f} 秒が 0〜{float(self.cfg['max_duration_s']):.2f} 秒の外")
        if p.vx < 0 and not self.cfg["axes"]["backward"].get("enabled", False):
            raise LocoError("後ろへは動かさない（approach.yaml の loco.axes.backward.enabled が false）")
        lim = self.cfg["limits"]
        if self.n_pulses >= int(lim["max_pulses"]):
            raise LocoError(f"動いた回数が上限（{int(lim['max_pulses'])} 回）に達した。人が確かめる")
        if p.vx > 0:
            add = float(self.cfg["axes"]["forward"]["rate"]) * p.duration_s
            if self.forward_total_m + add > float(lim["max_forward_total_m"]):
                raise LocoError(f"前に進んだ合計が上限（{float(lim['max_forward_total_m']):.2f} m）を超える。人が確かめる")
        if p.vy != 0 and lim.get("max_side_total_m") is not None:
            # 頭カメラは前の下しか見ないので、横の物（11/14 はハーネスの台）は見えない。横へ動く合計に上限を置く
            side = self.side_step(p)
            if abs(self.side_total_m + side) > float(lim["max_side_total_m"]):
                raise LocoError(f"横へ動いた合計が上限（±{float(lim['max_side_total_m']):.2f} m）を超える"
                                f"（今 {self.side_total_m * 100:+.0f} cm、この 1 回で {side * 100:+.0f} cm）。人が確かめる")

    def side_step(self, p: Pulse) -> float:
        """この 1 回で横へ動く見込み [m]（+ が左。rate × 時間。前・回るなら 0）。"""
        if p.vy == 0:
            return 0.0
        axis = "left" if p.vy > 0 else "right"
        return float(np.sign(p.vy)) * float(self.cfg["axes"][axis]["rate"]) * p.duration_s

    # ---- 送る ------------------------------------------------------------------

    def pulse(self, p: Pulse) -> PulseResult:
        """1 回分を送り、脚が止まるまで待つ。"""
        self.check(p)
        if self.go and not self._fsm_checked:
            self._check_fsm()
        self.log(f"[loco] {p.summary()}")
        t0 = self.now()
        self._moving = True
        self.backend.set_velocity(p.vx, p.vy, p.wz, p.duration_s)
        self.n_pulses += 1
        if p.vx > 0:
            self.forward_total_m += float(self.cfg["axes"]["forward"]["rate"]) * p.duration_s
        self.side_total_m += self.side_step(p)
        if not self.go:
            self._moving = False
            return PulseResult(p, sent=False, stopped=True, settle_s=0.0, note="送っていない（go=False）")
        # 指令の時間のあいだは待つ（歩いている）。そのあと、止まるまで見る
        self.sleep(p.duration_s)
        deadline = t0 + p.duration_s + float(self.cfg["still"]["timeout_s"])
        assert self.stillness is not None
        while self.now() < deadline:
            if self.stillness.is_still():
                self._moving = False
                return PulseResult(p, sent=True, stopped=True, settle_s=self.now() - t0)
            self.sleep(float(self.cfg["still"]["poll_s"]))
        self.backend.stop()
        self._moving = False
        return PulseResult(p, sent=True, stopped=False, settle_s=self.now() - t0,
                           note=f"{float(self.cfg['still']['timeout_s']):.1f} 秒待っても脚が止まらない。StopMove を送った")

    def stop(self) -> None:
        """止める（動いている途中なら StopMove）。"""
        if self._moving:
            self.backend.stop()
            self._moving = False

    def __enter__(self) -> "LocoMover":
        return self

    def __exit__(self, *exc: Any) -> None:
        # 例外（Ctrl+C を含む）で抜けるときも、動いている途中なら止める
        self.stop()

    def _check_fsm(self) -> None:
        allowed = self.cfg.get("allowed_fsm_ids")
        if allowed:
            fsm = self.backend.fsm_id()
            if fsm not in [int(x) for x in allowed]:
                raise LocoError(f"FSM が {fsm}（歩ける状態 {list(allowed)} ではない）。リモコンで立たせ、歩ける状態にする")
        self._fsm_checked = True


# ---- テストとシミュレーション用の偽物の足 ------------------------------------------------


class FakeLoco:
    """動きを真似る偽物の足（LocoBackend と Stillness の両方）。床の上の pelvis の位置 (x, y, yaw) を持つ。

    実機の記録（U-07・U-12）の形を真似る: 下限より遅いと動かない、向きが少しずれる、歩き終わりの分だけ多く動く、ばらつく。
    時間は sleep・now を自分で進める（clock.sleep を LocoMover に渡す）。数値は仮で、実機で測った値に合わせる（E）。
    """

    def __init__(self, min_speed: dict[str, float] | None = None, rate_ratio: float = 0.75, tail_s: float = 1.9,
                 tail_ratio: float = 0.0, heading_error_deg: float = 0.0, noise: float = 0.0, seed: int = 0,
                 rates: dict[str, float] | None = None) -> None:
        self.min_speed = min_speed or {"vx": 0.25, "vy": 0.2, "wz": 0.2}
        # rates が無ければ: 実際の速さ / 指令の速さ（duration のあいだ。前・横・回るで同じ）。
        # rates（向きの名前 → 1 秒あたりに動く量。forward / backward / left / right / turn_left / turn_right）があれば、
        # 動く量は rates[向き] × 時間 × rate_ratio（rate_ratio はばらつきの倍率）。11/14 の実機では向きで大きく違った
        # （速さ 0.3 で前 0.21 m/s・左 0.105 m/s・右 0.23 m/s・回る 0.19 rad/s）
        self.rates = None if rates is None else {k: float(v) for k, v in rates.items()}
        self.rate_ratio = float(rate_ratio)
        self.tail_s = float(tail_s)           # 歩き終わりに止まるまでの時間
        self.tail_ratio = float(tail_ratio)   # 歩き終わりに余分に動く量（duration のあいだに動いた量に対する割合）
        self.heading_error = np.radians(float(heading_error_deg))  # 並進の向きのずれ（+ が左）
        self.noise = float(noise)             # 動いた量のばらつき（割合、標準偏差）
        self.rng = np.random.default_rng(seed)
        self.pose = np.zeros(3)
        self.t = 0.0
        self._busy_until = 0.0
        self.calls: list[tuple[float, float, float, float]] = []
        self.fsm = 501

    # 時計
    def sleep(self, dt: float) -> None:
        self.t += max(0.0, float(dt))

    def now(self) -> float:
        return self.t

    # LocoBackend
    def set_velocity(self, vx: float, vy: float, wz: float, duration_s: float) -> None:
        self.calls.append((vx, vy, wz, duration_s))
        v = np.array([vx, vy, wz], dtype=float)
        mins = np.array([self.min_speed[c] for c in COMPONENTS])
        v[np.abs(v) < mins] = 0.0  # 遅いと歩き出さない
        if not v.any():
            return
        k = self.rate_ratio * float(duration_s) * (1.0 + self.tail_ratio)
        if self.rates is not None:
            names = (("forward", "backward"), ("left", "right"), ("turn_left", "turn_right"))
            v = np.array([np.sign(x) * self.rates[names[i][0 if x > 0 else 1]] if x != 0 else 0.0 for i, x in enumerate(v)])
        d = v * k * (1.0 + self.noise * self.rng.standard_normal(3))
        # 並進は向きがずれる（胴体の座標で回してから、床の座標へ）
        c, s = np.cos(self.heading_error), np.sin(self.heading_error)
        body = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
        cy, sy = np.cos(self.pose[2]), np.sin(self.pose[2])
        self.pose[0] += cy * body[0] - sy * body[1]
        self.pose[1] += sy * body[0] + cy * body[1]
        self.pose[2] += d[2]
        self._busy_until = self.t + float(duration_s) + self.tail_s

    def stop(self) -> None:
        self._busy_until = min(self._busy_until, self.t + self.tail_s)

    def fsm_id(self) -> int | None:
        return self.fsm

    # Stillness
    def is_still(self) -> bool:
        return self.t >= self._busy_until
