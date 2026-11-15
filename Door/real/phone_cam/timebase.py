"""iPhone の動画の時刻 ↔ Mac の時計 ↔ PC2 の time.monotonic()（記録 door_stock_*.npz の t）を対応づける。numpy なし。

時計は 3 つある:
  電話の時計     撮影のページの performance.timeOrigin + performance.now()（ミリ秒）
  Mac の時計     time.time()。phone_server.py が答える /api/time と、PC2 の出力を受け取った時刻
  PC2 の時計     time.monotonic()。記録の t。PC2 の壁の時計は JST から約 1 時間ずれているので使わない

電話 → Mac: 撮影のページが /api/time を数回聞き、往復の一番短い 1 回の中点で差を取る（NTP と同じ考え）。
PC2 → Mac: PC2 の見張り役（pc2_feed_agent.py）が 1 行ごとに monotonic を付けて送る。受け取った Mac の時刻との差は
「時計の差 + 届くまでの遅れ」なので、遅れが一番小さかった時（差の最小）を時計の差とみなす（下側の包絡）。
動画の 0 秒: MediaRecorder の start の時刻（電話の時計）。カメラの遅れは Mac の画面の時計を写して 1 度だけ測る（calib）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

ENVELOPE_WINDOW_S = 120.0  # PC2 の時計の差は、この幅の中の最小で取る（monotonic と Mac の時計の歩みの差を無視できる幅）


@dataclass(frozen=True)
class Ping:
    """撮影のページの /api/time の 1 往復（どれも秒）"""
    send: float    # 電話の時計で聞いた時
    server: float  # Mac が答えた時刻
    recv: float    # 電話の時計で答えを受けた時

    @property
    def rtt(self) -> float:
        return self.recv - self.send

    @property
    def offset(self) -> float:
        """Mac − 電話"""
        return self.server - 0.5 * (self.send + self.recv)


def best_ping(pings: Iterable[Ping]) -> Ping:
    """往復が一番短い 1 回（遅れの片寄りが一番小さい）"""
    ok = [p for p in pings if p.rtt >= 0.0]
    if not ok:
        raise ValueError("使える往復が無い（電話と Mac の時計を合わせられない）")
    return min(ok, key=lambda p: p.rtt)


PING_NEAR_S = 60.0  # 動画の 0 秒を合わせる往復は、録り始めのこの前後から選ぶ（時計の歩みの差が 10 分で数十 ms 入るため）


def best_ping_near(pings: Sequence[Ping], at_phone: float, window: float = PING_NEAR_S) -> Ping:
    """電話の時刻 at_phone の前後 window 秒の往復のうち一番短い 1 回（無ければ全部から）"""
    near = [p for p in pings if abs(p.send - at_phone) <= window]
    return best_ping(near or pings)


def pings_from_ms(rows: Sequence[Sequence[float]]) -> List[Ping]:
    """撮影のページが送る [[send_ms, server_ms, recv_ms], ...] を秒に"""
    out = []
    for r in rows:
        if len(r) != 3:
            raise ValueError(f"往復の形が違う: {r!r}")
        out.append(Ping(float(r[0]) / 1000.0, float(r[1]) / 1000.0, float(r[2]) / 1000.0))
    return out


def mono_offset(samples: Sequence[Tuple[float, float]], at: Optional[float] = None,
                window: float = ENVELOPE_WINDOW_S) -> float:
    """PC2 の monotonic → Mac の時計の差（Mac − monotonic）。samples は (monotonic, Mac で受けた時刻)。
    at（Mac の時刻）を渡すと、その前後 window 秒の中の最小を使う（無ければ全部の最小）"""
    if not samples:
        raise ValueError("PC2 の時刻の見本が無い（見張り役がつながっていなかった）")
    near = [m - t for t, m in samples if at is None or abs(m - at) <= window]
    if not near:
        near = [m - t for t, m in samples]
    return min(near)


BUCKET_S = 10.0  # MonoClock は見本を Mac の時刻 10 秒ごとに分け、各箱の最小の差だけ持つ
SAME_BOOT_S = 1.0  # 差がこれ以上違う箱は別の起動（同じ起動なら差は届くまでの遅れ ms しか動かない）


class MonoClock:
    """feed_*.jsonl の見本（何時間ぶんでも・PC2 を起こし直していても）から、PC2 の monotonic ↔ Mac の時計を引く。
    起こし直すと monotonic が 0 に戻り差が大きく変わるので、「その差で直した Mac の時刻が、その箱の時刻の近くに来る」
    箱だけを使う（同じ起動の箱だけが自分と合う）"""

    def __init__(self, samples: Sequence[Tuple[float, float]], window: float = ENVELOPE_WINDOW_S) -> None:
        if not samples:
            raise ValueError("PC2 の時刻の見本が無い（見張り役がつながっていなかった）")
        b: dict = {}
        for mono, mac in samples:
            k, d = int(mac // BUCKET_S), mac - mono
            if k not in b or d < b[k]:
                b[k] = d
        self.buckets = sorted(b.items())
        self.window = window

    def _min_near(self, mac_t: float, ref: float) -> float:
        """mac_t の前後 window の箱のうち、差が ref と同じ起動（SAME_BOOT_S 以内）の最小"""
        near = [d for k, d in self.buckets
                if abs((k + 0.5) * BUCKET_S - mac_t) <= self.window + BUCKET_S and abs(d - ref) < SAME_BOOT_S]
        return min(near) if near else ref

    def offset_at(self, mac_t: float) -> float:
        """Mac の時刻 mac_t の頃の差（Mac − monotonic）"""
        ref = min(self.buckets, key=lambda kd: abs((kd[0] + 0.5) * BUCKET_S - mac_t))[1]
        return self._min_near(mac_t, ref)

    def mono_to_mac(self, mono: float, hint_mac: Optional[float] = None) -> float:
        """hint_mac: だいたいの Mac の時刻（動画の 0 秒など）。同じ日に PC2 を起こし直していると、同じ monotonic が
        2 つの起動のどちらにも合うことがあるので、hint に近い方を選ぶ（無ければ一番よく合う箱）"""
        lim = self.window + BUCKET_S
        fit = [(abs(mono + d - (k + 0.5) * BUCKET_S), d) for k, d in self.buckets]
        ok = [(err, d) for err, d in fit if err <= lim]
        if not ok:
            raise ValueError(f"PC2 の時刻 {mono:.1f} に合う見本が無い（その時に見張り役がつながっていなかった）")
        if hint_mac is None:
            ref = min(ok)[1]
        else:
            ref = min(ok, key=lambda ed: abs(mono + ed[1] - hint_mac))[1]
        return mono + self._min_near(mono + ref, ref)


@dataclass(frozen=True)
class VideoClock:
    """動画の時刻（0 秒＝最初の画）↔ Mac の時計"""
    start_mac: float  # 動画の 0 秒の Mac の時刻
    duration: float = float("nan")

    def to_mac(self, video_t: float) -> float:
        return self.start_mac + video_t

    def to_video(self, mac_t: float) -> float:
        return mac_t - self.start_mac

    def contains(self, mac_t: float, margin: float = 0.0) -> bool:
        v = self.to_video(mac_t)
        return -margin <= v <= (self.duration if self.duration == self.duration else float("inf")) + margin


def video_clock(start_phone_s: float, pings: Sequence[Ping], calib_s: float = 0.0,
                duration: float = float("nan")) -> VideoClock:
    """start_phone_s: MediaRecorder が始まった時（電話の時計・秒）。
    calib_s: 写った Mac の画面の時計 − 合わせる前の見積もり（Mac の画面の時計を写して測る。負なら画が見積もりより古い）"""
    p = best_ping_near(pings, start_phone_s)
    return VideoClock(start_phone_s + p.offset + calib_s, duration)
