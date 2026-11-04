#!/usr/bin/env python3
"""[G1_Hackason 改変] 記録の歩きの指令を純正の歩行へ流し直す部品（replay_arm.py --legs が使う。2026-11-03）。

記録（data.json）の actions.body.qpos は、遠隔操作が機体へ実際に送った歩きの指令 [前後 vx, 横 vy, 旋回 vyaw]
（修正 record-loco-cmd。m/s・rad/s。一時停止中と足なし（no-loco）では 0）。これを腕と同じ時刻で純正へ流す。

送り方は Door/real/door_stock_io.py の VelocitySender と同じ教訓に従う:
  - Move()・StopMove() は SetVelocity の戻り値を捨てる（拒否が見えない）ので、SetVelocity を直接呼ぶ
  - 制御の輪（50 Hz）は RPC の応答を待たない。別の糸から 20 回/秒
  - 1 回の SetVelocity が効くのは 0.5 秒。送り手が止まっても純正は 0.5 秒で止まる。止めるときは 0 を 3 回
  - 歩ける FSM は 500・501 だけ（802 の立位ロックでは指令が通っても足が動かない）
⚠️ 歩きは開ループ。同じ指令でも同じ所へ着くとは限らない（10-29 の扉の前では前への指令に 12% しか進まなかった）。
"""

import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import numpy as np

# 遠隔操作が送りうる範囲（xr_teleoperate のスティック × 0.3。前だけは修正 loco-fwd-max で 1.0 まで）
VX_FWD_MAX = 1.0
V_OTHER_MAX = 0.3
RANGE_TOL = 1e-3
WALK_EPS = 1e-3            # これ以下の指令は「歩いていない」
WALKING_FSM_IDS = (500, 501)
SEND_RATE_HZ = 20.0
VEL_DURATION_S = 0.5
STOP_REPEATS = 3
FAIL_TRIP = 5              # SetVelocity が続けてこれだけ失敗したら再生を止める（20 回/秒で 0.25 秒）
# RPC の応答を待つ上限。糸は 1 本なので、詰まると次の送信（止める 0 も）が待たされる。
# 有線の機体の中では数 ms で返るので短くし、FAIL_TRIP 回の失敗で止まるまでを 5 × 0.3 = 1.5 秒以内にする
RPC_TIMEOUT_S = 0.3


def load_loco(episode_dir: Path) -> Optional[np.ndarray]:
    """記録の歩きの指令を (コマ数, 3) で返す。歩きの欄が無い記録（手ありの分岐・古い記録）は None。"""
    with open(Path(episode_dir) / "data.json", encoding="utf-8") as f:
        frames = json.load(f).get("data", [])
    rows = []
    for fr in frames:
        v = ((fr.get("actions") or {}).get("body") or {}).get("qpos")
        if not isinstance(v, list) or len(v) != 3:
            return None
        rows.append(v)
    return np.asarray(rows, dtype=np.float64) if rows else None


def check_loco(cmd: np.ndarray) -> None:
    """遠隔操作が送りえない値（壊れた記録）なら、何も送らずに止める。"""
    if not np.all(np.isfinite(cmd)):
        sys.exit("エラー: 歩きの指令に NaN / inf があります（記録の破損）。実機には送りません。")
    vx, vy, vyaw = cmd[:, 0], cmd[:, 1], cmd[:, 2]
    bad = []
    if vx.max() > VX_FWD_MAX + RANGE_TOL:
        bad.append(f"前 {vx.max():.3f} m/s > {VX_FWD_MAX}")
    if vx.min() < -V_OTHER_MAX - RANGE_TOL:
        bad.append(f"後ろ {vx.min():.3f} m/s < -{V_OTHER_MAX}")
    if np.abs(vy).max() > V_OTHER_MAX + RANGE_TOL:
        bad.append(f"横 {np.abs(vy).max():.3f} m/s > {V_OTHER_MAX}")
    if np.abs(vyaw).max() > V_OTHER_MAX + RANGE_TOL:
        bad.append(f"旋回 {np.abs(vyaw).max():.3f} rad/s > {V_OTHER_MAX}")
    if bad:
        sys.exit("エラー: 歩きの指令が遠隔操作の送りうる範囲を超えています（" + "・".join(bad) + "）。実機には送りません。")


def walking_mask(cmd: np.ndarray) -> np.ndarray:
    return np.abs(cmd).max(axis=1) > WALK_EPS


def dead_reckon(cmd: np.ndarray, dt: np.ndarray) -> Tuple[float, float, float]:
    """指令どおりに歩いたときの (前 [m], 左 [m], 向き [rad])。始めの向きを基準にする。dt は各コマの長さ [s]。"""
    x = y = yaw = 0.0
    for (vx, vy, vyaw), h in zip(cmd, dt):
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * h
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * h
        yaw += vyaw * h
    return x, y, yaw


def walking_seconds(cmd: np.ndarray, times: np.ndarray) -> float:
    """歩いていたコマの長さの合計 [s]。各コマは次のコマまで効いていたとみなす（最後のコマは 0 秒）。"""
    dt = np.diff(times, append=times[-1])
    return float(dt[walking_mask(cmd)].sum())


def summarize_loco(cmd: np.ndarray, times: np.ndarray) -> List[str]:
    """歩きの中身を人が読む行にする。times は各コマの経過秒（先頭 0）。"""
    dt = np.diff(times, append=times[-1])
    walk = walking_mask(cmd)
    x, y, yaw = dead_reckon(cmd, dt)
    return [
        f"  歩き       : {walking_seconds(cmd, times):.1f} 秒 / {times[-1]:.1f} 秒（{walk.sum()} コマ）",
        f"  指令の最大 : 前 {max(cmd[:, 0].max(), 0):.2f}・後ろ {max(-cmd[:, 0].min(), 0):.2f} m/s・"
        f"横 {np.abs(cmd[:, 1]).max():.2f} m/s・旋回 {np.abs(cmd[:, 2]).max():.2f} rad/s",
        f"  指令どおりなら: 前へ {x:+.2f} m・左へ {y:+.2f} m・向き {math.degrees(yaw):+.0f}°"
        "（開ループ。実際はこれより短いことが多い）",
    ]


def make_loco_client(rpc_timeout_s: float = RPC_TIMEOUT_S):
    """純正の歩行の RPC の口。ChannelFactoryInitialize の後に呼ぶ。"""
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    client = LocoClient()
    client.SetTimeout(rpc_timeout_s)
    client.Init()
    return client


def fsm_id(client) -> Optional[int]:
    """今の FSM。読めなければ None。"""
    try:
        code, fsm = client.GetFsmId()
    except Exception:  # noqa: BLE001  読めないことを None で返す
        return None
    return int(fsm) if code == 0 and fsm is not None else None


class LocoSender:
    """歩きの指令を 20 回/秒で純正へ送る糸。再生の輪は set()・stop() で置くだけ（RPC を待たない）。

    歩く間は SetVelocity(vx, vy, vyaw, 0.5 秒)、歩くのをやめた・止めたときは 0 を STOP_REPEATS 回、
    それ以外は何も送らない（純正は最後の指令が切れて立つ）。
    """

    def __init__(self, client, rate_hz: float = SEND_RATE_HZ, duration_s: float = VEL_DURATION_S,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._client = client
        self._period = 1.0 / rate_hz
        self._duration = duration_s
        self._clock = clock
        self._lock = threading.Lock()
        self._want: Optional[Tuple[float, float, float]] = None
        self._stop_left = 0
        self._inflight = False     # SetVelocity の応答を待っている間は真（close が送り切りを待つ）
        self._failed = 0
        self._sent = 0
        self._last_code: Optional[int] = None
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._run, name="loco-replay", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def set(self, vx: float, vy: float, vyaw: float) -> None:
        with self._lock:
            if max(abs(vx), abs(vy), abs(vyaw)) > WALK_EPS:
                self._want = (float(vx), float(vy), float(vyaw))
            elif self._want is not None:
                self._want = None
                self._stop_left = STOP_REPEATS

    def stop(self) -> None:
        """歩いていたかに関係なく 0 を送り直す（止める側は念のため多めに）。"""
        with self._lock:
            self._want = None
            self._stop_left = STOP_REPEATS

    def status(self) -> Tuple[int, int, Optional[int]]:
        """(送った数, 続けて失敗した数, 最後の戻り値)"""
        with self._lock:
            return self._sent, self._failed, self._last_code

    def close(self, timeout_s: float = 1.0) -> None:
        """0 を送り切ってから糸を終える（最後の 0 の応答を待つ間も含めて待つ）。"""
        self.stop()
        t_end = self._clock() + timeout_s
        while self._clock() < t_end and self._thread.is_alive():
            with self._lock:
                if self._stop_left == 0 and not self._inflight:
                    break
            time.sleep(0.01)
        self._quit.set()
        if self._thread.is_alive():
            self._thread.join(timeout_s)

    def _next(self) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            v = None
            if self._want is not None:
                v = self._want
            elif self._stop_left > 0:
                self._stop_left -= 1
                v = (0.0, 0.0, 0.0)
            self._inflight = v is not None
            return v

    def _run(self) -> None:
        next_t = self._clock()
        while not self._quit.is_set():
            v = self._next()
            if v is not None:
                try:
                    code = self._client.SetVelocity(*v, self._duration)
                except Exception:  # noqa: BLE001  糸を止めない。失敗は数えて再生の側が止める
                    code = -1
                with self._lock:
                    self._inflight = False
                    self._sent += 1
                    self._failed = 0 if code == 0 else self._failed + 1
                    self._last_code = code
            next_t += self._period
            slack = next_t - self._clock()
            if slack > 0:
                self._quit.wait(slack)
            else:
                next_t = self._clock()
