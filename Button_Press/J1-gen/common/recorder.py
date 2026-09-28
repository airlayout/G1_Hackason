"""収録の進め方（どのフレームを保存するか、lowstate の記録）。タスク5。

- フレーム: 取得元（深度付き、または RGB だけ）から受信し、保存のしかた（mode）に従って保存する
  - all: 受信したフレームをすべて保存する
  - interval: interval_s 秒に 1 枚だけ保存する（間引き保存。ボタン撮影用）
  - enter: Enter が押されたときだけ 1 枚保存する（ボタン撮影で、構図を決めてから撮るとき）
- lowstate: 別のスレッドで lowstate_hz ごとに最新の値を確かめ、新しければ 1 行書く
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Protocol

import numpy as np

from .arm.types import JointState
from .recording import RecordingWriter
from .rgbd_protocol import RgbdFrame

# 取得元: 呼ぶと (カラー BGR, RgbdFrame か None) を返す。届かなければ None
FrameGetter = Callable[[], "tuple[np.ndarray, RgbdFrame | None] | None"]


class StateSource(Protocol):
    def latest(self) -> JointState | None: ...


class Recorder:
    def __init__(
        self,
        writer: RecordingWriter,
        get_frame: FrameGetter,
        lowstate: StateSource | None,
        mode: str = "all",
        interval_s: float = 1.0,
        lowstate_hz: float = 100.0,
        enter_trigger: Callable[[], bool] | None = None,
    ) -> None:
        if mode not in ("all", "interval", "enter"):
            raise ValueError(f"mode は all / interval / enter: {mode}")
        self.writer = writer
        self.get_frame = get_frame
        self.lowstate = lowstate
        self.mode = mode
        self.interval_s = float(interval_s)
        self.lowstate_hz = float(lowstate_hz)
        # enter のとき: 呼ぶと「保存の合図が来ていれば True（合図は消す）」を返す
        self.enter_trigger = enter_trigger
        self._stop = threading.Event()
        self.n_received = 0
        self.n_timeouts = 0

    def stop(self) -> None:
        self._stop.set()

    def _lowstate_loop(self) -> None:
        assert self.lowstate is not None
        last_stamp = None
        period = 1.0 / self.lowstate_hz
        while not self._stop.is_set():
            st = self.lowstate.latest()
            if st is not None and st.stamp != last_stamp:
                self.writer.add_lowstate(time.time(), st)
                last_stamp = st.stamp
            time.sleep(period)

    def _should_save(self, now: float, last_saved: float | None) -> bool:
        if self.mode == "all":
            return True
        if self.mode == "interval":
            return last_saved is None or now - last_saved >= self.interval_s
        return bool(self.enter_trigger and self.enter_trigger())

    def run(self, max_frames: int | None = None, duration_s: float | None = None,
            on_saved: Callable[[int, float], None] | None = None) -> int:
        """止められる（stop()、max_frames、duration_s）まで収録する。保存したフレーム数を返す。"""
        th = None
        if self.lowstate is not None:
            th = threading.Thread(target=self._lowstate_loop, daemon=True)
            th.start()
        t0 = time.monotonic()
        last_saved: float | None = None
        try:
            while not self._stop.is_set():
                if max_frames is not None and self.writer.n_frames >= max_frames:
                    break
                if duration_s is not None and time.monotonic() - t0 >= duration_s:
                    break
                got = self.get_frame()
                if got is None:
                    self.n_timeouts += 1
                    continue
                self.n_received += 1
                now = time.monotonic()
                if not self._should_save(now, last_saved):
                    continue
                color, rgbd = got
                state = self.lowstate.latest() if self.lowstate is not None else None
                i = self.writer.add_frame(color, None if rgbd is None else rgbd.depth, time.time(), rgbd, state)
                last_saved = now
                if on_saved:
                    on_saved(i, now - t0)
        finally:
            self._stop.set()
            if th is not None:
                th.join(timeout=2.0)
        return self.writer.n_frames
