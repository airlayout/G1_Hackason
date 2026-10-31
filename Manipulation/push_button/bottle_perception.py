"""Single-owner RGB-D processing; control reads completed immutable results only.

The same engine runs in a real worker thread or on the simulator event queue.
Neither camera waits nor inference hold the mailbox lock used by control.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import threading
import time
import os

import numpy as np

from bottle import BottleLocalizer
from trial_policy import ObservationUnavailable, TrialPause


@dataclass(frozen=True)
class TableEstimate:
    captured_at: float
    offset: tuple
    velocity: tuple

    def predict(self, now, *, strict=True):
        age = now-self.captured_at
        if not 0 <= age <= .5:
            if strict:
                raise ObservationUnavailable('新しい台基準の機体位置を取得できません')
            return np.array(self.offset)
        return np.array(self.offset)+np.clip(np.array(self.velocity)*min(.15, age),
                                              [-.003, -.003, -.0005], [.003, .003, .0005])


class PerceptionEngine:
    """Nonblocking input, at most 10Hz geometry and 5Hz bottle detection."""

    def __init__(self, read_frame, detector, settings, now, target_pixel=None, *, table_snapshot=None, close=None,
                 position_stability_m=.004, recoverable=False):
        self.read_frame, self.now = read_frame, now
        self.localizer = BottleLocalizer(None, detector, settings, now, target_pixel,
                                        position_stability_m=position_stability_m)
        self.recoverable = recoverable
        self.table_snapshot, self.close_reader = table_snapshot, close
        self.mode = 'idle'
        self.next_frame = self.next_detection = -np.inf
        self.processing_times = []
        self.rejections = Counter()

    def configure(self, request):
        mode, after, reference, expected, after_push = request
        self.mode = mode
        loc = self.localizer
        loc.previous, loc.expected_xy, loc.after_push = reference, expected, after_push
        loc.begin(after, tracking=mode == 'track')
        self.next_frame = self.next_detection = -np.inf

    def step(self, progress=0.):
        stamp = self.now()
        if stamp < self.next_frame:
            return None, None
        self.next_frame = stamp+.1
        observation = None
        started = time.perf_counter()
        try:
            frame = self.read_frame()
            if frame is not None and self.mode != 'idle' and stamp >= self.next_detection:
                self.next_detection = stamp+.2
                self.localizer.progress_m = progress
                observation = self.localizer.consume(frame)
        except ValueError as exc:
            # Geometry/pose failures are temporary; last good results expire normally.
            if '一意' in str(exc) and not self.recoverable:
                raise
            self.rejections[str(exc)] += 1
            self.localizer.samples.clear()
        finally:
            self.processing_times.append({'stage': 'perception', 'time_s': stamp,
                'duration_s': time.perf_counter()-started, 'load_average': list(os.getloadavg())})
        table = self.table_snapshot() if self.table_snapshot is not None else None
        return observation, table

    def close(self):
        if self.close_reader is not None:
            self.close_reader()


class PerceptionMailbox:
    """Control-facing API, also shared by the virtual scheduler adapter."""

    def __init__(self, engine, now, wait):
        self.engine, self.now, self.wait = engine, now, wait
        self.lock = threading.Lock()
        self.generation = 0
        self.pending = None
        self.progress = 0.
        self.latest = self.table = self.error = None
        self.reference = self.anchor = None
        self.after_push = False
        self.mode = 'idle'

    def request(self, mode):
        after = self.now()
        expected = None
        reference = self.anchor if self.after_push or mode == 'track' else self.reference
        if self.after_push and reference is not None:
            expected = np.array(reference.center_xy)+[self.engine.localizer.settings.push_distance, 0.]
        with self.lock:
            self.generation += 1
            self.mode = mode
            self.pending = (mode, after, reference, expected, self.after_push)
            self.latest = None
        return after

    def take_request(self):
        with self.lock:
            request, self.pending = self.pending, None
            return self.generation, request, self.progress

    def publish(self, generation, observation, table, error=None):
        with self.lock:
            if error is not None:
                self.error = error
            if generation != self.generation:
                return  # A completed job from a previous phase cannot satisfy a new measurement.
            if observation is not None:
                self.latest = observation
            if table is not None:
                self.table = table

    def check(self):
        with self.lock:
            error = self.error
        if error is not None:
            raise RuntimeError(f'画像処理で中止: {error}') from error

    def _await_observation(self, after, timeout_s):
        deadline = after+timeout_s
        while self.now() < deadline:
            self.check()
            with self.lock:
                result = self.latest
            if result is not None and result.captured_at > after and 0 <= self.now()-result.captured_at <= .5:
                self.reference = result.target
                return result
            self.wait(.01)
        self.engine.localizer.rejections['安定計測の期限超過'] += 1
        error = TrialPause if getattr(self.engine, 'recoverable', False) else RuntimeError
        raise error('安定した新しい3フレームのボトル計測を取得できません（対象消失・深度・鮮度を確認）')

    def observe(self, timeout_s=5.):
        return self._await_observation(self.request('measure'), timeout_s)

    def start_tracking(self, observation, timeout_s=5.):
        self.anchor = observation.target
        self.progress = 0.
        return self._await_observation(self.request('track'), timeout_s)

    def latest_tracking(self, progress_m):
        self.check()
        with self.lock:
            self.progress = float(np.clip(progress_m, 0., self.engine.localizer.settings.push_distance))
            result = self.latest
        if result is None or not 0 <= self.now()-result.captured_at <= .75:
            raise ObservationUnavailable('押下中のボトル追跡が古すぎる、または対象を確認できません')
        return result

    def stop_tracking(self):
        self.request('idle')

    def reset_measurement_reference(self):
        """現場で置き直した対象を新しい3枚で選ぶ。異常は解除しない。"""
        with self.lock:
            self.generation += 1
            self.reference = self.anchor = self.latest = None
            self.progress, self.after_push, self.mode = 0., False, 'idle'
            self.pending = ('idle', self.now(), None, None, False)

    def expect_translation(self):
        if self.anchor is None:
            self.anchor = self.reference
        self.after_push = True

    def table_offset(self, *, strict=True):
        with self.lock:
            estimate = self.table
        if estimate is None:
            if strict:
                raise ObservationUnavailable('新しい台基準の機体位置を取得できません')
            return np.zeros(3)
        return estimate.predict(self.now(), strict=strict)

    @property
    def measurements(self):
        return list(self.engine.localizer.measurements)

    @property
    def rejections(self):
        return self.engine.localizer.rejections.copy()+self.engine.rejections.copy()

    @property
    def diagnostics(self):
        return list(self.engine.localizer.diagnostics)


class ThreadedPerception(PerceptionMailbox):
    def __init__(self, engine, now, wait):
        super().__init__(engine, now, wait)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name='bottle-perception', daemon=True)
        self.thread.start()

    def _run(self):
        generation = 0
        try:
            while not self.stop.is_set():
                generation, request, progress = self.take_request()
                if request is not None:
                    self.engine.configure(request)
                observation, table = self.engine.step(progress)
                self.publish(generation, observation, table)
                self.stop.wait(.005)
        except Exception as exc:
            self.publish(generation, None, None, exc)
        finally:
            try:
                self.engine.close()
            except Exception as exc:
                self.publish(generation, None, None, exc)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            raise RuntimeError('画像処理ワーカーが終了しません')
        self.check()


def make_perception_worker(engine, now, wait):
    return ThreadedPerception(engine, now, wait)
