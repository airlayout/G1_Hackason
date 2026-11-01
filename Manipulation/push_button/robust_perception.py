"""Continuous, single-owner perception. Initial three frames, then a nonblocking mailbox."""
from collections import deque, Counter
from dataclasses import dataclass
import threading
import time

import numpy as np

from robust_scene import SceneSnapshot


@dataclass(frozen=True)
class Observation:
    geometry: dict
    q: tuple
    imu: tuple
    captured_at: float
    frame_id: int
    lamp: float | None = None
    frame_ids: tuple = ()


class LatestFrameSampler:
    """Rate-limit processing without discarding a frame arriving just before a tick."""
    def __init__(self, period_s=.2):
        self.period_s = period_s
        self.last_sample = -np.inf
        self.pending = None

    def consume(self, delivered, now):
        if delivered is not None:
            self.pending = delivered
        if self.pending is None or now-self.last_sample < self.period_s:
            return None
        result, self.pending = self.pending, None
        self.last_sample = now
        return result


class ObservationWindow:
    def __init__(self, stability_m=.004):
        self.stability_m = stability_m
        self.samples = deque(maxlen=3)
        self.last_id, self.last_stamp = -1, -np.inf
        self.reference = None

    def consume(self, value, now):
        if (value.frame_id <= self.last_id or value.captured_at <= self.last_stamp
                or not 0 <= now-value.captured_at <= .5):
            self.samples.clear()
            raise ValueError('Stale or duplicate perception frame')
        self.last_id, self.last_stamp = value.frame_id, value.captured_at
        if self.samples and value.captured_at-self.samples[-1].captured_at > .5:
            self.samples.clear()  # a disappearance must not reuse pre-loss stability samples
        g = value.geometry
        if self.reference is not None:
            if (np.linalg.norm(np.array(g['face'])-self.reference['face']) > .025
                    or abs(g['radius']-self.reference['radius']) > .005):
                self.samples.clear()
                raise ValueError('Target identity changed')
        self.samples.append(value)
        if len(self.samples) < 3:
            return None
        points = np.array([v.geometry['face'] for v in self.samples])
        centre = np.median(points, axis=0)
        if np.max(np.linalg.norm(points-centre, axis=1)) > self.stability_m:
            return None
        radii = [v.geometry['radius'] for v in self.samples]
        if np.ptp(radii) > .003:
            return None
        geometry = dict(g, face=tuple(centre), radius=float(np.median(radii)))
        if 'base_height' in g:
            heights = [v.geometry['base_height'] for v in self.samples]
            if np.ptp(heights) > .008:
                return None
            geometry['base_height'] = float(np.median(heights))
        self.reference = geometry
        return Observation(geometry, value.q, value.imu, value.captured_at, value.frame_id, value.lamp,
                           tuple(v.frame_id for v in self.samples))


class ContinuousObserver:
    """read() -> Observation or None. The camera and its FK model have one owner."""
    def __init__(self, read, close=lambda: None, *, now=time.monotonic, stability_m=.004):
        self.read, self.close_reader, self.now = read, close, now
        self.window = ObservationWindow(stability_m)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.value = self.error = None
        self.frames, self.rejections = [], Counter()
        self.thread = threading.Thread(target=self._run, name='robust-perception', daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stop.is_set():
                value = self.read()
                if value is None:
                    self.stop.wait(.005)
                    continue
                try:
                    completed = self.window.consume(value, self.now())
                except ValueError as exc:
                    self.rejections[str(exc)] += 1
                    continue
                if completed is not None:
                    with self.lock:
                        self.value = completed
                    self.frames.append(dict(frame_id=value.frame_id, captured_at_s=value.captured_at,
                                            frame_ids=completed.frame_ids,geometry=completed.geometry,lamp=value.lamp))
        except Exception as exc:
            self.error = exc
        finally:
            self.close_reader()

    def latest(self):
        if self.error is not None:
            raise RuntimeError(f'Perception stopped: {self.error}') from self.error
        with self.lock:
            value = self.value
        if value is None or not 0 <= self.now()-value.captured_at <= .5:
            raise RuntimeError('No fresh continuous observation')
        return value

    def initial(self, timeout_s=5.):
        deadline = self.now()+timeout_s
        while self.now() < deadline:
            if self.error is not None:
                raise RuntimeError(str(self.error)) from self.error
            try:
                return self.latest()
            except RuntimeError:
                self.stop.wait(.01)
        raise RuntimeError('Initial three-frame observation unavailable')

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            raise RuntimeError('Perception worker did not stop')


def snapshot(task, observed, calibration, args):
    return SceneSnapshot(task=task, q=observed.q, imu=observed.imu,
        captured_at=observed.captured_at, tip_radius=calibration['tip_radius_m'],
        frame_ids=observed.frame_ids,
        uncertainty_m=calibration['position_uncertainty_m'],
        stroke=args.stroke, push_distance=args.push_distance, **observed.geometry)


class CorrectionGate:
    """Ignore measurement jitter. Never accumulate unbounded retargeting."""
    def __init__(self, scene):
        self.scene = scene
        self.offset = np.zeros(3)
        self.last_id = -1

    def update(self, observed, progress, *, freeze=False):
        if observed.frame_id == self.last_id:
            return self.offset.copy()
        self.last_id = observed.frame_id
        normal = np.array(self.scene.normal)
        expected = np.array(self.scene.face)+normal*progress
        error = np.array(observed.geometry['face'])-expected
        if self.scene.task == 'bottle' and abs(observed.geometry.get('object_height',self.scene.object_height)-self.scene.object_height) > .01:
            raise RuntimeError('Bottle vertical extent changed')
        if abs(error@normal) > .010:
            raise RuntimeError('Target changed in the press direction')
        if np.array(observed.geometry['normal'])@normal < np.cos(np.deg2rad(3)):
            raise RuntimeError('Observed target normal changed')
        # Forward motion/contact depth remains controlled by the planned reference.
        lateral = error-normal*(error@normal)
        limit = min(.005, self.scene.radius-self.scene.tip_radius-self.scene.uncertainty_m-.003)
        if np.linalg.norm(lateral) > limit:
            raise RuntimeError('Observed target shift exceeds certified correction envelope')
        if freeze:
            return self.offset.copy()
        if np.linalg.norm(lateral) > max(.001, self.scene.uncertainty_m):
            # Bound the increment; command filtering also enforces joint acceleration.
            delta = lateral-self.offset
            self.offset += delta*min(1., .0005/max(np.linalg.norm(delta), 1e-12))
        return self.offset.copy()
