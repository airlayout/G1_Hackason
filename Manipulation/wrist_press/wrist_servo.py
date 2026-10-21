"""Rolling wrist observations in pelvis coordinates; no simulator inputs."""
from collections import deque
from dataclasses import replace

import numpy as np


class WristTargetFilter:
    def __init__(self, count=3, spread_m=.003, max_age_s=.35):
        self.items = deque(maxlen=count)
        self.spread_m, self.max_age_s = spread_m, max_age_s
        self.last_t = -float("inf")
        self.latest = None
        self.reason = "collecting wrist images"

    def reject(self, reason):
        self.items.clear()
        self.latest = None
        self.reason = reason

    def fresh(self, now):
        if any(not -.02-1e-9 <= now-t.captured_at <= self.max_age_s+1e-9 for t in self.items):
            self.reject("stale wrist images")
        return self.latest is not None

    def add(self, target, now, reference):
        if target.captured_at <= self.last_t + 1e-6:
            return None  # A duplicate is neither a new observation nor a loss.
        self.last_t = target.captured_at
        if target.camera != "wrist" or target.kind != reference.kind or target.name != reference.name:
            self.reject("wrong wrist target")
            return None
        if not -.02-1e-9 <= now-target.captured_at <= self.max_age_s+1e-9:
            self.reject("stale wrist image")
            return None
        if (np.linalg.norm(target.point-reference.point) >= .015 or
                target.normal @ reference.normal <= np.cos(np.radians(8))):
            self.reject("wrist target outside reference corridor")
            return None
        self.fresh(now)
        self.items.append(target)
        self.latest = None
        self.reason = "collecting wrist images"
        if len(self.items) != self.items.maxlen:
            return None
        points = np.array([t.point for t in self.items])
        if np.max(np.linalg.norm(points[:, None]-points[None, :], axis=2)) > self.spread_m:
            self.reason = "wrist positions disagree"
            return None
        median = np.median(points, axis=0)
        # A changes only the tangent position. Surface normal/depth/radius remain
        # the initial head estimate; surface re-estimation belongs to proposal B.
        delta = median-reference.point
        delta -= (delta @ reference.normal)*reference.normal
        self.latest = replace(target, point=reference.point+delta,
                              normal=reference.normal.copy(), radius=reference.radius,
                              wall_point=None if reference.wall_point is None else reference.wall_point.copy())
        self.reason = "stable wrist target"
        return self.latest


def tangent_error(target, tip):
    error = target.point-tip
    return error-(error @ target.normal)*target.normal


def bounded_correction(error, gain=.5, max_step_m=.003):
    step = gain*np.asarray(error, dtype=float)
    length = np.linalg.norm(step)
    return step*(min(1., max_step_m/length) if length > 0 else 1.)
