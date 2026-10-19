"""Adapt Yada observations; interpolate robot state at image exposure time."""
from __future__ import annotations

from collections import deque
import numpy as np

from .types import CameraFrame, Sample


class SampleBuilder:
    def __init__(self, robot, sim="mujoco"):
        self.robot, self.sim = robot, sim
        self.history = deque(maxlen=100)

    def at(self, stamp):
        if not self.history or stamp < self.history[0][0]-0.005 or stamp > self.history[-1][0]+0.005:
            return None
        for (ta, qa), (tb, qb) in zip(self.history, list(self.history)[1:]):
            if ta <= stamp <= tb:
                return qa + (qb-qa)*(stamp-ta)/max(tb-ta, 1e-9)
        return self.history[-1][1].copy()

    def build(self, obs):
        t = float(obs.t)
        self.history.append((t, obs.q.copy()))
        # Old Isaac adapter has no exposure timestamp; its image is read synchronously.
        stamp = t if self.sim == "isaac" else float(getattr(obs, "image_t", t))
        q_image = self.at(stamp)
        frame = None
        if q_image is not None:
            frame = CameraFrame(obs.rgb, obs.depth, obs.K, stamp, q_image,
                                self.robot.head_transform(q_image), sequence=int(round(stamp*1000000)))
        wrist = getattr(obs, "wrist", None)
        if wrist is not None and not isinstance(wrist, CameraFrame):
            raise ValueError("wrist observation must be a calibrated CameraFrame")
        return Sample(obs.q, obs.dq, t, frame, wrist)
