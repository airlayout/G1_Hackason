"""GPU・実機なしでUIと通信を確かめる。VLA推論は行わず、現在角度を保持する。"""

from __future__ import annotations

import io
import time

import numpy as np
from PIL import Image, ImageDraw

from .robot import RobotBridge


class DemoBridge(RobotBridge):
    def __init__(self, config: dict):
        super().__init__(config, execute=False)
        self.start_thread(self.update, "demo-observation")
        self.start_thread(self.run_control, "demo-control")

    def update(self) -> None:
        image = Image.new("RGB", (640, 480), "#14191e")
        draw = ImageDraw.Draw(image)
        for x in range(0, 640, 40):
            draw.line((x, 0, x, 480), fill="#242c33")
        for y in range(0, 480, 40):
            draw.line((0, y, 640, y), fill="#242c33")
        draw.rectangle((270, 160, 370, 320), outline="#8fa5a5", width=2)
        draw.ellipse((306, 210, 334, 238), fill="#d3755f")
        draw.text((24, 24), "DEMO / NO ROBOT / NO VLA", fill="#b8c9c9")
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG")
        q = np.zeros(29)
        q[16], q[23] = 0.2, -0.2
        while not self.done.is_set():
            with self.lock:
                self.jpeg = buffer.getvalue()
                now = time.monotonic()
                self.control.update_state(q, np.zeros(29), now=now)
                self.control.camera_at = now
            self.done.wait(0.02)


class HoldPolicy:
    def load(self) -> None:
        pass

    def predict(self, observation: dict) -> np.ndarray:
        if "observation.state" in observation:
            q = observation["observation.state"][15:29]
        else:
            state = observation["state"]
            q = np.concatenate([state["left_arm"][0, 0], state["right_arm"][0, 0]])
        return np.repeat(q[None], 8, axis=0)

    def metrics(self) -> dict:
        return {"loaded": False, "demo": True}
