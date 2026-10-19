"""Record actual MuJoCo motion beside the agent's head and wrist RGB images."""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np


class SynchronizedRecorder:
    """Read-only rendering hook; it never changes physics or controller inputs."""

    def __init__(self, robot, path, fps=10):
        import mujoco

        self.robot = robot
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self.renderer = mujoco.Renderer(robot.model, height=480, width=640)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [.17, -.18, 1.02]
        self.camera.distance = 1.25
        self.camera.azimuth = 70
        self.camera.elevation = -15
        self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"VP90"), fps, (960, 592))
        if not self.writer.isOpened():
            self.renderer.close()
            raise RuntimeError("OpenCV needs a VP9/WebM encoder to record this demo")
        self.frames = []
        self.next_t = 0.0
        self.peak_force = 0.0
        self.contact_seen = False
        self.poster_force = -1.0

    @staticmethod
    def label(image, text, x, y, scale=.55, color=(240, 244, 248)):
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, 1, cv2.LINE_AA)

    def capture(self, obs, command):
        # Track physical contacts at the control rate, including between video frames.
        force = float(getattr(self.robot, "bottle_force", 0.0))
        self.peak_force = max(self.peak_force, force)
        self.contact_seen |= bool(getattr(self.robot, "bottle_contact", False))
        if obs.t + 1e-6 < self.next_t and not command.done:
            return
        self.next_t = obs.t + 1 / self.fps
        self.renderer.update_scene(self.robot.data, camera=self.camera)
        canvas = np.full((592, 960, 3), (20, 25, 33), dtype=np.uint8)
        canvas[44:524, :640] = self.renderer.render()
        canvas[44:284, 640:] = cv2.resize(obs.rgb, (320, 240), interpolation=cv2.INTER_AREA)
        wrist = getattr(obs, "wrist", None)
        if wrist is not None:
            canvas[284:524, 640:] = cv2.resize(wrist.rgb, (320, 240), interpolation=cv2.INTER_AREA)
        self.label(canvas, "G1 / FIXED BOTTLE PUSH", 16, 29, .7)
        self.label(canvas, f"SIM {obs.t:5.2f} s   /   1x", 680, 29, .6)
        for x, y, title in [(0, 44, "MUJOCO / OVERVIEW"), (640, 44, "HEAD RGB / 10 Hz"),
                            (640, 284, "WRIST RGB / 10 Hz" if wrist is not None else "WRIST / UNAVAILABLE")]:
            cv2.rectangle(canvas, (x, y), (x + (640 if x == 0 else 320), y + 30), (20, 25, 33), -1)
            self.label(canvas, title, x + 12, y + 21)
        self.label(canvas, f"PHASE  {command.phase.upper()}", 16, 550, .65)
        contact = "Middle finger contact detected" if self.contact_seen else "Approaching bottle"
        self.label(canvas, contact, 320, 550, .6,
                   (115, 235, 175) if self.contact_seen else (240, 244, 248))
        self.label(canvas, f"Peak fingertip force: {self.peak_force:.2f} N", 16, 577, .55)
        self.label(canvas, "Fixed fixture / physical simulation", 520, 577, .55)
        bgr = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)
        self.writer.write(bgr)
        self.frames.append({"t": round(obs.t, 4), "phase": command.phase,
                            "head_image_t": float(obs.image_t),
                            "wrist_image_t": float(wrist.captured_at) if wrist is not None else None,
                            "contact_seen": self.contact_seen, "peak_force_n": self.peak_force})
        if command.phase == "press" and force > self.poster_force:
            cv2.imwrite(str(self.path.with_suffix(".jpg")), bgr)
            self.poster_force = force

    def close(self):
        self.writer.release()
        self.renderer.close()
        self.path.with_suffix(".frames.json").write_text(
            json.dumps({"fps": self.fps, "frames": self.frames}, indent=2) + "\n")
