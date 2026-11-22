"""Validated Ubuntu-side Pepper motion adapter.

QiSDK runs in the Android application on Pepper.  The HTTP transport here is
only the small LAN boundary; the Pepper-side implementation owns QiContext,
charging-flap checks, GoTo/LookAt Futures, timeouts, and cancellation.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import shlex
import subprocess
import time
from typing import Any, Protocol
from urllib import error, request


class PepperMotionError(RuntimeError):
    pass


class MotionValidationError(ValueError):
    pass


class MotionTransport(Protocol):
    def post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]: ...
    def status(self) -> dict[str, Any]: ...


def _number(name: str, value: Any, low: float, high: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MotionValidationError(f"{name} must be a number")
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise MotionValidationError(f"{name} must be between {low} and {high}")
    return value


@dataclass
class DryRunTransport:
    """Never opens a socket; records and prints validated commands."""

    output: Any = print

    def __post_init__(self):
        self.commands: list[tuple[str, dict[str, Any]]] = []

    def status(self) -> dict[str, Any]:
        return {"ready": True, "movement_running": False, "dry_run": True}

    def post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        self.commands.append((path, dict(payload)))
        if path == "/move":
            line = (f"DRY-RUN: MOVE forward={payload['forward_m']:g}m "
                    f"sideways={payload['sideways_m']:g}m speed={payload['speed_mps']:g}")
        elif path == "/turn":
            line = f"DRY-RUN: TURN {payload['direction']} {payload['degrees']:g}deg"
        elif path == "/look":
            line = (f"DRY-RUN: LOOK x={payload['x']:g} y={payload['y']:g} z={payload['z']:g} "
                    f"policy={payload['movement_policy']}")
        elif path == "/stop":
            line = "DRY-RUN: STOP"
        else:
            raise PepperMotionError(f"Unsupported motion path: {path}")
        self.output(line)
        return {"accepted": True, "dry_run": True}


class HttpBridgeTransport:
    """Minimal JSON client for a QiSDK Android bridge on Pepper."""

    def __init__(self, base_url: str, *, timeout_s: float = 3.0):
        if not isinstance(base_url, str) or not base_url.startswith(("http://", "https://")):
            raise ValueError("base_url must start with http:// or https://")
        self.base_url = base_url.rstrip("/")
        self.timeout_s = _number("timeout_s", timeout_s, 0.1, 30.0)

    def _request(self, path: str, *, method: str, payload: dict[str, Any] | None = None):
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        req = request.Request(
            self.base_url + path,
            data=body,
            method=method,
            headers={"Accept": "application/json", "Content-Type": "application/json"},
        )
        try:
            with request.urlopen(req, timeout=self.timeout_s) as response:
                result = json.loads(response.read().decode("utf-8"))
        except (error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            raise PepperMotionError(f"Pepper bridge request failed: {exc}") from exc
        if not isinstance(result, dict):
            raise PepperMotionError("Pepper bridge returned a non-object response")
        if result.get("error"):
            raise PepperMotionError(str(result["error"]))
        return result

    def status(self) -> dict[str, Any]:
        return self._request("/status", method="GET")

    def post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        return self._request(path, method="POST", payload=payload)


class Ros2HeadTransport:
    """Minimal ROS 2 backend for today's fixed, absolute HEAD_ONLY test."""

    def __init__(self, *, dry_run: bool = True, setup_script: str = "/opt/ros/jazzy/setup.bash",
                 overlay_script: str = "/tmp/pepper_naoqi_ws.BwBAe7/install2/setup.bash"):
        self.dry_run = dry_run
        self.setup_script = setup_script
        self.overlay_script = overlay_script

    def status(self) -> dict[str, Any]:
        return {"ready": True, "movement_running": False, "dry_run": self.dry_run}

    def post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        if path != "/look":
            raise PepperMotionError("ROS2 E2E backend currently permits LOOK only")
        y = float(payload["y"])
        direction = "LEFT" if y > 0 else "RIGHT" if y < 0 else "CENTER"
        yaw = {"LEFT": 0.30, "CENTER": 0.00, "RIGHT": -0.30}[direction]
        pitch, speed = 0.15, 0.10
        return self.look_at_joints(yaw, pitch, speed=speed)

    def look_at_joints(self, yaw: float, pitch: float, *, speed: float = 0.10) -> dict[str, Any]:
        yaw = _number("yaw", yaw, -0.80, 0.80)
        pitch = _number("pitch", pitch, -0.40, 0.40)
        speed = _number("speed", speed, 0.01, 0.30)
        message = ("{joint_names: ['HeadYaw', 'HeadPitch'], "
                   f"joint_angles: [{yaw:.2f}, {pitch:.2f}], speed: {speed:.2f}, relative: 0}}")
        print(f"[PEPPER] target HeadYaw={yaw:.2f} HeadPitch={pitch:.2f} speed={speed:.2f}")
        print(f"[ROS2] topic=/joint_angles relative=0 payload={message}")
        if self.dry_run:
            print("[PEPPER] DRY-RUN only")
            return {"accepted": True, "dry_run": True, "yaw": yaw, "pitch": pitch}
        command = (f"source {self.setup_script} && source {self.overlay_script} && "
                   "ros2 topic pub --once /joint_angles "
                   "naoqi_bridge_msgs/msg/JointAnglesWithSpeed " + repr(message))
        completed = subprocess.run(["/bin/bash", "-lc", command], check=True,
                                   text=True, capture_output=True, timeout=15)
        print(completed.stdout.strip())
        return {"accepted": True, "dry_run": False, "yaw": yaw, "pitch": pitch}

    def speak(self, text: str, *, delay_s: float = 0.75) -> dict[str, Any]:
        if not isinstance(text, str) or not text.strip() or len(text) > 80:
            raise MotionValidationError("speech must be 1..80 characters")
        text = text.strip()
        print(f"[PEPPER SPEAK] {text}")
        if self.dry_run:
            print("[PEPPER SPEAK] DRY-RUN only")
            return {"accepted": True, "dry_run": True, "speech": text}
        time.sleep(delay_s)
        message = "{data: " + json.dumps(text, ensure_ascii=False) + "}"
        command = (f"source {shlex.quote(self.setup_script)} && "
                   f"source {shlex.quote(self.overlay_script)} && "
                   "ros2 topic pub --once /speech std_msgs/msg/String "
                   + shlex.quote(message))
        completed = subprocess.run(["/bin/bash", "-lc", command], check=True,
                                   text=True, capture_output=True, timeout=15)
        print(completed.stdout.strip())
        return {"accepted": True, "dry_run": False, "speech": text}


class PepperMotionAdapter:
    """Pepper-independent command API used above the LAN/QiSDK boundary."""

    LOOK_DIRECTIONS = {
        "left": (1.0, 1.0, 1.2),
        "right": (1.0, -1.0, 1.2),
        "up": (1.0, 0.0, 2.0),
        "down": (1.0, 0.0, 0.0),
        "center": (1.0, 0.0, 1.2),
    }

    def __init__(self, transport: MotionTransport):
        self.transport = transport

    def _require_idle(self):
        status = self.transport.status()
        if status.get("ready") is False:
            raise PepperMotionError("Pepper bridge reports robot not ready")
        if status.get("movement_running") is True:
            raise PepperMotionError("Another movement is already running; STOP it first")

    def move(self, forward_m: float, sideways_m: float = 0.0,
             speed_mps: float = 0.2) -> dict[str, Any]:
        forward = _number("forward_m", forward_m, -0.5, 0.5)
        sideways = _number("sideways_m", sideways_m, -0.5, 0.5)
        speed = _number("speed_mps", speed_mps, 0.1, 0.3)
        if forward == 0.0 and sideways == 0.0:
            raise MotionValidationError("MOVE requires a non-zero distance")
        self._require_idle()
        return self.transport.post("/move", {
            "action": "MOVE", "forward_m": forward,
            "sideways_m": sideways, "speed_mps": speed,
        })

    def turn(self, degrees: float, direction: str | None = None) -> dict[str, Any]:
        signed = _number("degrees", degrees, -90.0, 90.0)
        if direction is None:
            if signed == 0.0:
                raise MotionValidationError("TURN requires a non-zero angle")
            direction = "left" if signed > 0 else "right"
            amount = abs(signed)
        else:
            direction = str(direction).lower()
            if direction not in {"left", "right"}:
                raise MotionValidationError("direction must be left or right")
            amount = abs(signed)
        if not 15.0 <= amount <= 90.0:
            raise MotionValidationError("degrees must be between 15 and 90")
        self._require_idle()
        return self.transport.post("/turn", {
            "action": "TURN", "direction": direction, "degrees": amount,
        })

    def look_at(self, x: float | None = None, y: float | None = None,
                z: float | None = None, *, direction: str | None = None) -> dict[str, Any]:
        if direction is not None:
            direction = str(direction).lower()
            if direction not in self.LOOK_DIRECTIONS:
                raise MotionValidationError("LOOK direction must be left/right/up/down/center")
            if any(value is not None for value in (x, y, z)):
                raise MotionValidationError("Use LOOK coordinates or direction, not both")
            x, y, z = self.LOOK_DIRECTIONS[direction]
        if any(value is None for value in (x, y, z)):
            raise MotionValidationError("LOOK requires x, y and z")
        target = {
            "x": _number("x", x, -5.0, 5.0),
            "y": _number("y", y, -5.0, 5.0),
            "z": _number("z", z, -2.0, 5.0),
        }
        if all(value == 0.0 for value in target.values()):
            raise MotionValidationError("LOOK target cannot be the robot origin")
        return self.transport.post("/look", {
            "action": "LOOK", **target, "movement_policy": "head_only",
        })

    def stop(self) -> dict[str, Any]:
        # STOP deliberately bypasses readiness/running checks and is idempotent.
        return self.transport.post("/stop", {"action": "STOP"})

    def execute(self, command: dict[str, Any]) -> dict[str, Any]:
        """Dispatch a strict structured command without exposing Pepper APIs."""
        if not isinstance(command, dict):
            raise MotionValidationError("command must be an object")
        action = command.get("action")
        allowed = {
            "MOVE": {"action", "forward_m", "sideways_m", "speed_mps"},
            "TURN": {"action", "direction", "degrees"},
            "LOOK": {"action", "x", "y", "z", "direction"},
            "STOP": {"action"},
        }
        if action not in allowed:
            raise MotionValidationError("action must be MOVE, TURN, LOOK or STOP")
        extras = set(command) - allowed[action]
        if extras:
            raise MotionValidationError(f"Unexpected command fields: {sorted(extras)}")
        if action == "MOVE":
            return self.move(command.get("forward_m", 0.0), command.get("sideways_m", 0.0),
                             command.get("speed_mps", 0.2))
        if action == "TURN":
            return self.turn(command.get("degrees"), command.get("direction"))
        if action == "LOOK":
            return self.look_at(command.get("x"), command.get("y"), command.get("z"),
                                direction=command.get("direction"))
        return self.stop()
