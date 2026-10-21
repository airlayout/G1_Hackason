from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


def vector(value, size=3):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f"expected finite vector of length {size}")
    return a.copy()


def unit(value):
    a = vector(value)
    n = np.linalg.norm(a)
    if n < 1e-9:
        raise ValueError("zero direction")
    return a / n


@dataclass
class CameraFrame:
    rgb: np.ndarray                 # RGB, never BGR
    depth: np.ndarray | None        # optical z in metres
    K: np.ndarray
    captured_at: float              # same clock as observation.t
    q: np.ndarray                   # interpolated at exposure time
    T_pelvis_optical: np.ndarray
    camera: str = "head"
    sequence: int = 0

    def __post_init__(self):
        self.q = vector(self.q, 29)
        self.K = np.asarray(self.K, dtype=float)
        self.T_pelvis_optical = np.asarray(self.T_pelvis_optical, dtype=float)
        if self.K.shape != (3, 3) or not np.isfinite(self.K).all() or min(self.K[0, 0], self.K[1, 1]) <= 0:
            raise ValueError("invalid camera intrinsics")
        T = self.T_pelvis_optical
        if (T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1])
                or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-4)
                or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-4)):
            raise ValueError("invalid camera transform")
        if self.rgb.ndim != 3 or self.rgb.shape[2] != 3:
            raise ValueError("RGB frame required")
        if self.depth is not None and self.depth.shape != self.rgb.shape[:2]:
            raise ValueError("depth must be aligned to RGB")
        if not np.isfinite(self.captured_at):
            raise ValueError("invalid image time")


@dataclass
class Target:
    kind: str
    name: str
    point: np.ndarray
    normal: np.ndarray             # points INTO the target
    radius: float
    captured_at: float
    confidence: float = 1.0
    camera: str = "head"
    bbox: tuple = ()
    brightness: float | None = None
    wall_point: np.ndarray | None = None

    def __post_init__(self):
        self.point = vector(self.point)
        self.normal = unit(self.normal)
        if self.kind not in ("button", "bottle") or not 0.005 < self.radius < 0.1:
            raise ValueError("invalid target kind/dimensions")
        if not np.isfinite([self.captured_at, self.confidence]).all():
            raise ValueError("invalid target metadata")
        if self.wall_point is not None:
            self.wall_point = vector(self.wall_point)


@dataclass
class Sample:
    q: np.ndarray
    dq: np.ndarray
    t: float
    frame: CameraFrame | None
    wrist: CameraFrame | None = None

    def __post_init__(self):
        self.q, self.dq = vector(self.q, 29), vector(self.dq, 29)
        if not np.isfinite(self.t):
            raise ValueError("invalid observation time")


@dataclass
class Command:
    q: np.ndarray
    done: bool = False
    phase: str = "look"
    error: str = ""
    tau_ff: np.ndarray | None = None


@dataclass
class Report:
    mode: str
    detector: str
    phases: list = field(default_factory=list)
    cameras: list = field(default_factory=list)
    corrections: int = 0
    press_confirmed: bool = False
    contact_estimated: bool = False
    returned: bool = False
    error: str = ""
    elapsed_s: float = 0.0
    perception_ms: list = field(default_factory=list)
    planning_ms: list = field(default_factory=list)
    alignment_camera: str | None = None
    surface_camera: str | None = None
    correction_reversals: int = 0
    alignment_diagnostics: list = field(default_factory=list)
