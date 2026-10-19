from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import math

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]


def load_settings(path=None, **overrides):
    default = Path(__file__).with_name("config.yaml")
    cfg = yaml.safe_load(default.read_text())
    if path:
        def merge(a, b):
            for k, v in b.items():
                if isinstance(v, dict) and isinstance(a.get(k), dict):
                    merge(a[k], v)
                else:
                    a[k] = v
        merge(cfg, yaml.safe_load(Path(path).read_text()) or {})
    cfg.update({k: v for k, v in overrides.items() if v is not None})
    if cfg["mode"] not in ("button", "bottle"):
        raise ValueError("mode must be button or bottle")
    if cfg["detector"] not in ("yolo", "classic"):
        raise ValueError("detector must be yolo or explicit classic button diagnostic")
    if cfg["mode"] == "bottle" and cfg["detector"] != "yolo":
        raise ValueError("bottle mode requires the bottle YOLO")
    if cfg["gravity_mode"] not in ("position","torque"):
        raise ValueError("gravity_mode must be position or torque")
    for k in ("joint_speed_rad_s", "joint_acceleration_rad_s2", "joint_jerk_rad_s3",
              "clearance_m", "edge_resolution_rad", "planning_timeout_s", "approach_m",
              "retreat_m", "max_image_age_s", "cycle_timeout_s", "alignment_timeout_s"):
        if not math.isfinite(float(cfg[k])) or float(cfg[k]) <= 0:
            raise ValueError(f"{k} must be finite and positive")
    for name in ("button", "bottle"):
        p = cfg[name]
        if not 0 < p["stroke_m"] <= p["max_measured_penetration_m"] <= 0.015:
            raise ValueError(f"invalid {name} physical stroke limits")
        if not p["stroke_m"] <= p["max_command_penetration_m"] <= 0.02:
            raise ValueError(f"invalid {name} command compliance allowance")
        if not np.isfinite(p["feedforward_force_n"]) or not 0<=p["feedforward_force_n"]<=10:
            raise ValueError(f"invalid {name} feed-forward force")
    if not 0 < cfg["gravity_scale"] <= 1:
        raise ValueError("gravity_scale must be in (0, 1]")
    if cfg["retreat_m"] < cfg["approach_m"]:
        raise ValueError("retreat must clear the approach position")
    for key in ("tip_offset_m",):
        if np.asarray(cfg[key]).shape != (3,) or not np.isfinite(cfg[key]).all():
            raise ValueError(f"invalid {key}")
    return deepcopy(cfg)


def resolve_path(path):
    p = Path(path).expanduser()
    return p if p.is_absolute() else ROOT / p
