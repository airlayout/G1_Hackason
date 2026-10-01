"""Named trimmed MotionDecode trajectory profiles for the existing G1 runner."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from common import ROOT, JOINTS, NAMES, digest, read_motion, smoothstep
from play_g1_arms import MAX_ACCEL, MAX_SPEED, transition


PROFILE_PATH = ROOT / "config" / "named_reactions.json"


def load_trajectory_profile(name: str) -> dict:
    profiles = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))["reactions"]
    if name not in profiles:
        raise ValueError(f"Unknown named reaction profile: {name}")
    profile = profiles[name]
    required = ("source_csv", "source_frame_range", "base_duration_s", "playback_speed")
    if any(key not in profile for key in required):
        raise ValueError(f"Named reaction has no generic trajectory profile: {name}")
    source = ROOT / profile["source_csv"]
    if digest(source) != profile["source_sha256"]:
        raise ValueError(f"Named reaction source hash changed: {name}")
    return profile


def retime_upper_window(q: np.ndarray, duration: float, smoothing_window: int = 11,
                        sample_hz: int = 50) -> tuple[np.ndarray, np.ndarray]:
    q = np.asarray(q, dtype=float)
    if q.ndim != 2 or q.shape[1] != 17 or len(q) < 3 or not np.isfinite(q).all():
        raise ValueError("Expected at least three finite upper-body frames")
    if duration <= 0:
        raise ValueError("Duration must be positive")
    padded = np.pad(q, ((smoothing_window // 2, smoothing_window // 2), (0, 0)), mode="edge")
    kernel = np.ones(smoothing_window) / smoothing_window
    smoothed = np.stack([
        np.convolve(padded[:, i], kernel, mode="valid") for i in range(q.shape[1])
    ], axis=1)
    output_t = np.linspace(0, duration, int(round(duration * sample_hz)) + 1)
    phase = smoothstep(output_t / duration)
    path = np.stack([
        np.interp(phase, np.linspace(0, 1, len(q)), smoothed[:, i])
        for i in range(q.shape[1])
    ], axis=1)
    return output_t, path


def safe_transition_duration(a: np.ndarray, b: np.ndarray, minimum: float = 0.25) -> float:
    delta = float(np.max(np.abs(np.asarray(b) - np.asarray(a))))
    return max(minimum, 1.875 * delta / MAX_SPEED, np.sqrt(5.774 * delta / MAX_ACCEL))


def precompute_named_reaction(name: str, arm_scale: float, waist_scale: float,
                              playback_speed: float | None = None,
                              fast_reaction: bool = False) -> dict:
    """Load and retime a named asset without binding it to a robot pose."""
    if arm_scale not in (0.25, 0.5, 1.0):
        raise ValueError("Named arm scale must be 0.25, 0.5, or 1.0")
    if waist_scale not in (0.125, 0.25, 0.5, 1.0):
        raise ValueError("Named waist scale must be 0.125, 0.25, 0.5, or 1.0")
    profile = load_trajectory_profile(name)
    source, _ = read_motion(ROOT / profile["source_csv"])
    start, end = profile["source_frame_range"]
    speed = float(profile["playback_speed"] if playback_speed is None else playback_speed)
    if speed not in (1.0, 1.25, 1.5):
        raise ValueError("Playback speed must be 1.0, 1.25, or 1.5")
    duration = float(profile["base_duration_s"]) / speed
    _, raw_clip = retime_upper_window(source[start:end, 19:36], duration)
    delta = raw_clip - raw_clip[0]
    scaled_delta = arm_scale * delta
    scaled_delta[:, :3] = waist_scale * delta[:, :3]
    entry_s = 0.10 if fast_reaction else 0.25
    return_s = 0.50 if fast_reaction else safe_transition_duration(
        scaled_delta[-1], np.zeros(17))
    return {
        "name": name,
        "profile": profile,
        "delta": delta,
        "scaled_delta": scaled_delta,
        "arm_scale": float(arm_scale),
        "waist_scale": float(waist_scale),
        "speed": speed,
        "durations": {"entry": entry_s, "clip": duration, "return": return_s},
        "fast_reaction": bool(fast_reaction),
    }


def align_precomputed_reaction(asset: dict, q0: np.ndarray) -> tuple[dict, dict, dict, dict]:
    """Bind an in-memory delta asset to the latest q0 with no file parsing."""
    q0 = np.asarray(q0, dtype=float)
    if q0.shape != (29,) or not np.isfinite(q0).all():
        raise ValueError("Fresh q0 must contain 29 finite joints")
    neutral = q0[12:]
    full_clip = neutral + np.asarray(asset["delta"], dtype=float)
    clip = neutral + np.asarray(asset["scaled_delta"], dtype=float)
    durations = dict(asset["durations"])
    paths = {
        "entry": transition(neutral, clip[0], durations["entry"]),
        "clip": clip,
        "return": transition(clip[-1], neutral, durations["return"]),
    }
    full_return_s = (0.50 if asset["fast_reaction"] else
                     safe_transition_duration(full_clip[-1], neutral))
    full = {
        "entry": transition(neutral, full_clip[0], durations["entry"]),
        "clip": full_clip,
        "return": transition(full_clip[-1], neutral, full_return_s),
    }
    profile = asset["profile"]
    start, end = profile["source_frame_range"]
    metadata = {
        "named_reaction": asset["name"], "source": profile["source"],
        "source_csv": profile["source_csv"], "source_frame_range": [int(start), int(end)],
        "trimmed_frame_range": [int(start), int(end)],
        "playback_speed": asset["speed"], "clip_duration_s": durations["clip"],
        "fresh_q0_aligned": True, "arm_scale": asset["arm_scale"],
        "waist_scale": asset["waist_scale"], "fast_reaction": asset["fast_reaction"],
    }
    return paths, full, durations, metadata


def build_named_paths(snapshot: dict, name: str, arm_scale: float,
                      waist_scale: float, playback_speed: float | None = None,
                      fast_reaction: bool = False
                      ) -> tuple[dict, dict, dict, dict]:
    q0 = np.asarray(snapshot["all_q"], dtype=float)
    asset = precompute_named_reaction(
        name, arm_scale, waist_scale, playback_speed, fast_reaction)
    return align_precomputed_reaction(asset, q0)
