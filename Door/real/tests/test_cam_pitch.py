"""11-05 夜: door_vision_core.pitch_for_floor_tilt（カメラの加速度計の床からの俯角に合う torso からの俯角）の試験"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import door_vision_core as dvc  # noqa: E402


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def test_upright_torso_pitch_equals_floor_tilt() -> None:
    pitch = dvc.pitch_for_floor_tilt(np.array([1.0, 0, 0, 0]), np.zeros(3), 53.25, math.radians(51.0))
    assert math.degrees(pitch) == pytest.approx(53.25, abs=1e-6)


@pytest.mark.parametrize("seed", range(5))
def test_reproduces_floor_tilt_with_leaning_torso(seed: int) -> None:
    rng = np.random.default_rng(seed)
    quat = _quat_from_rpy(*np.radians(rng.uniform(-4, 4, 3)))
    waist = np.radians(rng.uniform(-5, 5, 3))
    target = float(rng.uniform(40, 65))
    pitch = dvc.pitch_for_floor_tilt(quat, waist, target, math.radians(51.0))
    assert dvc.floor_tilt_deg(dvc.head_gravity(quat, waist, pitch)) == pytest.approx(target, abs=1e-5)


def test_1105_numbers() -> None:
    """11-05 夜: PITCH 51° で表示 50.9°・加速度計 53.25° → 手で決めた 53.4° と同じ所に来る（胴の前傾 ≈ −0.1°）"""
    quat = _quat_from_rpy(0.0, math.radians(-0.1), 0.0)
    shown = dvc.floor_tilt_deg(dvc.head_gravity(quat, np.zeros(3), math.radians(51.0)))
    assert shown == pytest.approx(50.9, abs=0.01)
    pitch = dvc.pitch_for_floor_tilt(quat, np.zeros(3), 53.25, math.radians(51.0))
    assert math.degrees(pitch) == pytest.approx(53.35, abs=0.01)



@pytest.mark.parametrize("deg, ok", [(29.9, False), (30.1, True), (53.4, True), (69.9, True), (83.6, False)])
def test_head_pitch_out_of_range(deg: float, ok: bool) -> None:
    from types import SimpleNamespace

    vp = SimpleNamespace(meta={"camera": {"pitch_range_deg": [30.0, 70.0]}})
    why = dvc.head_pitch_out_of_range(vp, math.radians(deg))
    assert (why is None) == ok
