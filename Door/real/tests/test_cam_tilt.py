"""11-05: pc2/cam_tilt.py（頭カメラ D435i の加速度計から床からの俯角）の試験。軸の読み方と平均の取り方を固める。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pc2"))
import cam_tilt  # noqa: E402

G = 9.80665


def _raw(pitch_deg: float, roll_deg: float = 0.0) -> tuple[float, float, float]:
    """床からの俯角・ロールのカメラが止まっているときの生の加速度（y＝画像の下・z＝後ろ）"""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    return G * math.sin(r), -G * math.cos(r) * math.cos(p), G * math.cos(r) * math.sin(p)


def test_level_camera_is_zero() -> None:
    assert cam_tilt.tilt_from_accel(0.0, -G, 0.0) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_straight_down_is_90() -> None:
    assert cam_tilt.tilt_from_accel(0.0, 0.0, G)[0] == pytest.approx(90.0)


@pytest.mark.parametrize("pitch,roll", [(53.2, 0.05), (30.0, -3.0), (70.0, 2.5)])
def test_round_trip(pitch: float, roll: float) -> None:
    assert cam_tilt.tilt_from_accel(*_raw(pitch, roll)) == pytest.approx((pitch, roll), abs=1e-6)


def test_measured_1105_sample() -> None:
    """11-05 の実測の平均（0.008, −5.891, 7.874）→ 53.20°"""
    pitch, roll = cam_tilt.tilt_from_accel(0.008, -5.891, 7.874)
    assert pitch == pytest.approx(53.20, abs=0.01)
    assert roll == pytest.approx(0.05, abs=0.01)


def test_summarize_averages_vectors_not_angles() -> None:
    """雑音は平均の加速度で消える。|a| の 1 g 比も出す"""
    acc = [_raw(50.0), _raw(56.0), _raw(53.0)]
    pitch, sd, roll, dn = cam_tilt.summarize(acc)
    assert pitch == pytest.approx(53.0, abs=0.1)
    assert sd > 2.0
    assert roll == pytest.approx(0.0, abs=1e-6)
    assert dn < 0.0  # 向きの違うベクトルの平均は 1 g より短い
