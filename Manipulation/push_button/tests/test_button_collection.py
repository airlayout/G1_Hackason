"""Collection holds the planned normal without hiding position or input failures."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

pytest.importorskip('mujoco')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from button_collection import CollectionCorrectionGate
from robust_perception import CorrectionGate, Observation
from robust_scene import SceneSnapshot


def scene():
    return SceneSnapshot('button',(.375,-.2,.85),(1.,0.,0.),.03,
                         (0.,)*29,(0.,0.,0.),1.)


def observation(s, *, frame=1, angle=8., offset=(0.,0.,0.)):
    theta = np.deg2rad(angle)
    return Observation(dict(face=tuple(np.asarray(s.face)+offset),
        normal=(np.cos(theta),np.sin(theta),0.),radius=s.radius),s.q,s.imu,1.,frame)


def test_collection_records_normal_change_and_keeps_planned_direction():
    s = scene(); gate = CollectionCorrectionGate(s)
    observed = observation(s,offset=(0.,.002,0.))
    with pytest.raises(RuntimeError,match='normal changed'):
        CorrectionGate(s).update(observed,0.)
    np.testing.assert_allclose(gate.update(observed,0.),[0.,.0005,0.])
    np.testing.assert_allclose(gate.update(observed,0.),[0.,.0005,0.])
    assert len(gate.normal_deviations) == 1
    assert gate.normal_deviations[0]['angle_deg'] == pytest.approx(8.)
    assert gate.normal_deviations[0]['planned_normal'] == [1.,0.,0.]
    assert observed.geometry['normal'] != s.normal


def test_collection_keeps_offset_frozen_at_contact():
    s = scene(); gate = CollectionCorrectionGate(s)
    np.testing.assert_array_equal(gate.update(observation(s),0.,freeze=True),np.zeros(3))
    assert gate.normal_deviations


@pytest.mark.parametrize('offset,message',[
    ((.011,0.,0.),'press direction'), ((0.,.006,0.),'correction envelope')])
def test_normal_tolerance_preserves_position_stops(offset,message):
    s = scene()
    with pytest.raises(RuntimeError,match=message):
        CollectionCorrectionGate(s).update(observation(s,offset=offset),0.)


def test_invalid_normal_still_stops():
    s = scene(); observed = observation(s)
    for normal in ((float('nan'),0.,0.),(0.,0.,0.)):
        invalid = replace(observed,geometry=dict(observed.geometry,normal=normal))
        with pytest.raises(RuntimeError,match='Invalid observed target normal'):
            CollectionCorrectionGate(s).update(invalid,0.)
