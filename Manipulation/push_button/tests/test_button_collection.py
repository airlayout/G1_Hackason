"""Collection records target motion while rejecting malformed observations."""
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


@pytest.mark.parametrize('offset',[(.011,0.,0.),(0.,.006,0.),(.03,.08,-.04)])
def test_target_movement_is_recorded_without_stopping(offset):
    s = scene()
    gate = CollectionCorrectionGate(s)
    for frame in range(1,31):
        result = gate.update(observation(s,offset=offset,frame=frame),0.)
        assert np.linalg.norm(result) <= .005+1e-12
    np.testing.assert_allclose(gate.position_changes[-1]['displacement_m'],offset)
    assert len(gate.position_changes)==30
    assert gate.position_changes[-1]['correction_clamped']==(np.linalg.norm(offset[1:])>.005)


@pytest.mark.parametrize('offset,message',[
    ((.011,0.,0.),'press direction'),((0.,.006,0.),'correction envelope')])
def test_standard_gate_keeps_its_position_limits(offset,message):
    s = scene()
    with pytest.raises(RuntimeError,match=message):
        CorrectionGate(s).update(observation(s,offset=offset,angle=0.),0.)


def test_visual_correction_returns_to_zero_when_new_frames_return_to_reference():
    s = scene();gate=CollectionCorrectionGate(s)
    for frame in range(1,11):
        gate.update(observation(s,offset=(0.,.08,0.),frame=frame),0.)
    assert np.linalg.norm(gate.offset)==pytest.approx(.005)
    for frame in range(11,21):
        gate.update(observation(s,frame=frame),0.)
    np.testing.assert_allclose(gate.offset,np.zeros(3),atol=1e-12)


@pytest.mark.parametrize('face',[(float('nan'),0.,0.),(0.,0.),(float('inf'),0.,0.)])
def test_invalid_positions_still_raise(face):
    s=scene();observed=observation(s)
    observed=replace(observed,geometry=dict(observed.geometry,face=face))
    with pytest.raises(RuntimeError,match='Invalid observed target position'):
        CollectionCorrectionGate(s).update(observed,0.)


def test_invalid_normal_still_stops():
    s = scene(); observed = observation(s)
    for normal in ((float('nan'),0.,0.),(0.,0.,0.)):
        invalid = replace(observed,geometry=dict(observed.geometry,normal=normal))
        with pytest.raises(RuntimeError,match='Invalid observed target normal'):
            CollectionCorrectionGate(s).update(invalid,0.)
