import numpy as np
import pytest

from Manipulation.wrist_press.settings import load_settings
from Manipulation.wrist_press.planning import Trajectory
from Manipulation.wrist_press.controller import CommandLimiter


@pytest.mark.parametrize("distance", [.0001,.02,.2,2.0])
def test_scurve_stays_on_path_and_respects_limits(distance):
    cfg = load_settings()
    a,b = np.zeros(29),np.zeros(29)
    b[22],b[26] = distance,-distance/2
    trajectory = Trajectory([a,b],cfg)
    stamps = np.linspace(0,trajectory.duration,6001)
    samples = np.array([trajectory.at(t) for t in stamps])
    dt = stamps[1]-stamps[0]
    velocity = np.diff(samples,axis=0)/dt
    acceleration = np.diff(velocity,axis=0)/dt
    jerk = np.diff(acceleration,axis=0)/dt
    assert np.max(np.abs(velocity)) <= cfg["joint_speed_rad_s"]*1.001
    assert np.max(np.abs(acceleration)) <= cfg["joint_acceleration_rad_s2"]*1.001
    assert np.max(np.abs(jerk)) <= cfg["joint_jerk_rad_s3"]*1.01
    assert np.allclose(samples[-1],b)
    assert np.allclose(samples[:,26],-.5*samples[:,22])
    assert np.all(np.diff(samples[:,22]) >= -1e-10)


def test_online_correction_rate_limits():
    cfg = load_settings()
    limiter = CommandLimiter(np.zeros(29),cfg)
    outputs = [limiter.q.copy()]
    for _ in range(100):
        goal = np.zeros(29)
        goal[22] = 1
        q,v = limiter.next(goal,.02)
        limiter.commit(q,v)
        outputs.append(q)
    velocity = np.diff(outputs,axis=0)/.02
    assert np.max(np.abs(velocity)) <= cfg["joint_speed_rad_s"]+1e-9
    assert np.max(np.abs(np.diff(velocity,axis=0)/.02)) <= cfg["joint_acceleration_rad_s2"]+1e-9


def test_invalid_settings_rejected(tmp_path):
    p = tmp_path/"bad.yaml"
    p.write_text("button:\n  stroke_m: 0.1\n")
    with pytest.raises(ValueError):
        load_settings(p)


def test_joint_limit_braking_does_not_cross_the_bound():
    cfg=load_settings()
    q=np.zeros(29)
    limiter=CommandLimiter(q,cfg,np.full(7,-1.),np.full(7,1.))
    for _ in range(300):
        goal=np.zeros(29)
        goal[22]=.997
        out,v=limiter.next(goal,.02)
        assert out[22]<1
        limiter.commit(out,v)
