from types import SimpleNamespace
import numpy as np
import pytest

from Manipulation.wrist_press.adapter import SampleBuilder
from Manipulation.wrist_press.dds import capture_time, TorqueCommandFactory
from Manipulation.wrist_press.perception import LatestPerception, StableTarget
from Manipulation.wrist_press.types import CameraFrame, Target


def frame(t=0,camera="head"):
    return CameraFrame(np.zeros((10,10,3),np.uint8),np.ones((10,10)),
                       np.array([[10,0,5],[0,10,5],[0,0,1]]),t,np.zeros(29),np.eye(4),camera)


def target(t=0,camera="head"):
    return Target("button","up",[.3,0,.2],[1,0,0],.02,t,camera=camera)


def test_repeated_frames_not_reprocessed_when_cameras_alternate():
    class Detector:
        calls = 0
        def locate(self,f,n,p):
            self.calls += 1
            return target(f.captured_at,f.camera)
    d = Detector()
    pipeline = LatestPerception(d)
    for _ in range(10):
        pipeline.update(frame(0,"head"),"up")
        pipeline.update(frame(0,"wrist"),"up")
    assert d.calls == 2


def test_stable_target_requires_new_exposures():
    stable = StableTarget()
    assert stable.add(target(0)) is None
    assert stable.add(target(0)) is None
    assert stable.add(target(.1)) is None
    assert stable.add(target(.2)) is not None


def test_image_uses_interpolated_exposure_state():
    class Robot:
        def head_transform(self,q):
            T=np.eye(4)
            T[0,3]=q[22]
            return T
    builder = SampleBuilder(Robot())
    def obs(t,stamp,q):
        f=frame()
        return SimpleNamespace(t=t,image_t=stamp,q=q,dq=np.zeros(29),rgb=f.rgb,depth=f.depth,K=f.K)
    builder.build(obs(0,0,np.zeros(29)))
    q=np.zeros(29)
    q[22]=.1
    sample=builder.build(obs(.1,.05,q))
    assert sample.frame.q[22] == pytest.approx(.05)
    assert sample.frame.T_pelvis_optical[0,3] == pytest.approx(.05)
    assert builder.build(obs(.2,-1,q)).frame is None


def test_clock_skew_and_stale_images_are_rejected():
    assert capture_time(99.9,100,3) == pytest.approx(2.9)
    with pytest.raises(ValueError):
        capture_time(99,100,3)
    with pytest.raises(ValueError):
        capture_time(101,100,3)


def test_dds_torque_preserves_other_motors_and_does_not_alias():
    def factory():
        return SimpleNamespace(tau=np.zeros(29))
    wrap=TorqueCommandFactory(factory)
    wrap.tau[:]=1.0
    cmd=wrap()
    wrap.tau[:]=2.0
    assert np.all(cmd.tau[:22]==0)
    assert np.all(cmd.tau[22:]==1)
