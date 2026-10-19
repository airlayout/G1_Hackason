import numpy as np
import pytest

from Manipulation.wrist_press.perception import Detection, Localizer, NotFound, Yolo
from Manipulation.wrist_press.settings import load_settings
from Manipulation.wrist_press.types import CameraFrame


def bottle_localizer():
    # Inject only the detector: exercise real depth handling without model downloads.
    loc=Localizer.__new__(Localizer)
    loc.cfg=load_settings(mode="bottle")
    loc.tracks={}
    class Detector:
        def detect(self,rgb):
            return [Detection("bottle",.9,(0,0,100,100))]
    loc.detector=Detector()
    return loc


def make_frame(depth):
    T=np.eye(4)
    # Optical forward = pelvis forward, optical right = -pelvis y.
    T[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    return CameraFrame(np.zeros((100,100,3),np.uint8),depth,
                       np.array([[100,0,50],[0,100,50],[0,0,1]]),0,np.zeros(29),T)


def test_bottle_label_depth_and_horizontal_push_direction():
    result=bottle_localizer()._bottle(make_frame(np.full((100,100),.4)))
    assert result.point[0] == pytest.approx(.4)
    assert result.normal[0] == pytest.approx(1)
    assert result.normal[2] == 0


def test_transparent_or_mixed_bottle_depth_is_not_background_target():
    loc=bottle_localizer()
    with pytest.raises(NotFound):
        loc._bottle(make_frame(np.zeros((100,100))))
    depth=np.full((100,100),.4)
    depth[:,50:]=1.5
    with pytest.raises(NotFound):
        loc._bottle(make_frame(depth))


def test_wrong_model_does_not_silently_fall_back(tmp_path):
    with pytest.raises(FileNotFoundError):
        Yolo(tmp_path/"missing.pt",["bottle"])
