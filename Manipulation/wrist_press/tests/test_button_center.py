import numpy as np
import pytest

cv2=pytest.importorskip("cv2")

from Manipulation.wrist_press.perception import refine_button


def test_button_rim_corrects_a_biased_detection_box():
    rgb=np.full((120,120,3),20,np.uint8)
    cv2.ellipse(rgb,((60.,55.),(34.,28.),12.),(150,150,150),-1)
    cv2.fillPoly(rgb,[np.array([[60,48],[55,60],[65,60]])],(20,20,20))
    box,mask=refine_button(rgb,(45,38,87,76))
    center=(np.array(box[:2])+box[2:])/2
    assert np.linalg.norm(center-[60,55])<1
    assert mask[55,60]


def test_flat_patch_is_not_treated_as_a_button_rim():
    assert refine_button(np.full((80,80,3),90,np.uint8),(20,20,60,60)) is None
