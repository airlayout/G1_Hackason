"""Real YOLO11n on the CPU (needs the pinned weights in .cache/models)."""
import cv2
import pytest

from pepperapp.detector import DetectorConfig, YoloDetector
from pepperapp.sources import FileSource
from pepperapp.weights import MODEL_DIR, WEIGHTS

pytestmark = pytest.mark.skipif(not all((MODEL_DIR / name).is_file() for name in WEIGHTS),
                                reason="run `python -m pepperapp.weights` first")


def test_person_with_keypoints_track_id_and_objects(sample_image):
    detector = YoloDetector(DetectorConfig(object_labels=("dog",), imgsz=640))
    frame = cv2.resize(cv2.imread(str(sample_image)), (640, 480))
    detector(frame)
    result = detector(frame)  # ByteTrack confirms IDs from the second frame
    assert result.width == 640 and result.height == 480
    person = max(result.persons, key=lambda p: p.area)
    assert person.keypoints.shape == (17, 2) and person.keypoint_conf.shape == (17,)
    assert person.track_id is not None
    assert "dog" in {o.label for o in result.objects}
    assert result.inference_ms > 0


def test_unknown_label_and_missing_weights_fail_clearly(tmp_path):
    with pytest.raises(ValueError, match="知らないラベル"):
        YoloDetector(DetectorConfig(object_labels=("unicorn",)))
    with pytest.raises(FileNotFoundError, match="weights"):
        YoloDetector(DetectorConfig(pose_weights=tmp_path / "missing.pt"))


@pytest.mark.parametrize("kwargs", [{"imgsz": 300}, {"confidence": 0.0}, {"threads": 0}])
def test_config_validation(kwargs):
    with pytest.raises(ValueError):
        DetectorConfig(**kwargs)


def test_file_source_repeats_a_still_image(sample_image):
    source = FileSource(sample_image, fps_for_images=50)
    first, second = source.read(), source.read()
    source.close()
    assert first.shape == second.shape and first.ndim == 3


def test_file_source_shrinks_wide_images_to_vga_width(sample_image):
    source = FileSource(sample_image, fps_for_images=50)
    frame = source.read()
    source.close()
    assert frame.shape[1] == 640 and frame.shape[0] == round(1365 * 640 / 2048)


def test_dog_is_not_taken_for_a_person(sample_image):
    """The pose model alone scores the dog as a person (0.73 after INTER_AREA resizing)."""
    frame = FileSource(sample_image).read()
    confirmed = YoloDetector(DetectorConfig(object_labels=()))
    alone = YoloDetector(DetectorConfig(object_labels=(), confirm_persons=False))
    assert len(alone(frame).persons) == 2
    persons = confirmed(frame).persons
    assert len(persons) == 1 and persons[0].confidence > 0.8
