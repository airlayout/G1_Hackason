import json
from pathlib import Path
import cv2
import numpy as np
import pytest
from adapters.video_source import VideoSource, VideoError, interval_timestamps, validate_timestamps
from config import Config
from run_video import evaluate_video


@pytest.fixture
def video(tmp_path):
    path = tmp_path / "test.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 10, (64, 48))
    assert writer.isOpened()
    for i in range(30):
        writer.write(np.full((48, 64, 3), i * 7, dtype=np.uint8))
    writer.release()
    return path


@pytest.mark.parametrize("values", [[], [-1], [float("nan")], [float("inf")], [2, 1], [1, 1]])
def test_invalid_timestamps(values):
    with pytest.raises(ValueError):
        validate_timestamps(values)


@pytest.mark.parametrize("interval", [0, -1, float("nan"), float("inf"), 1e-310])
def test_invalid_intervals(interval):
    with pytest.raises(ValueError):
        interval_timestamps(29.5, interval)


def test_interval_excludes_end():
    assert interval_timestamps(29.5, 5) == [0, 5, 10, 15, 20, 25]
    assert interval_timestamps(10, 5) == [0, 5]


def test_exact_timestamp_seek_and_repeat(video):
    source = VideoSource(video)
    with source:
        assert source.metadata["duration_s"] == 3
        first = source.frame_at(2)
        second = source.frame_at(2)
        assert first.frame_index == second.frame_index == 20
        assert first.actual_timestamp == pytest.approx(2)
        assert first.image.size == (64, 48)
        assert first.image.tobytes() == second.image.tobytes()
        with pytest.raises(VideoError):
            source.frame_at(3)
    assert source.capture is None


def test_missing_video(tmp_path):
    with pytest.raises(VideoError):
        VideoSource(tmp_path / "missing.mp4").open()


def test_video_uses_existing_brain_parser_and_saves_inputs(video, tmp_path, monkeypatch):
    import run_video
    monkeypatch.setattr(run_video, "PROJECT_ROOT", tmp_path)
    class Engine:
        def generate(self, image, memory_context=None):
            assert image.size == (64, 48)
            return '{"action":"WAIT","direction":"UNKNOWN","target":"NONE","distance":"UNKNOWN","confidence":0.5}', {}
    report_path = tmp_path / "report.json"
    result = evaluate_video(video, [0, 1, 2], Config(), report_path, save_frames=True, engine=Engine())
    assert result["status"] == "pass"
    persisted = json.loads(report_path.read_text())
    assert len(persisted["frames"]) == 3
    assert all(Path(f["saved_input_frame"]).is_file() for f in persisted["frames"])
    assert all(f["decision"]["action"] == "WAIT" and not f["fallback"] for f in persisted["frames"])


def test_video_inference_failure_wait_report(video, tmp_path):
    class Engine:
        def generate(self, *args): raise RuntimeError("failed inference")
    result = evaluate_video(video, [0], Config(), tmp_path / "report.json", engine=Engine())
    assert result["status"] == "failed"
    assert result["frames"][0]["decision"]["action"] == "WAIT"


def test_out_of_range_does_not_infer(video, tmp_path):
    result = evaluate_video(video, [10], Config(), tmp_path / "report.json", engine=object())
    assert result["status"] == "failed" and not result["frames"]


def test_memory_context_is_passed_and_actions_are_not_overridden(video, tmp_path):
    # This fake engine deliberately returns WAIT after a sighting. Memory must
    # retain RIGHT but the runner must not force SEARCH or change the VLM Action.
    class Engine:
        def __init__(self): self.contexts = []
        def generate(self, image, memory_context=None, previous_image=None):
            self.contexts.append(memory_context)
            payload = {"action": "LOOK" if len(self.contexts) == 1 else "WAIT",
                       "direction": "RIGHT" if len(self.contexts) == 1 else "UNKNOWN",
                       "target": "PERSON" if len(self.contexts) == 1 else "NONE",
                       "distance": "UNKNOWN", "confidence": 0.7}
            visible = len(self.contexts) == 1
            obs = {"person_visible": visible, "person_count": int(visible),
                   "person_direction": "RIGHT" if visible else "UNKNOWN", "person_distance": "UNKNOWN",
                   "blocking_obstacle": "NONE", "person_transition": "UNKNOWN"}
            return json.dumps({"observation": obs, "decision": payload}), {}
    engine = Engine()
    report = evaluate_video(video, [0, 1], Config(combined_output=True), tmp_path / "memory.json", engine=engine, use_memory=True)
    assert engine.contexts[1]["last_seen_person_direction"] == "RIGHT"
    assert engine.contexts[1]["person_visible_last_frame"] is True
    assert report["frames"][1]["decision"]["action"] == "WAIT"
    assert report["frames"][1]["memory_after"]["frames_since_person_seen"] == 1
