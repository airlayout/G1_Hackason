"""final8のクラス・選択・既存モデル互換と、実機入口への接続。"""
import argparse
import hashlib
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "real")]
from button_detector import (ButtonDetector, DEFAULT_BUTTON_WEIGHTS, add_sim_detector_arguments,
                             direction_detections, make_button_detector, resolve_sim_weights)


def detection(name, box=(10, 10, 30, 30)):
    return SimpleNamespace(class_name=name, confidence=.9, bbox=box)


def backend(names, detections):
    return SimpleNamespace(class_names=names, device="cpu", detect=lambda _: detections)


def test_direction_keeps_all_matching_candidates_without_guessing():
    up, down1, down2 = detection("up"), detection("down"), detection("down", (40, 40, 60, 60))
    detector = ButtonDetector(backend(("up", "down"), [up, down1, down2]), direction="down")
    assert detector.detect(None) == [down1, down2]
    detector.detector.detect = lambda _: []
    assert detector.detect(None) == []


def test_pair_keeps_arrows_and_legacy_model_keeps_button():
    up, down, other = detection("up"), detection("down"), detection("person")
    assert ButtonDetector(backend(("up", "down", "person"), [up, down, other])).detect(None) == [up, down]
    old = detection("button")
    legacy = ButtonDetector(backend(("button",), [old]))
    assert direction_detections(legacy.detect(None), "down") == [old]
    # 実機には方向を推測させず、旧モデルでは画素指定を案内する。
    with pytest.raises(ValueError, match="target-pixel"):
        ButtonDetector(legacy.detector, direction="down")
    with pytest.raises(ValueError, match="ボタンクラス"):
        ButtonDetector(backend(("bottle",), []))


def test_factory_passes_explicit_settings_and_warms_up(tmp_path):
    path = tmp_path / "custom.pt"
    path.write_bytes(b"fake")
    seen, calls = [], []

    class Detector:
        class_names = ("up", "down")
        device = "cpu"
        def __init__(self, **kwargs):
            calls.append(kwargs)
        def detect(self, image):
            seen.append(image.copy())
            return [detection("up"), detection("down")]

    runtime = SimpleNamespace(module=lambda _: SimpleNamespace(perception=lambda _: SimpleNamespace(YoloDetector=Detector)))
    det = make_button_detector(path, .65, "cpu", runtime=runtime, imgsz=960, direction="down")
    assert calls == [dict(model_name=str(path), classes=None, confidence_threshold=.65, device="cpu", imgsz=960)]
    assert seen[0].shape == (480, 640, 3) and not seen[0].any()
    image = np.full((10, 20, 3), [12, 34, 56], dtype=np.uint8)
    assert [d.class_name for d in det.detect(image)] == ["down"]
    np.testing.assert_array_equal(seen[-1], image)  # BGRのまま渡す。
    with pytest.raises(FileNotFoundError):
        make_button_detector(tmp_path / "missing.pt", runtime=runtime)


@pytest.mark.parametrize("fallback", ["oracle", "color"])
def test_sim_requires_explicit_yolo_but_no_weight_path(fallback, tmp_path):
    parser = argparse.ArgumentParser()
    add_sim_detector_arguments(parser, fallback=fallback)
    assert resolve_sim_weights(parser.parse_args([]), fallback=fallback) is None
    args = parser.parse_args(["--detector", "yolo"])
    assert resolve_sim_weights(args, fallback=fallback) == str(DEFAULT_BUTTON_WEIGHTS)
    assert args.confidence == .4 and args.detector_imgsz == 640
    custom = str(tmp_path / "old.pt")
    assert resolve_sim_weights(parser.parse_args(["--weights", custom]), fallback=fallback) == custom
    with pytest.raises(ValueError):
        resolve_sim_weights(parser.parse_args(["--detector", fallback, "--weights", custom]), fallback=fallback)


def test_real_default_direction_and_pixel_selection(monkeypatch):
    import localize_button
    from alignment import TargetTracker
    from vision import ButtonTarget

    frames = [detection("up", (0, 0, 10, 10)), detection("down", (10, 10, 20, 20)),
              detection("down", (30, 30, 40, 40))]
    det = ButtonDetector(backend(("up", "down"), frames), direction="down")
    localizer = localize_button.RgbdLocalizer.__new__(localize_button.RgbdLocalizer)
    localizer.transform = np.eye(4)
    localizer.target_pixel = None
    localizer.detector = det
    localizer.tracker = TargetTracker()
    monkeypatch.setattr(localize_button, "estimate_button_target", lambda depth, bbox, *args, **kwargs:
        ButtonTarget((.35, -.25, 1.), (1., 0., 0.), .5, 1., 1.))
    frame = SimpleNamespace(intrinsics=SimpleNamespace(fx=250, fy=250, cx=20, cy=20),
                            color_bgr=np.zeros((50, 50, 3), np.uint8), depth_m=lambda: np.ones((50, 50)))
    assert len(localizer._candidates(frame)) == 2
    with pytest.raises(ValueError, match="一意"):
        localizer.tracker.update([t for t, _ in localizer._candidates(frame)], 1.)
    localizer.target_pixel = (15, 15)
    assert [box for _, box in localizer._candidates(frame)] == [(10, 10, 20, 20)]
    import align_button
    args = align_button.build_argument_parser().parse_args([
        "--model", "unused", "--base-transform", "unused", "--stroke", ".0015", "--output", "unused",
        "--button-direction", "down"])
    assert args.weights is None and args.confidence == .4 and args.button_direction == "down"


def test_bundled_model_hash_and_classes():
    assert hashlib.sha256(DEFAULT_BUTTON_WEIGHTS.read_bytes()).hexdigest() == (
        "7795e946b5331f17160ea688ef7ad2d8ea10cd59c93ef420f18bd75ca62eec41")
    pytest.importorskip("ultralytics")
    detector = make_button_detector(device="cpu")
    assert set(detector.class_names) == {"up", "down"}


def test_optional_robust_selectors_keep_defaults():
    from robust_options import add_arguments, parse
    parser = argparse.ArgumentParser()
    add_arguments(parser)
    parser.add_argument("--button-direction", choices=("up", "down"))
    args, _ = parse(parser, [])
    assert args.button_direction is None


def test_real_robust_defaults_reach_detector_before_camera_or_sdk(monkeypatch, tmp_path):
    pytest.importorskip("mujoco")
    import robust_push
    calls = []

    def factory(weights, confidence, device, **kwargs):
        calls.append((weights, confidence, device, kwargs))
        raise ValueError("detector reached")

    monkeypatch.setattr(robust_push, "read_calibration", lambda *a, **k: ({}, None))
    monkeypatch.setattr(robust_push, "J1CameraRuntime", lambda _: "fake runtime")
    monkeypatch.setattr(robust_push, "make_button_detector", factory)
    monkeypatch.setattr(robust_push, "J1RgbdCamera", lambda *a: pytest.fail("camera must not open"))
    code = robust_push.main(["--model", "unused", "--base-transform", "unused", "--state-json", "unused",
        "--stroke", ".0015", "--button-direction", "down", "--output", str(tmp_path / "trial.json")])
    assert code == 2
    weights, confidence, device, kwargs = calls[0]
    assert weights is None and confidence == .4 and device == "cpu"
    assert kwargs == dict(runtime="fake runtime", imgsz=640, direction="down")
