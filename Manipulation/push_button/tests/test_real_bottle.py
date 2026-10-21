"""実機入口のプレビューがDDSへ接続せず、同じ経路を計画することを検査。"""

import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "real")]

import push_bottle


def test_execution_requires_existing_calibration_gate_before_dds(monkeypatch, tmp_path):
    def no_dds(*args, **kwargs):
        pytest.fail("校正指定なしでDDSへ接続してはいけません")

    monkeypatch.setattr(push_bottle, "ArmSdk", no_dds)
    path = tmp_path / "rejected.json"
    assert push_bottle.main(["--model", "unused.xml", "--base-transform", "unused.json",
                             "--table-height", ".75", "--execute", "--output", str(path)]) == 2
    assert "--calibrated" in json.loads(path.read_text())["error"]


def test_real_preview_uses_camera_depth_and_plans_all_phases_without_dds(monkeypatch, tmp_path):
    model = os.environ.get("G1_MUJOCO_MODEL")
    if not model:
        pytest.skip("G1_MUJOCO_MODELを指定してください")
    from kinematics import load_robot
    from test_bottle import Camera, TRANSFORM

    _, rest = load_robot(Path(model), (.11, -.003, 0), .79)
    calibration = {"schema": "g1-button-calibration-v2", "camera_body": "torso_link",
                   "camera_offset_m": [0, 0, 0], "camera_rpy_offset_rad": [0, 0, 0],
                   "reference_waist_q_rad": rest[12:15].tolist(), "reference_imu_rpy_rad": [0, 0, 0],
                   "tip_radius_m": .015, "tip_offset_m": [.11, -.003, 0],
                   "pelvis_height_m": .79, "position_uncertainty_m": .001}
    cal_path, state_path = tmp_path / "calibration.json", tmp_path / "state.json"
    cal_path.write_text(json.dumps(calibration))
    state_path.write_text(json.dumps({"motor_q_rad": rest.tolist(), "imu_rpy_rad": [0, 0, 0]}))
    clock = Camera()
    clock.x = .33
    monkeypatch.setattr(push_bottle.time, "monotonic", clock.now)

    class Frames:
        closed = False

        def read_after(self, after, deadline):
            frame = clock.read(after, deadline)
            intr = frame.intrinsics
            return SimpleNamespace(color_bgr=frame.color_bgr, depth_m=lambda: frame.depth_m,
                                   intrinsics=intr, frame_id=frame.sequence), frame.captured_at

        def close(self):
            self.closed = True

    frames = Frames()
    monkeypatch.setattr(push_bottle, "J1CameraRuntime", lambda *_: object())
    monkeypatch.setattr(push_bottle, "J1RgbdCamera", lambda *_: frames)
    monkeypatch.setattr(push_bottle, "CalibratedHeadCamera", lambda *_: SimpleNamespace(transform=lambda *_: TRANSFORM))
    monkeypatch.setattr(push_bottle, "make_bottle_detector", lambda *_, **kw: clock)
    monkeypatch.setattr(push_bottle, "ArmSdk", lambda *_: pytest.fail("プレビューでDDSへ接続してはいけません"))
    output = tmp_path / "preview.json"
    assert push_bottle.main(["--model", model, "--base-transform", str(cal_path),
                             "--state-json", str(state_path), "--table-height", ".75",
                             "--output", str(output)]) == 0
    report = json.loads(output.read_text())
    assert report["schema"] == "g1-bottle-preview-v1"
    assert {"raise", "approach", "push", "retract", "return_raise", "home"} <= report["poses"].keys()
    assert report["physical_push_success"] is None
    assert len(report["measurements"]) == 1
    assert frames.closed
