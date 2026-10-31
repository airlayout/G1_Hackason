"""実機入口のプレビューがDDSへ接続せず、同じ経路を計画することを検査。"""

import json
import os
from pathlib import Path
import sys
import threading
import time
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


@pytest.mark.parametrize('profile', ['strict', 'field'])
def test_real_preview_uses_camera_depth_and_plans_all_phases_without_dds(monkeypatch, tmp_path, profile):
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
    if profile == 'field':
        import field_motion
        monitor_type = field_motion.FieldMonitor
        monkeypatch.setattr(field_motion, 'FieldMonitor',
                            lambda policy, state: monitor_type(policy, state, now=clock.now))

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
                             "--profile", profile, "--no-track-table", "--no-record-rgbd",
                             "--output", str(output)]) == 0
    report = json.loads(output.read_text())
    assert report["schema"] == "g1-bottle-preview-v1"
    assert {"raise", "approach", "push", "retract", "return_raise", "home"} <= report["poses"].keys()
    assert report["physical_push_success"] is None
    assert report['approach_mode'] == ('staged' if profile == 'field' else 'single')
    assert report['approach_clearances_m'] == ([.10, .07, .05, .03] if profile == 'field' else [.05])
    assert len(report["measurements"]) == 1
    assert frames.closed


@pytest.mark.parametrize('release_fails', [False, True])
def test_real_worker_owns_camera_and_fk_and_diagnostic_exit_does_not_claim_push(monkeypatch, tmp_path, release_fails):
    model = os.environ.get('G1_MUJOCO_MODEL')
    if not model:
        pytest.skip('G1_MUJOCO_MODELを指定してください')
    import numpy as np
    from kinematics import load_robot
    from test_bottle import cylinder_image, INTRINSICS, TRANSFORM
    _, rest = load_robot(Path(model), (.11, -.003, 0), .79)
    calibration = {'schema': 'g1-button-calibration-v2', 'camera_body': 'torso_link',
        'camera_offset_m': [0, 0, 0], 'camera_rpy_offset_rad': [0, 0, 0],
        'reference_waist_q_rad': rest[12:15].tolist(), 'reference_imu_rpy_rad': [0, 0, 0],
        'tip_radius_m': .015, 'tip_offset_m': [.11, -.003, 0],
        'pelvis_height_m': .79, 'position_uncertainty_m': .001}
    cal = tmp_path/'calibration.json'
    cal.write_text(json.dumps(calibration))
    depth, bbox = cylinder_image(.33)
    owners, models, releases = [], [], []

    class Camera:
        def __init__(self, *args):
            self.sequence = 0
            owners.append(('open', threading.get_ident()))

        def poll(self):
            owners.append(('read', threading.get_ident()))
            self.sequence += 1
            return SimpleNamespace(color_bgr=np.zeros((240, 320, 3), np.uint8), depth_m=lambda: depth,
                intrinsics=INTRINSICS, frame_id=self.sequence), time.monotonic()-.03

        def close(self):
            owners.append(('close', threading.get_ident()))

    class Geometry:
        def __init__(self, runtime, kin, calibration):
            self.kin = kin
            models.append(kin)

        def transform(self, q, imu):
            self.kin.update_fixed_base(np.array(q), imu)
            return TRANSFORM.copy()

    class Arm:
        def __init__(self, *args):
            pass

        def current_configuration(self):
            return rest.copy(), (0., 0., 0.), time.monotonic(), 0., 0.

        def stop_walking(self):
            pass

        def release(self):
            releases.append(True)
            if release_fails:
                raise OSError('release failed')

    class Routine:
        def __init__(self, kin, motion, localizer, *args, **kwargs):
            self.kin, self.localizer = kin, localizer
            assert kwargs['live_tracking'] and kwargs['stop_before_contact']
            assert kin is not models[0] and kin.model is not models[0].model

        def execute(self):
            before = self.kin.data.qpos.copy()
            result = self.localizer.observe()
            assert result.target.center_xy == pytest.approx((.33, -.25), abs=.001)
            assert self.kin.data.qpos == pytest.approx(before)
            self.report = {'completed': False, 'motion_completed': False, 'visual_success': None,
                           'diagnostic_completed': True, 'home_completed': True, 'release_completed': True}
            return self.report

    monkeypatch.setattr(push_bottle, 'J1CameraRuntime', lambda *_: object())
    monkeypatch.setattr(push_bottle, 'CalibratedHeadCamera', Geometry)
    monkeypatch.setattr(push_bottle, 'J1RgbdCamera', Camera)
    monkeypatch.setattr(push_bottle, 'make_bottle_detector', lambda *args, **kwargs: SimpleNamespace(
        detect=lambda _: [SimpleNamespace(class_name='bottle', bbox=bbox)]))
    monkeypatch.setattr(push_bottle, 'ArmSdk', Arm)
    monkeypatch.setattr(push_bottle, 'ArmMotion', lambda *args, **kwargs: object())
    monkeypatch.setattr(push_bottle, 'BottlePushRoutine', Routine)
    monkeypatch.setattr('builtins.input', lambda _: 'YES')
    output = tmp_path/'diagnostic.json'
    code = push_bottle.main(['--model', model, '--base-transform', str(cal), '--table-height', '.75',
        '--execute', '--calibrated', '--stop-before-contact', '--output', str(output)])
    assert code == (2 if release_fails else 0)
    report = json.loads(output.read_text())
    assert report['diagnostic_completed'] is (not release_fails)
    assert not report['motion_completed']
    assert 'perception_close_error' not in report and releases
    assert owners[-1][0] == 'close'
    assert len({owner for _, owner in owners}) == 1
    assert owners[0][1] != threading.get_ident()
