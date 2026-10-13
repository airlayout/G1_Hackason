"""J1-genの実際の配信形式、鮮度検査、床座標への接続を検証する。"""

import json
import queue
import sys
import threading
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "real"))

import j1gen_camera
import localize_button
from align_button import read_calibration
from j1gen_camera import CalibratedHeadCamera, J1CameraRuntime, J1RgbdCamera


@pytest.fixture
def runtime():
    # 他PCではprepare_camera.pyを実行してから、この実部品との互換性検査を実行する。
    pytest.importorskip("cv2")
    pytest.importorskip("zmq")
    try:
        return J1CameraRuntime()
    except FileNotFoundError:
        pytest.skip("real/prepare_camera.pyでJ1-genのカメラ部品を準備してください")


def frame(runtime, stamp, frame_id, scale=0.001):
    protocol = runtime.module("rgbd_protocol")
    return protocol.RgbdFrame(
        color_bgr=np.full((360, 640, 3), [10, 20, 200], dtype=np.uint8),
        depth=np.full((360, 640), round(1 / scale), dtype=np.uint16),
        depth_scale=scale,
        intrinsics=protocol.Intrinsics(640, 360, 300, 300, 320, 180),
        timestamp=stamp, frame_id=frame_id)


class Clock:
    epoch = 100.3
    mono = 10.0

    def advance(self):
        self.epoch += 0.01
        self.mono += 0.01


def fake_receiver(monkeypatch, frames):
    clock = Clock()
    pending = deque(frames)

    class Source:
        closed = False

        def __init__(self, host, port, timeout):
            self.host, self.port, self.timeout = host, port, timeout

        def open(self):
            pass

        def close(self):
            self.closed = True

        def read_rgbd(self):
            clock.advance()
            return pending.popleft() if pending else None

    monkeypatch.setattr(j1gen_camera.time, "time", lambda: clock.epoch)
    monkeypatch.setattr(j1gen_camera.time, "monotonic", lambda: clock.mono)
    runtime = SimpleNamespace(module=lambda name: {
        "camera_cfg": SimpleNamespace(load_camera_config=lambda args: {
            "rgbd": {"server_address": args.host or "pc2", "port": 5556, "timeout_ms": 5000}}),
        "camera_rgbd": SimpleNamespace(RgbdZmqSource=Source),
    }[name])
    return runtime, clock


def test_only_distinct_post_wait_fresh_images_count(monkeypatch, runtime):
    frames = [frame(runtime, 99.0, 1), frame(runtime, 99.99, 2),
              frame(runtime, 100.1, 3), frame(runtime, 100.1, 3),
              frame(runtime, 100.2, 2), frame(runtime, 100.2, 4)]
    fake, clock = fake_receiver(monkeypatch, frames)
    camera = J1RgbdCamera(fake, host="override")
    first, captured_at = camera.read_after(100.0, 11.0)
    second, second_at = camera.read_after(100.0, 11.0)
    assert (first.frame_id, second.frame_id) == (3, 4)
    assert captured_at == pytest.approx(9.8)
    assert second_at == pytest.approx(9.9)
    assert camera.source.host == "override"
    with pytest.raises(RuntimeError, match="新しいRGB-D"):
        camera.read_after(clock.epoch, clock.mono + 0.2)
    camera.close()
    assert camera.source.closed


def test_unsynchronized_future_timestamp_aborts(monkeypatch, runtime):
    fake, _ = fake_receiver(monkeypatch, [frame(runtime, 101.0, 1)])
    camera = J1RgbdCamera(fake)
    try:
        with pytest.raises(RuntimeError, match="時計同期"):
            camera.read_after(100.0, 11.0)
    finally:
        camera.close()


def test_live_localizer_keeps_bgr_depth_scale_and_three_frame_policy(monkeypatch, runtime):
    frames = [frame(runtime, 100.31, 1, scale=0.00025),
              frame(runtime, 100.32, 2, scale=0.00025),
              frame(runtime, 100.33, 3, scale=0.00025)]
    fake, _ = fake_receiver(monkeypatch, frames)
    reader = J1RgbdCamera(fake)
    monkeypatch.setattr(localize_button, "J1RgbdCamera", lambda *args: reader)
    seen = []

    class Detector:
        def __init__(self, **kwargs):
            pass

        def detect(self, image):
            seen.append(image[180, 320].copy())
            return [SimpleNamespace(bbox=(300, 160, 340, 200))]

    fake.module = lambda name: SimpleNamespace(perception=lambda name: SimpleNamespace(YoloDetector=Detector))
    # カメラの前方Zをロボットの前方Xへ変換し、作業範囲内のボタンを作る。
    transform = np.array([[0, 0, 1, -0.65], [-1, 0, 0, -0.25],
                          [0, -1, 0, 1.0], [0, 0, 0, 1]])
    localizer = localize_button.RgbdLocalizer("unused.pt", transform, runtime=fake)
    try:
        observation = localizer.observe()
        assert len(seen) == 3
        np.testing.assert_array_equal(seen, [[10, 20, 200]] * 3)
        np.testing.assert_allclose(observation.target.face_xyz, [0.35, -0.25, 1.0], atol=1e-6)
        np.testing.assert_allclose(observation.target.press_direction, [1, 0, 0], atol=1e-6)
        assert localizer.last_frame.frame_id == 3
    finally:
        localizer.close()


def test_real_zmq_accepts_j1gen_wire_format_with_videohub_resolution(monkeypatch, runtime, tmp_path):
    import zmq

    # 受信側がRealSenseやUnitree SDKを開かないことも確認する。
    monkeypatch.setitem(sys.modules, "pyrealsense2", None)
    protocol = runtime.module("rgbd_protocol")
    stopped = threading.Event()
    address = queue.Queue()
    errors = []

    def publish():
        try:
            with zmq.Context() as context:
                with context.socket(zmq.PUB) as socket:
                    address.put(socket.bind_to_random_port("tcp://127.0.0.1"))
                    frame_id = 0
                    while not stopped.wait(0.02):
                        frame_id += 1
                        socket.send(protocol.encode_rgbd(frame(runtime, time.time(), frame_id)))
        except BaseException as error:
            errors.append(error)

    publisher = threading.Thread(target=publish, daemon=True)
    publisher.start()
    camera = None
    try:
        port = address.get(timeout=5)
        config = tmp_path / "camera.yaml"
        config.write_text(f"pc2_host: 127.0.0.1\nrgbd:\n  port: {port}\n  timeout_ms: 1000\n")
        camera = J1RgbdCamera(runtime, str(config))
        received, captured_at = camera.read_after(time.time(), time.monotonic() + 5)
        assert received.color_bgr.shape == (360, 640, 3)
        # JPEGは非可逆だが、BGRの並びと赤い物の色は保たれる。
        assert received.color_bgr[180, 320, 2] > 180
        assert received.color_bgr[180, 320, 0] < 30
        np.testing.assert_array_equal(received.depth, 1000)
        np.testing.assert_allclose(received.depth_m(), 1.0)
        assert 0 <= time.monotonic() - captured_at < 0.5
    finally:
        if camera is not None:
            camera.close()
        stopped.set()
        publisher.join(timeout=5)
    assert not publisher.is_alive()
    assert not errors


def test_official_head_fk_and_legacy_calibration_use_same_floor_frame(runtime):
    pytest.importorskip("pinocchio")
    geometry = runtime.module("camera_geometry")

    class Kin:
        data = SimpleNamespace(qpos=np.zeros(29))

        def update_fixed_base(self, q, rpy):
            self.data.qpos = np.array(q)
            self.pelvis = np.eye(4)
            self.pelvis[:3, :3] = geometry.rpy_to_matrix(rpy[0], rpy[1], 0)
            self.pelvis[2, 3] = 0.8

        def body_transform(self, name):
            assert name == "pelvis"
            return self.pelvis.copy()

    kin = Kin()
    calibration = {"schema": "g1-button-calibration-v2", "camera_body": "torso_link",
                   "camera_offset_m": [0.001, -0.002, 0.003],
                   "camera_rpy_offset_rad": [0.002, -0.003, 0.004]}
    camera = CalibratedHeadCamera(runtime, kin, calibration)
    reference_q = np.zeros(29)
    reference_q[12:15] = [0.01, -0.01, 0.02]
    reference_rpy = [0.01, 0.02, 0.5]
    measured = camera.transform(reference_q, reference_rpy)
    legacy = CalibratedHeadCamera(runtime, kin, {
        "schema": "g1-button-calibration-v1", "camera_body": "torso_link",
        "T_base_optical": measured, "reference_waist_q_rad": reference_q[12:15],
        "reference_imu_rpy_rad": reference_rpy})
    np.testing.assert_allclose(legacy.transform(reference_q, reference_rpy), measured, atol=1e-12)
    moved = reference_q.copy()
    moved[12:15] += [0.01, 0.005, -0.008]
    tilted = [0.02, 0.01, 0.5]
    updated = camera.transform(moved, tilted)
    np.testing.assert_allclose(legacy.transform(moved, tilted), updated, atol=1e-12)
    assert np.linalg.norm(updated[:3, 3] - measured[:3, 3]) > 0.001
    nominal = np.eye(4)
    nominal[:3, :3], nominal[:3, 3] = camera.head.pelvis_from_optical(moved[12:15])
    np.testing.assert_allclose(updated, kin.pelvis @ nominal @ camera.correction, atol=1e-12)


def test_v2_calibration_accepts_small_corrections_and_rejects_missing_values(tmp_path):
    payload = {"schema": "g1-button-calibration-v2", "pelvis_height_m": 0.8,
               "camera_offset_m": [0.001, 0, 0], "camera_rpy_offset_rad": [0, 0.001, 0],
               "tip_offset_m": [0.1, 0, 0], "tip_radius_m": 0.015,
               "button_radius_m": 0.025, "position_uncertainty_m": 0.003,
               "reference_waist_q_rad": [0, 0, 0], "reference_imu_rpy_rad": [0, 0, 0]}
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(payload))
    calibration, transform = read_calibration(path)
    assert transform is None
    assert calibration["camera_body"] == "torso_link"
    payload["camera_offset_m"] = [None, None, None]
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="取付補正"):
        read_calibration(path)
