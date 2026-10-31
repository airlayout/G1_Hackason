"""実スレッドで画像世代・鮮度・対象同一性と、保存待ちからの分離を検証する。"""
from dataclasses import replace
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from alignment import TargetTracker
from button_perception import ButtonPerceptionWorker
from trial_policy import TrialPause
from trial_recording import TrialRecorder
from test_alignment import button


@pytest.fixture
def sensor():
    state = SimpleNamespace(sequence=0, stale=False, duplicate=False, ambiguous=False, fail=False,
                            level=100, target=button(), closed=False, threads=[], transforms=[], block=None)

    class Camera:
        def poll(self):
            state.threads.append(threading.get_ident())
            if state.fail:
                raise RuntimeError('camera disconnected')
            if not state.duplicate:
                state.sequence += 1
            stamp = time.monotonic()-(1. if state.stale else .00001)
            frame = SimpleNamespace(frame_id=state.sequence, timestamp=time.time(),
                color_bgr=np.full((40, 40, 3), state.level, np.uint8),
                intrinsics=SimpleNamespace(fx=30., fy=30., cx=20., cy=20.),
                depth_m=lambda: np.ones((40, 40), np.float32))
            return frame, stamp

    class Localizer:
        def __init__(self):
            state.owner = threading.get_ident()
            self.camera, self.tracker = Camera(), TargetTracker()
            state.localizer = self

        def _candidates(self, frame, *, transform):
            state.threads.append(threading.get_ident())
            if state.block:
                state.block[0].set()
                assert state.block[1].wait(2.)
            candidates = [(state.target, (10, 10, 30, 30))]
            return candidates*2 if state.ambiguous else candidates

        def close(self):
            state.close_owner = threading.get_ident()
            state.closed = True

    def transform(frame, stamp):
        state.transforms.append((frame.frame_id, stamp))
        return np.eye(4), np.zeros(29), np.zeros(3)

    state.factory, state.transform = Localizer, transform
    return state


def test_measurements_use_three_distinct_new_frames_and_single_socket_owner(sensor, tmp_path):
    recorder = TrialRecorder(tmp_path/'rgbd')
    worker = ButtonPerceptionWorker(sensor.factory, sensor.transform, recorder=recorder)
    try:
        first = worker.observe()
        after = time.monotonic()
        second = worker.observe()
        assert second.captured_at > after and second.sequence == first.sequence+3
        assert [row['generation'] for row in worker.diagnostics] == [1]*3+[2]*3
        assert len({row['frame_id'] for row in worker.diagnostics}) == 6
        assert set(sensor.threads) == {sensor.owner} and sensor.owner != threading.get_ident()
        assert worker.last_lamp_frame.frame_id == worker.measurements[-1]['frame_id']
        assert len(worker.last_measurement_frame_ids) == 3
        assert list(worker.last_measurement_frame_ids) == worker.measurements[-1]['frame_ids']
    finally:
        worker.close()
        recording = recorder.close()
    assert sensor.closed and sensor.close_owner == sensor.owner
    assert recording['saved_frames'] >= 6 and not recording['errors']
    with np.load(next((tmp_path/'rgbd').glob('*.npz'))) as saved:
        metadata = json.loads(str(saved['metadata_json']))
        assert len(metadata['motor_q_rad']) == 29 and metadata['depth_unit'] == 'm'


def test_lamp_measurement_does_not_retarget_press(sensor):
    worker = ButtonPerceptionWorker(sensor.factory, sensor.transform)
    try:
        observation = worker.observe()
        reference = sensor.localizer.tracker.previous.copy()
        sensor.target = replace(sensor.target, face_xyz=(.353, -.25, 1.))
        sensor.level = 180
        after = time.monotonic()
        lamps = list(worker.lamp_frames(after))
        assert len(lamps) == 3 and len({f.frame_id for f in lamps}) == 3
        assert all(f.captured_at > after for f in lamps)
        assert np.all(lamps[0].image == 180)
        np.testing.assert_array_equal(sensor.localizer.tracker.previous, reference)
        assert len(worker.measurements) == 1 and worker.measurements[0]['sequence'] == observation.sequence
    finally:
        worker.close()


@pytest.mark.parametrize('fault', ['stale', 'duplicate', 'ambiguous'])
def test_no_stable_observation_from_stale_duplicate_or_ambiguous_images(sensor, fault):
    setattr(sensor, fault, True)
    worker = ButtonPerceptionWorker(sensor.factory, sensor.transform, recoverable=True)
    try:
        with pytest.raises(TrialPause, match='新しい3フレーム'):
            worker.observe(timeout_s=.7)
        assert not worker.measurements
        worker.check()  # 計測の不成立を恒久的な接続エラーにしない。
    finally:
        worker.close()


def test_completed_inference_from_previous_generation_is_discarded(sensor):
    started, release = threading.Event(), threading.Event()
    sensor.block = (started, release)
    worker = ButtonPerceptionWorker(sensor.factory, sensor.transform)
    try:
        worker._request('measure')
        assert started.wait(1.)
        worker.reset_measurement_reference()
        sensor.block = None
        release.set()
        observation = worker.observe()
        assert observation.sequence >= 3
        assert [row['generation'] for row in worker.diagnostics][-3:] == [3]*3
        assert len(worker.measurements) == 1
    finally:
        release.set()
        worker.close()


def test_camera_failure_is_fatal_instead_of_recoverable_measurement_pause(sensor):
    sensor.fail = True
    worker = ButtonPerceptionWorker(sensor.factory, sensor.transform, recoverable=True)
    with pytest.raises(RuntimeError, match='camera disconnected'):
        worker.observe()
    with pytest.raises(RuntimeError, match='camera disconnected'):
        worker.close()
    assert sensor.closed
