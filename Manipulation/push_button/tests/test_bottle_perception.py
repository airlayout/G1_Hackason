"""Mailbox concurrency, exposure matching and CPU scheduling without physics."""
from pathlib import Path
import hashlib
import json
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bottle import BottleObservation, estimate_bottle_target
from bottle_perception import PerceptionEngine, PerceptionMailbox, TableEstimate, ThreadedPerception
from pose_history import PoseHistory
from test_bottle import Camera, INTRINSICS, SETTINGS, TRANSFORM, cylinder_image, target_at


def mailbox(clock):
    engine = PerceptionEngine(lambda: None, clock, SETTINGS, clock.now)
    return PerceptionMailbox(engine, clock.now, lambda duration: None)


def test_previous_phase_completion_and_reordered_frames_cannot_satisfy_measurement():
    clock = Camera()
    box = mailbox(clock)
    box.request('measure')
    generation, request, _ = box.take_request()
    box.engine.configure(request)
    old = BottleObservation(target_at(), .1, 1)
    box.request('measure')
    box.publish(generation, old, None)
    assert box.latest is None
    localizer = clock.localizer()
    localizer.begin(0.)
    frame = clock.read(0., 1.)
    assert localizer.consume(frame) is None
    assert localizer.consume(frame) is None
    assert not localizer.samples
    assert localizer.rejections['古い時刻または重複フレーム'] == 1


def test_stale_tracking_and_stale_table_are_rejected_even_with_saved_results():
    clock = Camera()
    box = mailbox(clock)
    box.latest = BottleObservation(target_at(), 0., 1)
    box.table = TableEstimate(0., (.001, 0., 0.), (0., 0., 0.))
    assert box.latest_tracking(0.).sequence == 1
    clock.t = .501
    with pytest.raises(RuntimeError, match='追跡が古すぎる'):
        box.latest_tracking(.01)
    with pytest.raises(RuntimeError, match='新しい台基準'):
        box.table_offset()
    assert box.table_offset(strict=False) == pytest.approx((.001, 0., 0.))
    with pytest.raises(RuntimeError):
        TableEstimate(2., (0.,)*3, (0.,)*3).predict(1.)


def test_worker_failure_is_not_cleared_by_new_phase_or_old_generation():
    clock = Camera()
    box = mailbox(clock)
    box.request('measure')
    generation = box.generation
    box.stop_tracking()
    box.publish(generation, None, None, RuntimeError('camera disconnected'))
    box.request('measure')
    with pytest.raises(RuntimeError, match='camera disconnected'):
        box.check()


def test_inference_does_not_hold_control_mailbox_lock():
    entered, unblock = threading.Event(), threading.Event()

    class BlockedEngine:
        next_frame = -np.inf

        def step(self, progress):
            entered.set()
            assert unblock.wait(2.)
            return None, None

        def close(self):
            pass

    box = ThreadedPerception(BlockedEngine(), time.monotonic, time.sleep)
    try:
        assert entered.wait(1.)
        with box.lock:
            box.latest = BottleObservation(target_at(), time.monotonic(), 1)
        results = []
        reader = threading.Thread(target=lambda: results.append(box.latest_tracking(0.)))
        # Supply only the settings accessed by latest_tracking, without a detector.
        box.engine.localizer = SimpleNamespace(settings=SETTINGS)
        reader.start()
        reader.join(timeout=.5)
        assert not reader.is_alive(), 'Control waited for blocked inference'
        assert len(results) == 1
    finally:
        unblock.set()
        box.close()


def test_engine_limits_geometry_and_detection_rates_and_keeps_three_frame_gate():
    clock = Camera()
    reads, detects = [], []

    def frame():
        reads.append(clock.t)
        return clock.read(0., 10.)

    def detect(image):
        detects.append(clock.t)
        return clock.detect(image)

    engine = PerceptionEngine(frame, SimpleNamespace(detect=detect), SETTINGS, clock.now)
    engine.configure(('measure', 0., None, None, False))
    outputs = []
    for t in np.arange(0., .81, .02):
        clock.t = float(t)
        result, _ = engine.step()
        if result is not None:
            outputs.append(result)
    assert len(reads) <= 9
    assert len(detects) <= 5
    assert outputs and outputs[0].sequence >= 3
    assert min(np.diff(reads)) >= .1-1e-9


def test_upper_cylinder_band_can_track_when_contact_height_is_occluded():
    depth, bbox = cylinder_image()
    vv, _ = np.mgrid[:depth.shape[0], :depth.shape[1]]
    world_height = .90-(vv-120)*depth/300.
    occluded = depth.copy()
    # A near surface hides the lower cylinder; it is not missing depth.
    occluded[world_height < .855] = 1.2
    with pytest.raises(ValueError):
        estimate_bottle_target(occluded, bbox, INTRINSICS, TRANSFORM, SETTINGS)
    target = estimate_bottle_target(occluded, bbox, INTRINSICS, TRANSFORM, SETTINGS, allow_occlusion=True)
    assert target.center_xy == pytest.approx((.37, -.25), abs=.001)
    assert target.radius_m == pytest.approx(.034, abs=.001)


def test_pose_history_does_not_substitute_a_distant_state_or_interpolate_across_a_gap():
    history = PoseHistory()
    state = (np.zeros(29), np.zeros(3), 1.)
    with pytest.raises(ValueError, match='状態履歴'):
        history.at(1., state, max_distance_s=.1)
    history.append(state)
    with pytest.raises(ValueError, match='状態履歴'):
        history.at(.5, state, max_distance_s=.1)
    history.append((np.ones(29), np.zeros(3), 2.))
    with pytest.raises(ValueError, match='空白'):
        history.at(1.5, state, max_distance_s=.1)


def test_virtual_sensor_results_are_visible_only_after_cost_has_elapsed():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'sim'))
    from run_realistic_bottle import VirtualPerception, World
    clock = Camera()
    world = SimpleNamespace(now=clock.now, perception_busy=False, perception_cost=0.,
        options=SimpleNamespace(timing_mode='measured'), costs=[], origin=0.,
        advance=lambda seconds: pytest.fail('Sensor work blocked the control clock'))
    world.charge = lambda stage, duration: World.charge(world, stage, duration)

    class Engine:
        next_frame = -np.inf
        localizer = SimpleNamespace(settings=SETTINGS)

        def step(self, progress):
            self.next_frame = 1.
            world.charge('detector', .3)
            return BottleObservation(target_at(), 0., 1), None

        def close(self):
            pass

    box = VirtualPerception(Engine(), clock.now, lambda seconds: None, world)
    box.tick()
    assert clock.t == 0. and box.latest is None
    clock.t = .2
    box.tick()
    assert box.latest is None
    clock.t = .3
    box.tick()
    assert box.latest.sequence == 1
    assert len(world.costs) == 1
    assert world.costs[0]['worker'] == 'perception'
    box.close()


def test_replay_uses_saved_cost_and_refuses_exhausted_or_invalid_timing():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'sim'))
    from run_realistic_bottle import World
    advances = []
    world = SimpleNamespace(now=lambda: 1., origin=0., options=SimpleNamespace(timing_mode='replay'),
        replay_indices={}, replay_costs={'detector': [.04]}, costs=[], perception_busy=False,
        perception_cost=0., advance=advances.append)
    World.charge(world, 'detector', .9)
    assert advances == [.04]
    assert world.costs[0]['measured_duration_s'] == .9
    with pytest.raises(RuntimeError, match='不足'):
        World.charge(world, 'detector', .1)
    world.replay_costs['table_reference'] = [float('nan')]
    with pytest.raises(ValueError, match='有限値'):
        World.charge(world, 'table_reference', .1)


def test_head_profile_uses_head_mount_and_pelvis_pose_instead_of_target_viewpoint():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'sim'))
    from run_realistic_bottle import camera_profile_geometry
    waist = []
    head = SimpleNamespace(pelvis_from_optical=lambda q: (waist.append(q.copy()) or np.eye(3), np.array([.05, .02, .6])))
    runtime = SimpleNamespace(module=lambda name: SimpleNamespace(
        load_config=lambda _: {'head_camera': {'fovy_deg': 42.6}}, HeadCameraTransform=lambda _: head))
    model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0))
    data = SimpleNamespace(xmat=np.eye(3)[None, :, :], xpos=np.array([[0., 0., .79]]))
    rest = np.arange(29)*.001
    optical, fovy = camera_profile_geometry(model, data, rest, 'head', runtime)
    assert optical[:3, 3] == pytest.approx((.05, .02, 1.39))
    assert waist[0] == pytest.approx(rest[12:15])
    assert fovy == 42.6


def test_virtual_cost_is_included_in_post_inference_freshness_check():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'sim'))
    from run_realistic_bottle import VirtualPerception, World
    clock = Camera()
    world = SimpleNamespace(now=clock.now, perception_busy=False, perception_cost=0., origin=0.,
        options=SimpleNamespace(timing_mode='measured'), costs=[], advance=lambda _: pytest.fail('blocked'))
    world.charge = lambda stage, duration: World.charge(world, stage, duration)

    def detect(image):
        world.charge('detector', .6)
        return clock.detect(image)

    engine = PerceptionEngine(lambda: clock.read(0., 5.), SimpleNamespace(detect=detect), SETTINGS, clock.now)
    box = VirtualPerception(engine, clock.now, lambda _: None, world)
    box.request('measure')
    box.tick()
    assert engine.localizer.rejections['処理後の画像鮮度超過'] == 1
    assert not engine.localizer.measurements and box.latest is None
    box.close()


def test_replay_requires_matching_settings_sources_and_assets(monkeypatch, tmp_path):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'sim'))
    import run_realistic_bottle as harness
    monkeypatch.setattr(harness, 'REPO', tmp_path)
    source, model, weights = [tmp_path/name for name in ('source.py', 'model.xml', 'weights.pt')]
    for path in (source, model, weights):
        path.write_bytes(b'test data')
    options = SimpleNamespace(timing_mode='replay', timing_log=tmp_path, seeds=[0], model=model, weights=weights,
        camera_profile='head', profile='calibrated', fault='none', gravity_scale=1., detector_imgsz=960,
        preview_step_m=.005, track_table=True, dynamics_feedforward=True, stop_before_contact=False)
    digest = hashlib.sha256(b'test data').hexdigest()
    manifest = {'options': {k: v for k, v in vars(options).items() if k not in ('timing_log', 'model', 'weights')},
                'source_sha256': {'source.py': digest}, 'model_sha256': digest, 'weights_sha256': digest}
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    (tmp_path/'seed_0').mkdir()
    (tmp_path/'seed_0/processing_times.json').write_text('[]')
    harness.validate_replay(options, ['source.py'])
    options.camera_profile = 'legacy'
    with pytest.raises(ValueError, match='camera_profile'):
        harness.validate_replay(options, ['source.py'])
    options.camera_profile = 'head'
    source.write_text('changed')
    with pytest.raises(ValueError, match='ソース'):
        harness.validate_replay(options, ['source.py'])
