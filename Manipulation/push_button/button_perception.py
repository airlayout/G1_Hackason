"""頭RGB-Dの単独所有ワーカー。制御は計測世代ごとの完成した結果だけを読む。"""
from collections import Counter, deque
import threading
import time

import numpy as np

from button_feedback import LampFrame
from trial_policy import TrialPause


class ButtonPerceptionWorker:
    def __init__(self, factory, frame_transform, *, now=time.monotonic, wait=time.sleep,
                 recoverable=False, recorder=None):
        self.factory, self.frame_transform = factory, frame_transform
        self.now, self.wait = now, wait
        self.recoverable, self.recorder = recoverable, recorder
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.generation = 0
        self.request = None
        self.result = self.error = None
        self.lamps = deque(maxlen=3)
        self.last_lamp_frame = None
        self.last_measurement_frame_ids = ()
        self.measurements, self.processing_times, self.diagnostics = [], [], []
        self.rejections = Counter()
        self.thread = threading.Thread(target=self._run, name='button-perception', daemon=True)
        self.thread.start()

    def check(self):
        with self.lock:
            error = self.error
        if error is not None:
            raise RuntimeError(f'画像処理で中止: {error}') from error

    def _request(self, mode, after=None):
        with self.lock:
            self.generation += 1
            self.request = (mode, self.now() if after is None else after)
            self.result = None
            self.lamps.clear()
            return self.generation

    def observe(self, timeout_s=5.):
        started = self.now()
        generation = self._request('measure')
        while self.now()-started < timeout_s:
            self.check()
            with self.lock:
                result = self.result
            if result is not None and result[0] == generation:
                _, observation, frame, frame_ids = result
                if 0 <= self.now()-observation.captured_at <= .5:
                    self.last_lamp_frame = frame
                    self.last_measurement_frame_ids = frame_ids
                    return observation
            self.wait(.01)
        self.rejections['安定計測の期限超過'] += 1
        error = TrialPause if self.recoverable else RuntimeError
        raise error('安定した新しい3フレームのボタン計測を取得できません')

    def lamp_frames(self, after, timeout_s=2.):
        generation = self._request('lamp', after=max(after, self.now()))
        deadline = self.now()+timeout_s
        count = 0
        while self.now() < deadline:
            self.check()
            with self.lock:
                frame = self.lamps.popleft() if self.generation == generation and self.lamps else None
            if frame is not None:
                count += 1
                yield frame
                if count == 3:
                    return
            else:
                self.wait(.01)
        raise RuntimeError('対象ボタンの点灯確認用画像を取得できません')

    def reset_measurement_reference(self):
        # 対象の同一性を保ち、前回の完成結果と安定フレームだけを破棄する。
        self._request('idle')

    def _run(self):
        localizer = None
        mode, after, generation = 'idle', -np.inf, -1
        next_frame, next_detection = -np.inf, -np.inf
        last_id, last_stamp = -1, -np.inf
        lamp_count = 0
        sample_ids = deque(maxlen=3)
        try:
            localizer = self.factory()
            while not self.stop.is_set():
                with self.lock:
                    current_generation, request = self.generation, self.request
                if current_generation != generation:
                    generation = current_generation
                    mode, after = request or ('idle', self.now())
                    localizer.tracker.begin_measurement()
                    sample_ids.clear()
                    lamp_count = 0
                    next_detection = -np.inf
                if self.now() < next_frame:
                    self.stop.wait(.005)
                    continue
                next_frame = self.now()+.1
                started = time.perf_counter()
                try:
                    delivered = localizer.camera.poll()
                    if delivered is None:
                        continue
                    frame, stamp = delivered
                    if frame.frame_id <= last_id or stamp <= last_stamp or not 0 <= self.now()-stamp <= .5:
                        self.rejections['古い画像または重複'] += 1
                        continue
                    last_id, last_stamp = frame.frame_id, stamp
                    transform, q, imu = self.frame_transform(frame, stamp)
                    if self.recorder is not None:
                        self.recorder.submit(frame, stamp, motor_q_rad=q, imu_rpy_rad=imu)
                    if mode == 'idle' or stamp <= after or self.now() < next_detection:
                        continue
                    next_detection = self.now()+.2
                    candidates = localizer._candidates(frame, transform=transform)
                    if not 0 <= self.now()-stamp <= .5:
                        localizer.tracker.samples.clear()
                        sample_ids.clear()
                        self.rejections['推論後の画像失効'] += 1
                        continue
                    if len(candidates) != 1:
                        localizer.tracker.samples.clear()
                        sample_ids.clear()
                        self.rejections[f'候補数={len(candidates)}'] += 1
                        continue
                    target, bbox = candidates[0]
                    lamp = LampFrame(frame.color_bgr.copy(), bbox, stamp, frame.frame_id)
                    observation = None
                    if mode == 'measure':
                        observation = localizer.tracker.update([target], stamp)
                        sample_ids.append(frame.frame_id)
                    self.diagnostics.append({'frame_id': frame.frame_id, 'captured_at_s': stamp,
                                             'generation': generation, 'mode': mode})
                    with self.lock:
                        if generation != self.generation:
                            continue
                        if observation is not None and self.result is None:
                            self.result = (generation, observation, lamp, tuple(sample_ids))
                            self.measurements.append({'sequence': observation.sequence, 'captured_at_s': stamp,
                                'frame_id': frame.frame_id, 'frame_ids': list(sample_ids),
                                'face_xyz_m': list(observation.target.face_xyz),
                                'press_direction': list(observation.target.press_direction)})
                            mode = 'idle'
                        elif mode == 'lamp' and lamp_count < 3:
                            # lampモードではtrackerの目標を更新しない。
                            self.lamps.append(lamp)
                            lamp_count += 1
                            if lamp_count == 3:
                                mode = 'idle'
                except ValueError as exc:
                    self.rejections[str(exc)] += 1
                    localizer.tracker.samples.clear()
                    sample_ids.clear()
                finally:
                    self.processing_times.append({'stage': 'button_perception', 'time_s': self.now(),
                                                   'duration_s': time.perf_counter()-started})
        except Exception as exc:
            with self.lock:
                self.error = exc
        finally:
            if localizer is not None:
                try:
                    localizer.close()
                except Exception as exc:
                    with self.lock:
                        self.error = exc

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            raise RuntimeError('画像処理ワーカーが終了しません')
        self.check()
