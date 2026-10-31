"""認識用RGB-Dを制御から独立した、容量制限のあるキューで保存する。"""
from __future__ import annotations

import json
from pathlib import Path
import queue
import threading

import numpy as np


class TrialRecorder:
    def __init__(self, directory, *, capacity=32):
        self.directory = Path(directory)
        # 前回の画像を混ぜない。再使用する場合は別のoutputを指定する。
        self.directory.mkdir(parents=True, exist_ok=False)
        self.queue = queue.Queue(maxsize=capacity)
        self.stop = threading.Event()
        self.saved = self.dropped = 0
        self.errors = []
        self.thread = threading.Thread(target=self._run, name='bottle-recording', daemon=True)
        self.thread.start()

    def submit(self, frame, captured_at, *, motor_q_rad=None, imu_rpy_rad=None):
        intr = frame.intrinsics
        metadata = {'sequence': int(frame.frame_id), 'captured_at_s': captured_at,
                    'source_timestamp': getattr(frame, 'timestamp', None),
                    'motor_q_rad': None if motor_q_rad is None else np.asarray(motor_q_rad).tolist(),
                    'imu_rpy_rad': None if imu_rpy_rad is None else np.asarray(imu_rpy_rad).tolist(),
                    'intrinsics': {k: float(getattr(intr, k)) for k in ('fx', 'fy', 'cx', 'cy')},
                    'depth_unit': 'm', 'color_order': 'BGR'}
        try:
            self.queue.put_nowait((frame.color_bgr.copy(), frame.depth_m().copy(), metadata))
        except queue.Full:
            self.dropped += 1

    def _run(self):
        while not self.stop.is_set() or not self.queue.empty():
            try:
                color, depth, metadata = self.queue.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                path = self.directory/f'{metadata["sequence"]:09d}.npz'
                with path.open('xb') as stream:
                    np.savez_compressed(stream, color_bgr=color, depth_m=depth,
                                        metadata_json=np.array(json.dumps(metadata, ensure_ascii=False)))
                self.saved += 1
            except Exception as exc:
                self.errors.append(str(exc))
            finally:
                self.queue.task_done()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=5.)
        if self.thread.is_alive():
            self.errors.append('RGB-D保存ワーカーの終了待ちが5秒を超えました')
        return {'directory': str(self.directory.resolve()), 'saved_frames': self.saved,
                'dropped_frames': self.dropped, 'errors': list(self.errors)}
