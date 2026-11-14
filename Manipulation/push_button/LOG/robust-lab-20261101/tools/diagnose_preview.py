import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'Manipulation/push_button/real'), str(ROOT/'Manipulation/push_button')]


def main():
    import robust_push
    import robust_planner
    events = []
    last_frame = []
    origin = time.monotonic()

    def timed(name, function):
        def call(*args, **kwargs):
            started = time.monotonic()
            event = dict(name=name, start_s=started-origin)
            events.append(event)
            try:
                result = function(*args, **kwargs)
                if name == 'poll' and result is not None:
                    event['image_age_s'] = time.monotonic()-result[1]
                    event['frame_id'] = result[0].frame_id
                    last_frame[:] = [result[0]]
                if name == 'estimate_bottle':
                    event['geometry'] = result
                return result
            except Exception as exc:
                event['error'] = str(exc)
                raise
            finally:
                event['duration_s'] = time.monotonic()-started
        return call

    robust_planner.build_scene_model = timed('build_scene_model', robust_planner.build_scene_model)
    robust_planner.generate_candidate = timed('generate_candidate', robust_planner.generate_candidate)
    robust_push.estimate_bottle = timed('estimate_bottle', robust_push.estimate_bottle)
    robust_push.J1RgbdCamera.poll = timed('poll', robust_push.J1RgbdCamera.poll)
    runtime = robust_push.J1CameraRuntime()
    detector = runtime.module('perception_bridge').perception('detector.yolo_detector').YoloDetector
    detector.detect = timed('detect', detector.detect)
    original = robust_push.ContinuousObserver.latest

    def latest(self):
        try:
            return original(self)
        except Exception:
            if self.value is not None:
                events.append(dict(name='latest_failure', start_s=time.monotonic()-origin,
                                   age_s=self.now()-self.value.captured_at,
                                   thread_alive=self.thread.is_alive(), error=str(self.error),
                                   frame_id=self.value.frame_id,
                                   window=[dict(frame_id=v.frame_id, stamp=v.captured_at,
                                                geometry=v.geometry) for v in self.window.samples]))
            raise

    robust_push.ContinuousObserver.latest = latest
    session = Path(__file__).resolve().parent
    output = session/time.strftime('diagnostic_%Y%m%d_%H%M%S')
    output.mkdir()
    print(output, flush=True)
    try:
        return robust_push.main(['--task', 'bottle', '--motion-planner', 'robust',
            '--trial-config', str(session/'trial.json'), '--state-json', sys.argv[1],
            '--output', str(output/'result.json')])
    finally:
        (output/'timing.json').write_text(json.dumps(events, indent=2)+'\n')
        if last_frame:
            import numpy as np
            import cv2
            frame = last_frame[0]
            np.savez_compressed(output/'rgbd.npz', color_bgr=frame.color_bgr, depth_m=frame.depth_m())
            cv2.imwrite(str(output/'camera.jpg'), frame.color_bgr)
            (output/'camera.json').write_text(json.dumps(dict(intrinsics=vars(frame.intrinsics)), indent=2)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
