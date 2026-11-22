"""Pepper producer for the Phase 3.6 latest-frame slot."""

from threading import Event, Thread
from time import perf_counter

from adapters.pepper.camera import PepperCameraSource
from .core import Frame


class PepperProducer(Thread):
    def __init__(self, ip, slot, *, fps=5, source_factory=PepperCameraSource):
        super().__init__(daemon=True)
        self.ip = ip
        self.slot = slot
        self.fps = fps
        self.source_factory = source_factory
        self.stop = Event()
        self.done = Event()
        self.ready = Event()
        self.error = None
        self.frames = 0
        self.read_failures = 0
        self.metadata = []

    def run(self):
        source = self.source_factory(self.ip, fps=self.fps)
        try:
            source.open()
            self.ready.set()
            generation = 0
            next_due = perf_counter()
            while not self.stop.is_set():
                self.stop.wait(max(0, next_due - perf_counter()))
                if self.stop.is_set():
                    break
                try:
                    camera = source.read()
                except Exception:
                    self.read_failures += 1
                    raise
                frame = Frame(
                    generation=generation,
                    source_timestamp=camera.pepper_timestamp_s,
                    capture_timestamp=camera.host_receive_monotonic,
                    decode_finished_timestamp=perf_counter(),
                    bgr=camera.bgr,
                )
                self.slot.publish(frame)
                self.metadata.append({
                    "generation": generation,
                    "pepper_timestamp_s": camera.pepper_timestamp_s,
                    "host_receive_timestamp": camera.host_receive_timestamp,
                    "host_receive_monotonic": camera.host_receive_monotonic,
                    "camera_latency_s": camera.acquisition_latency_s,
                    "payload_bytes": camera.payload_bytes,
                    "width": camera.width,
                    "height": camera.height,
                    "layers": camera.layers,
                    "colorspace": camera.colorspace,
                })
                self.frames += 1
                generation += 1
                # getImageRemote may return the current cached image immediately;
                # the requested camera FPS does not itself pace client polling.
                next_due += 1 / self.fps
                if next_due < perf_counter() - 1 / self.fps:
                    next_due = perf_counter()
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.ready.set()
        finally:
            try:
                source.close()
            except Exception as exc:
                if self.error is None:
                    self.error = f"unsubscribe failed: {type(exc).__name__}: {exc}"
            self.done.set()
