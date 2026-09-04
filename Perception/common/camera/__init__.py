from .base import FrameSource
from .video_file import VideoFileSource
from .webcam import WebcamSource
from .zmq_camera import ZmqFrameSource

__all__ = [
    "FrameSource",
    "VideoFileSource",
    "WebcamSource",
    "ZmqFrameSource",
    "build_frame_source",
]


def build_frame_source(config: dict) -> FrameSource:
    """config["source"] の内容から適切な FrameSource を組み立てる"""
    source_cfg = config["source"]
    source_type = source_cfg["type"]

    if source_type == "webcam":
        webcam_cfg = source_cfg.get("webcam", {})
        return WebcamSource(device_index=webcam_cfg.get("device_index", 0))

    if source_type == "video":
        video_cfg = source_cfg.get("video", {})
        return VideoFileSource(
            path=video_cfg["path"],
            loop=video_cfg.get("loop", False),
        )

    if source_type == "zmq":
        zmq_cfg = source_cfg.get("zmq", {})
        return ZmqFrameSource(
            server_address=zmq_cfg["server_address"],
            port=zmq_cfg.get("port", 5555),
            camera_name=zmq_cfg.get("camera_name", "head_camera"),
            timeout_ms=zmq_cfg.get("timeout_ms", 5000),
        )

    raise ValueError(f"Unknown source.type: {source_type!r}")
