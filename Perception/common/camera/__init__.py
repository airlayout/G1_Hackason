from .base import FrameSource
from .video_file import VideoFileSource
from .webcam import WebcamSource

__all__ = ["FrameSource", "VideoFileSource", "WebcamSource", "build_frame_source"]


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

    raise ValueError(f"Unknown source.type: {source_type!r}")
