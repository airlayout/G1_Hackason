"""Pepper-specific adapters. Importing this package does not import qi."""

from .camera import CameraFrame, PepperCameraError, PepperCameraSource, decode_remote_image
from .motion import (
    DryRunTransport,
    HttpBridgeTransport,
    MotionValidationError,
    PepperMotionAdapter,
    PepperMotionError,
)

__all__ = [
    "CameraFrame", "PepperCameraError", "PepperCameraSource", "decode_remote_image",
    "DryRunTransport", "HttpBridgeTransport", "MotionValidationError",
    "PepperMotionAdapter", "PepperMotionError",
]
