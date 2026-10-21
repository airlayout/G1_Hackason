"""Exercise the existing camera wire format without importing a press backend."""
import json
import struct
import zlib

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2", reason="optional camera decoder requires OpenCV")

from Navigation.arrival.collect import decode_rgbd


def wire(encoding="zlib", **depth_overrides):
    image = np.full((20, 24, 3), [5, 40, 230], np.uint8)  # BGR
    ok, jpg = cv2.imencode(".jpg", image)
    assert ok
    depth = np.full((20, 24), 1234, "<u2")
    raw = depth.tobytes()
    raw = zlib.compress(raw) if encoding == "zlib" else raw
    header = {"version": 1, "camera": "head_camera", "frame_id": 4, "timestamp": 100.,
              "color": {"encoding": "jpeg", "order": "bgr", "nbytes": jpg.size, "width": 24, "height": 20},
              "depth": {"encoding": encoding, "dtype": "uint16le", "aligned_to": "color",
                        "width": 24, "height": 20, "scale_m": .001, "nbytes": len(raw), **depth_overrides},
              "intrinsics": {"width": 24, "height": 20, "fx": 20., "fy": 20., "cx": 12., "cy": 10., "coeffs": [0.]*5}}
    text = json.dumps(header).encode()
    return b"G1RGBD1\0"+struct.pack("<I", len(text))+text+jpg.tobytes()+raw


@pytest.mark.parametrize("encoding", ["none", "zlib"])
def test_rgbd_channel_order_depth_scale_and_intrinsics(encoding):
    header, rgb, depth, K = decode_rgbd(wire(encoding))
    assert rgb[0, 0, 0] > rgb[0, 0, 2]  # decoded BGR became RGB
    assert depth == pytest.approx(np.full((20, 24), 1.234), abs=1e-6)
    assert K[0, 0] == 20.
    assert header["timestamp"] == 100.


@pytest.mark.parametrize("payload", [b'{"images":{}}', wire()[:-3], wire()+b"extra",
                                     wire(aligned_to="depth"), wire(scale_m=-.01)])
def test_wrong_truncated_misaligned_or_invalid_scale_is_rejected(payload):
    with pytest.raises(ValueError):
        decode_rgbd(payload)
