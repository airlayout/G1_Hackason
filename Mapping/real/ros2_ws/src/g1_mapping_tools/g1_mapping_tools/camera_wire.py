"""Camera wire helpers usable on G1 Python 3.8 without ROS."""
import base64
import json
import struct


def decode_lerobot_message(wire_data, camera_name):
    payload = json.loads(wire_data.decode("utf-8"))
    images, timestamps = payload.get("images", {}), payload.get("timestamps", {})
    if camera_name not in images or camera_name not in timestamps:
        return None
    return (base64.b64decode(images[camera_name], validate=True),
            int(float(timestamps[camera_name]) * 1_000_000_000))


class Cdr:
    """ROS 2 CDR1 little-endian writer; alignment excludes encapsulation."""
    def __init__(self):
        self.data = bytearray(b"\x00\x01\x00\x00")

    def pack(self, fmt, value, alignment=4):
        self.data.extend(b"\x00" * (-(len(self.data) - 4) % alignment))
        self.data.extend(struct.pack("<" + fmt, value))

    def string(self, value):
        encoded = value.encode("utf-8") + b"\x00"
        self.pack("I", len(encoded))
        self.data.extend(encoded)

    def header(self, stamp, frame):
        self.pack("i", stamp // 1_000_000_000)
        self.pack("I", stamp % 1_000_000_000)
        self.string(frame)


def camera_records(jpeg, stamp, config, sequence):
    """Serialize CompressedImage, CameraInfo and String without a ROS install."""
    frame = config.get("camera_frame_id", "camera_color_optical_frame")
    image = Cdr()
    image.header(stamp, frame)
    image.string("rgb8; jpeg compressed rgb8")
    image.pack("I", len(jpeg))
    image.data.extend(jpeg)
    info = Cdr()
    info.header(stamp, frame)
    width, height = int(config.get("camera_width", 0)), int(config.get("camera_height", 0))
    # Read actual dimensions, including when camera calibration is not configured.
    index = 2
    while jpeg[:2] == b"\xff\xd8" and index + 4 <= len(jpeg):
        if jpeg[index] != 255:
            index += 1
            continue
        marker = jpeg[index + 1]
        length = int.from_bytes(jpeg[index + 2:index + 4], "big")
        if marker in {192, 193, 194, 195, 197, 198, 199, 201, 202, 203, 205, 206, 207}:
            height = int.from_bytes(jpeg[index + 5:index + 7], "big")
            width = int.from_bytes(jpeg[index + 7:index + 9], "big")
            break
        if length < 2:
            break
        index += 2 + length
    info.pack("I", height)
    info.pack("I", width)
    info.string("plumb_bob")
    info.pack("I", 5)
    fx, fy, cx, cy = [float(config.get("camera_" + key, 0)) for key in ("fx", "fy", "cx", "cy")]
    for value in ([0.] * 5 + [fx, 0., cx, 0., fy, cy, 0., 0., 1.] +
                  [1., 0., 0., 0., 1., 0., 0., 0., 1.] +
                  [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]):
        info.pack("d", value, 8)
    for _ in range(6):  # binning_x/y, ROI x/y/height/width
        info.pack("I", 0)
    info.pack("?", False, 1)
    metadata = Cdr()
    metadata.string(json.dumps({"schema_version": 1, "sequence": sequence,
                               "stamp_ns": stamp, "timestamp_source": "lerobot_camera_capture_realtime",
                               "camera_name": config.get("camera_name", "head_camera"),
                               "width": width, "height": height,
                               "calibration_complete": fx > 0 and fy > 0}))
    return bytes(image.data), bytes(info.data), bytes(metadata.data)
