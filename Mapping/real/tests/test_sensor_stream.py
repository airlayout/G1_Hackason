from __future__ import annotations

import ast
import io
import sys
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1] / "ros2_ws/src/g1_mapping_tools"
sys.path.insert(0, str(TOOLS))
from g1_mapping_tools.sensor_stream import (
    HEADER,
    IMU,
    MAGIC,
    MAX_PAYLOAD,
    POINTS,
    read_frame,
    write_frame,
)


class FragmentedIO(io.BytesIO):
    def read(self, size=-1):
        return super().read(min(size, 3))

    def write(self, data):
        return super().write(data[:7])


class SensorStreamTest(unittest.TestCase):
    def test_fragmented_stream_preserves_cdr_and_message_order(self):
        samples = [
            (POINTS, bytes(range(256)) * 2048),
            (IMU, b"\x00\x01\x00\x00\xff\x00" * 50),
        ]
        stream = FragmentedIO()
        for kind, payload in samples:
            write_frame(stream, kind, payload)
        stream.seek(0)
        for expected in samples:
            self.assertEqual(read_frame(stream), expected)
        self.assertIsNone(read_frame(stream))

    def test_truncation_is_reported_instead_of_publishing_partial_cdr(self):
        for contents in (
            MAGIC,
            HEADER.pack(MAGIC, POINTS, 10),
            HEADER.pack(MAGIC, IMU, 10) + b"short",
        ):
            with self.subTest(contents=contents), self.assertRaises(EOFError):
                read_frame(io.BytesIO(contents))

    def test_unknown_frames_and_oversized_payloads_are_rejected(self):
        for magic, kind, size in (
            (b"oops", POINTS, 10),
            (MAGIC, 9, 10),
            (MAGIC, IMU, MAX_PAYLOAD + 1),
            (MAGIC, IMU, 0),
        ):
            with self.subTest(kind=kind, size=size), self.assertRaises(ValueError):
                read_frame(io.BytesIO(HEADER.pack(magic, kind, size)))

    def test_remote_script_parses_on_the_g1_python_version(self):
        source = TOOLS / "g1_mapping_tools/sensor_stream.py"
        ast.parse(source.read_text(), feature_version=(3, 8))


if __name__ == "__main__":
    unittest.main()
