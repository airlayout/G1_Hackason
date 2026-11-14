from __future__ import annotations

import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

from Voice.onboard_voice import ContinuousListener, VoiceController
from Voice.segmenter import EnergySegmenter, Segment, SegmentConfig
from Voice.task_router import Task, TaskCatalog, TaskRouter
from Voice.tool_runtime import ToolResult, ToolRuntime, ToolSpec


class VoiceControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.clock = [10.0]
        self.encoder = Mock(model_name="test", revision="test")
        self.encoder.encode_audio.return_value = np.array([1, 0])
        catalog = TaskCatalog((Task("start", ("巡回開始",), "patrol_start"), Task("none", ("待つ",), None)), 0.75, 0.08)
        router = TaskRouter(catalog, np.eye(2))
        spec = ToolSpec("patrol_start", "巡回", "fake", {}, {"type": "object", "additionalProperties": False})
        self.backend = Mock(return_value=ToolResult("patrol_start", True, "completed", "ok"))
        runtime = ToolRuntime((spec,), {"fake": self.backend})
        self.client = Mock()
        self.client.call.return_value = {"success": True, "status": "completed"}
        self.controller = VoiceController(self.encoder, router, runtime, Path(self.tmp.name), self.client, clock=lambda: self.clock[0])
        self.segment = Segment(np.full(6400, 0.1, dtype=np.float32), 10)

    def test_expired_segment_is_discarded_before_model(self) -> None:
        self.clock[0] = 16
        self.assertEqual(self.controller.process(self.segment)["status"], "expired")
        self.encoder.encode_audio.assert_not_called()
        self.client.call.assert_not_called()

    def test_segment_expiring_during_inference_never_calls_tool(self) -> None:
        def encode(clip):
            self.clock[0] = 16
            return np.array([1, 0])
        self.encoder.encode_audio.side_effect = encode
        self.assertEqual(self.controller.process(self.segment)["status"], "expired")
        self.client.call.assert_not_called()

    def test_execution_and_cooldown_keep_vector_and_result(self) -> None:
        self.assertEqual(self.controller.process(self.segment)["status"], "completed")
        self.assertEqual(self.controller.process(self.segment)["status"], "cooldown")
        self.client.call.assert_called_once()
        self.backend.assert_not_called()
        records = list(Path(self.tmp.name).glob("*/result.json"))
        self.assertEqual(len(records), 2)
        self.assertEqual(json.loads(records[0].read_text())["model"], "test")

    def test_model_error_is_recorded(self) -> None:
        self.encoder.encode_audio.side_effect = RuntimeError("failed")
        with self.assertRaises(RuntimeError):
            self.controller.process(self.segment)
        record = json.loads(next(Path(self.tmp.name).glob("*/result.json")).read_text())
        self.assertEqual(record["status"], "error")


class ContinuousListenerTests(unittest.TestCase):
    def test_160ms_microphone_packets_form_complete_utterance(self) -> None:
        listener = ContinuousListener(Mock(client=None), EnergySegmenter(SegmentConfig()))
        for index in range(7):
            samples = np.full(2560, 0.2 if index < 3 else 0.0, dtype=np.float32)
            listener.feed(samples, 10 + index * 0.16)
        self.assertEqual(listener.pending.qsize(), 1)
        segment = listener.pending.get_nowait()
        self.assertAlmostEqual(len(segment.samples) / 16000, 0.5)

    def test_missing_160ms_packet_discards_partial_utterance(self) -> None:
        listener = ContinuousListener(Mock(client=None), EnergySegmenter(SegmentConfig()))
        listener.feed(np.full(2560, 0.2, dtype=np.float32), 10)
        listener.feed(np.full(2560, 0.2, dtype=np.float32), 10.16)
        for index in range(4):
            listener.feed(np.zeros(2560, dtype=np.float32), 10.48 + index * 0.16)
        self.assertTrue(listener.pending.empty())

    def test_capture_continues_and_queue_is_bounded_while_inference_blocks(self) -> None:
        entered, release = threading.Event(), threading.Event()
        controller = Mock(client=None)
        def process(segment):
            entered.set()
            release.wait(2)
        controller.process.side_effect = process
        listener = ContinuousListener(controller, EnergySegmenter(SegmentConfig()))
        listener.start()
        samples = np.concatenate((np.full(6400, 0.2), np.zeros(9600)))
        try:
            listener.feed(samples, time.monotonic())
            self.assertTrue(entered.wait(1))
            for _ in range(4):
                listener.feed(samples, time.monotonic())
            self.assertEqual(listener.pending.qsize(), 1)
            self.assertGreater(listener.last_packet, 0)
            self.assertTrue(listener.healthy(time.monotonic()))
            self.assertFalse(listener.healthy(time.monotonic() + 2))
        finally:
            release.set()
            listener.close()

    def test_gap_discards_partial_utterance(self) -> None:
        listener = ContinuousListener(Mock(client=None), EnergySegmenter(SegmentConfig()))
        listener.feed(np.full(6400, 0.2), 10)
        listener.feed(np.zeros(9600), 11)
        self.assertTrue(listener.pending.empty())
