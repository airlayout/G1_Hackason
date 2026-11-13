from __future__ import annotations

import unittest

import numpy as np

from Voice.g1_mic import decode_pcm
from Voice.segmenter import EnergySegmenter, SegmentConfig


class SegmenterTests(unittest.TestCase):
    def waveform(self, *parts: tuple[int, float]) -> np.ndarray:
        return np.concatenate([np.full(ms * 16, value, dtype=np.float32) for ms, value in parts])

    def test_arbitrary_packet_sizes_preserve_onset_and_end_after_silence(self) -> None:
        segmenter = EnergySegmenter(SegmentConfig())
        samples = self.waveform((400, 0), (400, 0.2), (600, 0))
        segments = []
        for offset in range(0, len(samples), 713):
            segments += segmenter.feed(samples[offset:offset + 713], (offset + 713) / 16000)
        self.assertEqual(len(segments), 1)
        clip = segments[0].samples
        self.assertEqual(np.count_nonzero(clip), 400 * 16)
        self.assertEqual(np.argmax(clip != 0), 300 * 16)
        self.assertLessEqual(len(clip), 740 * 16)

    def test_short_burst_does_not_form_command(self) -> None:
        segmenter = EnergySegmenter(SegmentConfig())
        self.assertEqual(segmenter.feed(self.waveform((120, 0.2), (700, 0)), 1), [])

    def test_short_pause_does_not_split_utterance(self) -> None:
        segmenter = EnergySegmenter(SegmentConfig())
        clips = segmenter.feed(self.waveform((300, 0.2), (200, 0), (300, 0.2), (600, 0)), 2)
        self.assertEqual(len(clips), 1)

    def test_long_sound_is_discarded_until_quiet_and_next_command_works(self) -> None:
        cfg = SegmentConfig(start_ms=40, end_ms=100, pre_roll_ms=40, min_voiced_ms=60, max_ms=1000)
        segmenter = EnergySegmenter(cfg)
        self.assertEqual(segmenter.feed(self.waveform((2500, 0.2)), 3), [])
        self.assertTrue(segmenter.blocked)
        self.assertEqual(segmenter.feed(self.waveform((200, 0)), 3.2), [])
        clips = segmenter.feed(self.waveform((300, 0.2), (100, 0)), 3.6)
        self.assertEqual(len(clips), 1)

    def test_invalid_audio_clears_partial_utterance(self) -> None:
        segmenter = EnergySegmenter(SegmentConfig())
        segmenter.feed(self.waveform((400, 0.2)), 1)
        with self.assertRaises(ValueError):
            segmenter.feed(np.array([np.nan]), 1.1)
        self.assertEqual(segmenter.feed(self.waveform((600, 0)), 2), [])

    def test_invalid_configuration_is_rejected(self) -> None:
        for kwargs in ({"frame_ms": 0}, {"threshold_dbfs": float("nan")}, {"max_ms": 100}, {"sample_rate": 8000}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                SegmentConfig(**kwargs)

    def test_pcm_is_little_endian_normalized_and_malformed_packet_is_rejected(self) -> None:
        np.testing.assert_allclose(decode_pcm(b"\x00\x80\x00\x00\xff\x7f"), [-1, 0, 32767 / 32768])
        for data in (b"", b"\x01"):
            with self.assertRaises(ValueError):
                decode_pcm(data)
