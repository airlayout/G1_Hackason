from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import soundfile as sf

from Voice.audio_input import load_audio, prepare_audio, record_audio


class AudioInputTests(unittest.TestCase):
    def test_stereo_48khz_becomes_mono_16khz(self) -> None:
        stereo = np.column_stack((np.full(24_000, 0.2), np.full(24_000, 0.4)))
        clip = prepare_audio(stereo, 48_000)
        self.assertEqual(clip.sample_rate, 16_000)
        self.assertEqual(clip.samples.shape, (8_000,))
        self.assertEqual(clip.samples.dtype, np.float32)
        np.testing.assert_allclose(clip.samples[100:-100], 0.3, atol=1e-4)

    def test_load_wav_resamples(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "input.wav"
            sf.write(path, np.full(22_050, 0.2), 44_100)
            clip = load_audio(path)
        self.assertAlmostEqual(clip.duration, 0.5)

    def test_invalid_or_silent_audio_is_rejected(self) -> None:
        for samples, rate in (
            (np.array([]), 16_000), (np.zeros(4_000), 16_000),
            (np.full(4_000, np.nan), 16_000), (np.ones(100), 16_000),
            (np.ones(16_000 * 31), 16_000), (np.ones(4_000), 0),
        ):
            with self.subTest(rate=rate, shape=samples.shape), self.assertRaises(ValueError):
                prepare_audio(samples, rate)

    def test_long_file_is_rejected_before_reading(self) -> None:
        with patch("Voice.audio_input.sf.info") as info, patch("Voice.audio_input.sf.read") as read:
            info.return_value.duration = 31
            with self.assertRaises(ValueError):
                load_audio(Path("long.wav"))
            read.assert_not_called()

    def test_invalid_record_duration_is_rejected_before_opening_mic(self) -> None:
        for duration in (-1.0, 0.0, float("nan"), float("inf"), 31.0):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                record_audio(duration)
