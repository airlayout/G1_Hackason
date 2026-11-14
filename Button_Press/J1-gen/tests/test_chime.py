"""G1 のマイクの音を受けて、エレベーターの到着の音（高めのチャイム）を探す（common/chime.py、real/mic_record.py）。

実機の音はまだ録っていないので、ファンのような低い音と雑音に、高い音を混ぜた作りものの音で確かめる。
マイクのマルチキャストは、lo に模擬の送り手を立てて確かめる（lo のマルチキャストが使えない環境ではスキップ）。
"""

from __future__ import annotations

import json
import re
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from common.chime import (
    SAMPLE_RATE, Chunk, MicTimeout, detect_tones, level_dbfs, load_wav, open_mic_socket, receive, save_wav, to_samples,
)
from common.config import FEATURE_DIR

GROUP = "239.255.77.1"  # 試験用（G1 のマイクの 239.168.123.161 とは別）


def background(seconds: float, seed: int = 0) -> np.ndarray:
    """ファンのような低い音（120 Hz とその倍音）と、白い雑音。"""
    t = np.arange(int(seconds * SAMPLE_RATE)) / SAMPLE_RATE
    rng = np.random.default_rng(seed)
    hum = sum(0.02 / k * np.sin(2 * np.pi * 120 * k * t) for k in range(1, 6))
    return hum + 0.01 * rng.standard_normal(len(t))


def add_tone(x: np.ndarray, start: float, dur: float, hz: float, amp: float) -> np.ndarray:
    i, n = int(start * SAMPLE_RATE), int(dur * SAMPLE_RATE)
    t = np.arange(n) / SAMPLE_RATE
    env = np.minimum(1.0, np.minimum(t, dur - t) / 0.01)  # 端を 10 ms でなめらかに
    y = x.copy()
    y[i:i + n] += amp * env * np.sin(2 * np.pi * hz * t)
    return y


class TestDetectTones(unittest.TestCase):
    def test_finds_two_chimes(self) -> None:
        x = add_tone(add_tone(background(8.0), 2.0, 0.6, 2800.0, 0.05), 5.0, 0.4, 2200.0, 0.05)
        tones = detect_tones(x)
        self.assertEqual(len(tones), 2, tones)
        a, b = tones
        self.assertAlmostEqual(a.start_s, 2.0, delta=0.08)
        self.assertAlmostEqual(a.duration_s, 0.6, delta=0.12)
        self.assertAlmostEqual(a.peak_hz, 2800.0, delta=20.0)
        self.assertAlmostEqual(b.peak_hz, 2200.0, delta=20.0)
        self.assertGreater(a.above_db, 15.0)

    def test_background_only_has_none(self) -> None:
        self.assertEqual(detect_tones(background(8.0, seed=1)), [])

    def test_low_hum_outside_the_band_is_ignored(self) -> None:
        """帯（1〜6 kHz）の外の強い音（500 Hz）は数えない。"""
        self.assertEqual(detect_tones(add_tone(background(6.0), 2.0, 1.0, 500.0, 0.2)), [])

    def test_short_click_is_ignored(self) -> None:
        self.assertEqual(detect_tones(add_tone(background(6.0), 2.0, 0.03, 3000.0, 0.1)), [])

    def test_level(self) -> None:
        self.assertEqual(level_dbfs(np.zeros(100)), float("-inf"))
        self.assertAlmostEqual(level_dbfs(np.full(100, 0.1)), -20.0, places=6)


class TestWav(unittest.TestCase):
    def test_roundtrip(self) -> None:
        x = add_tone(background(1.0), 0.2, 0.3, 3000.0, 0.05)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "a.wav"
            save_wav(path, x)
            y, sr = load_wav(path)
        self.assertEqual(sr, SAMPLE_RATE)
        np.testing.assert_allclose(y, x, atol=1.0 / 32768)

    def test_chunks_to_samples(self) -> None:
        pcm = np.array([0, 16384, -32768], dtype="<i2").tobytes()
        np.testing.assert_allclose(to_samples([Chunk(0.0, pcm[:2]), Chunk(0.1, pcm[2:])]), [0.0, 0.5, -1.0])


def send_pcm(port: int, x: np.ndarray, stop: threading.Event, packet: int = 640) -> None:
    """lo のマルチキャストで、16 bit の音を packet バイトずつ実時間で送る（G1 の音声の基板の代わり）。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton("127.0.0.1"))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    data = np.clip(np.round(x * 32768), -32768, 32767).astype("<i2").tobytes()
    for i in range(0, len(data), packet):
        if stop.is_set():
            break
        s.sendto(data[i:i + packet], (GROUP, port))
        time.sleep(packet / 2 / SAMPLE_RATE)
    s.close()


def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class TestMulticast(unittest.TestCase):
    def test_receive_from_fake_mic(self) -> None:
        port = free_port()
        x = add_tone(background(1.5), 0.5, 0.4, 3000.0, 0.05)
        try:
            sock = open_mic_socket("127.0.0.1", GROUP, port, timeout_s=1.0)
        except OSError as e:
            self.skipTest(f"lo でマルチキャストに参加できない: {e}")
        stop = threading.Event()
        th = threading.Thread(target=send_pcm, args=(port, x, stop), daemon=True)
        th.start()
        try:
            chunks = receive(sock, 1.0)
        except MicTimeout:
            self.skipTest("lo のマルチキャストが届かない環境（WSL のミラーモードなど）")
        finally:
            stop.set()
            sock.close()
            th.join(3.0)
        y = to_samples(chunks)
        self.assertEqual(len(y), SAMPLE_RATE)  # 1 秒分で止まる
        np.testing.assert_allclose(y, x[:SAMPLE_RATE], atol=1.0 / 32768)
        self.assertTrue(all(b.t >= a.t for a, b in zip(chunks, chunks[1:])))

    def test_no_packets_times_out(self) -> None:
        try:
            sock = open_mic_socket("127.0.0.1", GROUP, free_port(), timeout_s=0.3)
        except OSError as e:
            self.skipTest(f"lo でマルチキャストに参加できない: {e}")
        with self.assertRaises(MicTimeout):
            receive(sock, 1.0)
        sock.close()

    def test_mic_record_script(self) -> None:
        """real/mic_record.py が WAV と .json（録り始めの時刻・パケットの数・見つけた高い音）を残す。"""
        port = free_port()
        x = add_tone(background(3.0), 1.0, 0.5, 2800.0, 0.05)
        stop = threading.Event()
        with tempfile.TemporaryDirectory() as tmp:
            th = threading.Thread(target=lambda: (time.sleep(0.5), send_pcm(port, x, stop)), daemon=True)
            th.start()
            r = subprocess.run([sys.executable, str(FEATURE_DIR / "real" / "mic_record.py"), "--local-ip", "127.0.0.1",
                                "--group", GROUP, "--port", str(port), "--seconds", "2.5", "--timeout", "2.0",
                                "--label", "test", "--out-dir", tmp], capture_output=True, text=True, timeout=30)
            stop.set()
            th.join(5.0)
            if r.returncode == 1 and "届かない" in r.stdout:
                self.skipTest("lo のマルチキャストが届かない環境（WSL のミラーモードなど）")
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            wavs = list(Path(tmp).glob("*_test.wav"))
            self.assertEqual(len(wavs), 1)
            meta = json.loads(wavs[0].with_suffix(".json").read_text())
            self.assertGreater(meta["packets"], 0)
            self.assertAlmostEqual(meta["seconds"], 2.5, delta=0.05)
            self.assertFalse(meta["all_zero"])
            self.assertEqual(len(meta["tones"]), 1, meta["tones"])
            self.assertAlmostEqual(meta["tones"][0]["peak_hz"], 2800.0, delta=20.0)
            # 解析のスクリプトも同じ音を見つけ、図を書く
            a = subprocess.run([sys.executable, str(FEATURE_DIR / "real" / "chime_analyze.py"), str(wavs[0])],
                               capture_output=True, text=True, timeout=60)
            self.assertEqual(a.returncode, 0, a.stderr)
            hz = [float(m) for m in re.findall(r"秒、(\d+) Hz、ふだんより", a.stdout)]
            self.assertEqual(len(hz), 1, a.stdout)
            self.assertAlmostEqual(hz[0], 2800.0, delta=20.0)  # 周波数の刻みは 15.6 Hz（1024 点）
            self.assertTrue(wavs[0].with_name(wavs[0].stem + "_spec.png").exists())


if __name__ == "__main__":
    unittest.main()
