"""人を検知したときの声かけ（2026-11-03）のテスト。

守りたいのは3つ。
1. どんな PCM WAV でも、`PlayStream` が受ける 16kHz・モノラル・16bit に**正しい長さで**直ること
   （周波数を取り違えると、早回し・遅回しの声になる）
2. **人が映っている間は繰り返し、消えたら止まる**こと。再生中に重ねて頼まないこと
3. **カメラが途切れても止まる**こと（映像が途絶えると「消失」が来ないので、鳴りっぱなしになる）
4. PC2 の再生プロセスが落ちたら「失敗」と見えること（黙って鳴らないのがいちばん困る）

⚠️ unitree_sdk2py も ssh も使わない。PC2 の再生プロセスは手元の偽物で真似る。
"""
from __future__ import annotations

import io
import math
import struct
import sys
import tempfile
import textwrap
import time
import unittest
import wave
from dataclasses import dataclass
from pathlib import Path

UI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(UI / "Main"))
sys.path.insert(0, str(UI / "Speaker"))

import g1_speaker                                   # noqa: E402
from voice import VoiceAnnouncer, build_voice       # noqa: E402


@dataclass
class _Event:
    """vision.DetectionEvent の代わり（vision は ultralytics を引き込むので使わない）。"""

    kind: str
    at: float


def _write_wav(path: Path, rate: int, channels: int, width: int, seconds: float) -> None:
    """440Hz の正弦波（振幅 0.5）を書く。"""
    n = int(rate * seconds)
    frames = bytearray()
    for i in range(n):
        v = 0.5 * math.sin(2 * math.pi * 440 * i / rate)
        if width == 1:
            sample = bytes([int(round(v * 127)) + 128])
        elif width == 2:
            sample = struct.pack("<h", int(round(v * 32767)))
        elif width == 3:
            sample = int(round(v * 8388607)).to_bytes(3, "little", signed=True)
        else:
            sample = struct.pack("<i", int(round(v * 2147483647)))
        frames += sample * channels
    with wave.open(str(path), "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(width)
        w.setframerate(rate)
        w.writeframes(bytes(frames))


class ConvertTest(unittest.TestCase):
    def test_any_pcm_becomes_16k_mono_16bit(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            for rate, ch, width in [(44100, 1, 2), (48000, 2, 2), (16000, 1, 1),
                                    (22050, 1, 3), (44100, 2, 4)]:
                with self.subTest(rate=rate, channels=ch, bits=width * 8):
                    path = Path(d) / "x.wav"
                    _write_wav(path, rate, ch, width, 1.0)
                    pcm = g1_speaker.to_pcm16_mono_16k(path)
                    # 1 秒 = 16000 サンプル × 2 バイト。長さが合えば周波数は取り違えていない
                    self.assertAlmostEqual(len(pcm) / 2, 16000, delta=2)
                    peak = max(abs(v) for v in struct.unpack(f"<{len(pcm) // 2}h", pcm))
                    # 振幅 0.5 が保たれている（ステレオの平均で潰れたり、桁を間違えたりしない）
                    self.assertAlmostEqual(peak / 32767, 0.5, delta=0.03)

    def test_compressed_wav_is_refused(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "x.wav"
            _write_wav(path, 16000, 1, 2, 0.1)
            raw = bytearray(path.read_bytes())
            raw[20:22] = struct.pack("<H", 0x11)   # IMA ADPCM と偽る
            path.write_bytes(bytes(raw))
            with self.assertRaises(Exception):
                g1_speaker.to_pcm16_mono_16k(path)


class ServeTest(unittest.TestCase):
    """PC2 側の常駐ループ。標準入力の1行ごとに1回だけ鳴らし、閉じたら終わる。"""

    def test_plays_once_per_line_and_keeps_going_after_error(self) -> None:
        played: list[bytes] = []

        class _FakeSpeaker:
            def __init__(self) -> None:
                self.calls = 0

            def play(self, pcm: bytes) -> None:
                self.calls += 1
                if self.calls == 2:
                    raise RuntimeError("PlayStream が失敗しました（code=3104）")
                played.append(pcm)

        out = io.StringIO()
        old_in, old_out = sys.stdin, sys.stdout
        sys.stdin, sys.stdout = io.StringIO("play\nplay\nhello\nplay\n"), out
        try:
            g1_speaker.serve(_FakeSpeaker(), b"\x00\x00")   # type: ignore[arg-type]
        finally:
            sys.stdin, sys.stdout = old_in, old_out
        statuses = [line.split('"status": "')[1].split('"')[0] for line in out.getvalue().splitlines()]
        self.assertEqual(statuses, ["played", "error", "error", "played"])
        self.assertEqual(len(played), 2)


# PC2 の `g1_speaker.py --serve` の真似。play が来たら少し待って played を返す
_FAKE_HELPER = textwrap.dedent("""
    import json, sys, time
    print(json.dumps({"status": "ready", "volume": 85}), flush=True)
    for line in sys.stdin:
        time.sleep(float(sys.argv[1]))
        print(json.dumps({"status": "played", "seconds": 0.1}), flush=True)
        if len(sys.argv) > 2:
            sys.exit(3)       # 1 回鳴らしたら落ちる版
""")


class AnnouncerTest(unittest.TestCase):
    def setUp(self) -> None:
        self._dir = tempfile.TemporaryDirectory()
        self.helper = Path(self._dir.name) / "fake_helper.py"
        self.helper.write_text(_FAKE_HELPER, encoding="utf-8")
        self.camera_ok = True

    def tearDown(self) -> None:
        self._dir.cleanup()

    def _start(self, *helper_args: str, gap: float = 0.05) -> VoiceAnnouncer:
        v = VoiceAnnouncer([sys.executable, str(self.helper), *helper_args], "立ち入り禁止です",
                           gap, still_present=lambda: self.camera_ok)
        v.start()
        self.addCleanup(v.close)
        self._wait(lambda: v.status["state"] == "ready")
        return v

    def _wait(self, cond, timeout: float = 5.0) -> None:
        deadline = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > deadline:
                self.fail("時間内に状態が変わらなかった")
            time.sleep(0.02)

    def test_repeats_while_present_and_stops_on_absence(self) -> None:
        v = self._start("0.05")
        v.on_event(_Event("消失", 100.0))
        time.sleep(0.2)
        self.assertEqual(v.status["played"], 0)       # 映っていなければ鳴らさない
        v.on_event(_Event("検知", 100.0))
        self._wait(lambda: v.status["played"] >= 3)    # 映っている間は繰り返す
        v.on_event(_Event("消失", 101.0))
        self._wait(lambda: v.status["state"] == "ready")
        n = v.status["played"]
        time.sleep(0.4)
        self.assertEqual(v.status["played"], n)        # 消えたら止まる

    def test_camera_loss_stops_even_without_absence_event(self) -> None:
        v = self._start("0.05")
        v.on_event(_Event("検知", 100.0))
        self._wait(lambda: v.status["played"] >= 2)
        self.camera_ok = False                          # 「消失」は来ないまま映像が途絶えた
        self._wait(lambda: not v.status["present"])
        self._wait(lambda: v.status["state"] == "ready")
        n = v.status["played"]
        time.sleep(0.4)
        self.assertEqual(v.status["played"], n)

    def test_redetection_while_speaking_does_not_overlap(self) -> None:
        v = self._start("0.5", gap=10.0)
        v.on_event(_Event("検知", 100.0))
        self.assertEqual(v.status["state"], "speaking")
        v.on_event(_Event("消失", 100.1))
        v.on_event(_Event("検知", 100.2))               # 言っている最中にまた映った
        self._wait(lambda: v.status["played"] == 1)
        time.sleep(0.3)
        # 重ねて頼んでいれば、言い終えた直後にもう 1 回鳴っている（間隔は 10 秒にしてある）
        self.assertEqual(v.status["played"], 1)
        self.assertEqual(v.status["state"], "ready")

    def test_helper_dying_shows_error(self) -> None:
        v = self._start("0.05", "die")
        v.on_event(_Event("検知", 100.0))
        self._wait(lambda: v.status["state"] == "error")
        self.assertIn("終了コード 3", str(v.status["message"]))


class BuildTest(unittest.TestCase):
    def test_without_host_logs_only(self) -> None:
        v = build_voice({"phrase": "立ち入り禁止です"}, None)
        assert v is not None
        self.addCleanup(v.close)
        self.assertEqual(v.status["state"], "off")
        v.on_event(_Event("検知", 1.0))          # ログに出すだけ。例外にならない
        self.assertTrue(v.status["present"])

    def test_disabled(self) -> None:
        self.assertIsNone(build_voice({"enabled": False}, "g1"))

    def test_ssh_command_points_at_pc2_files(self) -> None:
        v = build_voice({"wav": "../Voice/tachiiri_kinshi.wav",
                         "remote_dir": "/home/unitree/g1_ui_speaker"}, "g1")
        assert v is not None
        cmd = v._command or []
        self.assertEqual(cmd[0], "ssh")
        self.assertIn("BatchMode=yes", cmd)
        self.assertTrue(cmd[-1].endswith(
            "/home/unitree/g1_ui_speaker/g1_speaker.py --serve "
            "--wav /home/unitree/g1_ui_speaker/tachiiri_kinshi.wav"))


if __name__ == "__main__":
    unittest.main()
