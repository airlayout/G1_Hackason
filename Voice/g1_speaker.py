#!/usr/bin/env python3
"""G1 本体のスピーカーで WAV を鳴らす（PC2 で動かす）。

元は `Dev/SecurityUI` の `G1_HuggingFace/UI/Speaker/g1_speaker.py`（8923194、2026-11-03 に
実機の有線・無線で鳴ることを確認済み）。`Voice/voicevox_speak.py` から使うために写し、
`APP_NAME` だけ変えた。直すときは元のほうにも反映すること。

    # 単発で鳴らす（音が出るかの確認）
    python3 g1_speaker.py --wav ~/g1_ui_speaker/voice.wav

    # 常駐して、標準入力に 1 行来るたびに鳴らす（UI サーバが ssh 越しに使う）
    python3 g1_speaker.py --wav ~/g1_ui_speaker/voice.wav --serve

    # 鳴らさずに、音声サービスに届くかと音量だけ見る
    python3 g1_speaker.py --check

⚠️ **PC2 の system python3（3.8）で動かす。** `unitree_sdk2py` が入っているのはそちらで、
pixi の 3.11 には入っていない（Navigation/nav2_stable/tools/README.md）。
そのため**標準ライブラリと unitree_sdk2py だけ**で書いてある（numpy も使わない）。

⚠️ **PC 側から音声の DDS を叩かない。** エンタメ班が 2026-09-10 に、PC でカメラと音声の
DDS を同時に使うとエラー 3102/3104 になるのを踏み、音声だけ ssh で G1 の中から鳴らす形に
した（Entame/.../docs/G1_FOUND_AUDIO.md）。無線では DDS 自体を Wi-Fi に通さない方針でもある。

⚠️ **音量は変えない。** 共用の機体なので、いまの音量のまま鳴らす（`--check` で見るだけ）。

⚠️ **純正の TtsMaker は使わない。** 公式資料では中国語と英語のみで、日本語は未確認。
録音した WAV を `PlayStream` で流す（エンタメ班が実機で「聞こえます」を確認した方式）。

WAV は PCM ならサンプリング周波数・チャンネル数は問わない。ここで 16kHz・モノラル・
16bit に直してから送る（`PlayStream` が受けるのはこの形式だけ）。
"""
from __future__ import annotations

import argparse
import array
import json
import signal
import sys
import time
import wave
from pathlib import Path
from typing import List, Optional

TARGET_RATE = 16000
APP_NAME = "g1_voice"
# 0.5 秒分ずつ送り、送るたびに 0.5 秒待つ。エンタメ班が実機で確かめた値
#（一度に送ると数秒の音の途中が切れた）
CHUNK_BYTES = 16000
CHUNK_DELAY_S = 0.5


def to_pcm16_mono_16k(path: Path) -> bytes:
    """PCM WAV を 16kHz・モノラル・16bit（リトルエンディアン）のバイト列にする。"""
    with wave.open(str(path), "rb") as src:
        if src.getcomptype() != "NONE":
            raise ValueError(f"圧縮された WAV は扱えません: {path}")
        channels = src.getnchannels()
        rate = src.getframerate()
        width = src.getsampwidth()
        raw = src.readframes(src.getnframes())
    samples = _decode(raw, width)
    if channels > 1:
        samples = [sum(samples[i:i + channels]) / channels
                   for i in range(0, len(samples) - channels + 1, channels)]
    if rate != TARGET_RATE and samples:
        samples = _resample_linear(samples, rate, TARGET_RATE)
    out = array.array("h", (int(round(max(-1.0, min(1.0, s)) * 32767)) for s in samples))
    if sys.byteorder != "little":
        out.byteswap()
    return out.tobytes()


def _decode(raw: bytes, width: int) -> List[float]:
    """PCM を -1.0〜1.0 の浮動小数に直す（8/16/24/32bit）。"""
    if width == 1:      # 8bit は符号なし
        return [(b - 128) / 128.0 for b in raw]
    if width == 2:
        a = array.array("h")
        a.frombytes(raw[:len(raw) - len(raw) % 2])
        if sys.byteorder != "little":
            a.byteswap()
        return [v / 32768.0 for v in a]
    if width == 3:
        return [int.from_bytes(raw[i:i + 3], "little", signed=True) / 8388608.0
                for i in range(0, len(raw) - 2, 3)]
    if width == 4:
        a = array.array("i")
        a.frombytes(raw[:len(raw) - len(raw) % 4])
        if sys.byteorder != "little":
            a.byteswap()
        return [v / 2147483648.0 for v in a]
    raise ValueError(f"対応していないビット深度です: {width * 8}bit")


def _resample_linear(samples: List[float], src_rate: int, dst_rate: int) -> List[float]:
    """線形補間で周波数を変える。声かけの数秒には十分（音楽用ではない）。"""
    n_out = max(1, int(round(len(samples) * dst_rate / src_rate)))
    step = src_rate / dst_rate
    last = len(samples) - 1
    out = []
    for i in range(n_out):
        pos = i * step
        j = int(pos)
        if j >= last:
            out.append(samples[last])
            continue
        frac = pos - j
        out.append(samples[j] * (1.0 - frac) + samples[j + 1] * frac)
    return out


class Speaker:
    """unitree_sdk2py の AudioClient を1つ持ち回る。"""

    def __init__(self, iface: str) -> None:
        # ⚠️ import はここで行う。変換だけのテストを unitree_sdk2py の無い PC で回すため
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

        ChannelFactoryInitialize(0, iface)
        self._client = AudioClient()
        self._client.SetTimeout(3.0)
        self._client.Init()

    def wait_ready(self, timeout_s: float = 10.0) -> int:
        """音声サービスが応答するまで待ち、いまの音量を返す。

        作った直後の DDS は相手をまだ見つけていないことがあるので、**読むだけの**
        GetVolume を繰り返す（再生の要求は繰り返さない）。
        """
        deadline = time.monotonic() + timeout_s
        while True:
            code, data = self._client.GetVolume()
            if code == 0:
                return _volume_of(data)
            if time.monotonic() >= deadline:
                raise RuntimeError(f"音声サービスが応答しません（GetVolume code={code}）")
            time.sleep(0.1)

    def play(self, pcm: bytes) -> None:
        stream_id = str(time.time_ns() // 1_000_000)
        try:
            for offset in range(0, len(pcm), CHUNK_BYTES):
                code = _code_of(self._client.PlayStream(
                    APP_NAME, stream_id, pcm[offset:offset + CHUNK_BYTES]))
                if code != 0:
                    raise RuntimeError(f"PlayStream が失敗しました（code={code}）")
                time.sleep(CHUNK_DELAY_S)
        finally:
            self._client.PlayStop(APP_NAME)


def _code_of(result: object) -> int:
    return int(result[0] if isinstance(result, tuple) else result)


def _volume_of(data: object) -> int:
    """GetVolume の中身を音量にする。PC2 の版は dict（`{'volume': 100}`、2026-11-03 実測）。
    念のため数値と JSON 文字列も受ける。"""
    if isinstance(data, dict):
        return int(data.get("volume", -1))
    if isinstance(data, (int, float)):
        return int(data)
    try:
        obj = json.loads(str(data))
        return int(obj.get("volume", -1)) if isinstance(obj, dict) else int(obj)
    except (ValueError, TypeError):
        return -1


def _say(obj: dict) -> None:
    """UI サーバへの返事。標準出力は 1 行 1 JSON だけにする（ログは標準エラーへ）。"""
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def serve(speaker: Speaker, pcm: bytes) -> None:
    """標準入力に 1 行来るたびに鳴らす。標準入力が閉じたら終わる。

    ⚠️ **閉じたら必ず終わる**ことが大事。UI サーバ（ssh）が落ちると標準入力が閉じるので、
    PC2 に取り残されない。
    """
    for line in sys.stdin:
        if line.strip() != "play":
            _say({"status": "error", "message": f"知らない指示です: {line.strip()}"})
            continue
        started = time.monotonic()
        try:
            speaker.play(pcm)
        except Exception as exc:   # 1 回失敗しても常駐は続ける
            _say({"status": "error", "message": str(exc)})
            continue
        _say({"status": "played", "seconds": round(time.monotonic() - started, 2)})


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="G1 本体のスピーカーで WAV を鳴らす")
    parser.add_argument("--wav", type=Path, help="鳴らす WAV（PCM）")
    parser.add_argument("--serve", action="store_true", help="常駐し、標準入力の play で鳴らす")
    parser.add_argument("--check", action="store_true", help="鳴らさずに音声サービスと音量だけ見る")
    parser.add_argument("--iface", default="eth0", help="DDS のインターフェース（PC2 は eth0）")
    args = parser.parse_args(argv)
    if not args.check and args.wav is None:
        parser.error("--wav か --check が要ります")

    # systemd/ssh からの SIGTERM・SIGHUP でも PlayStop を通って終わるようにする
    def _stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGHUP, _stop)

    pcm = b""
    if args.wav is not None:
        pcm = to_pcm16_mono_16k(args.wav)
        print(f"[声] {args.wav.name}: {len(pcm) / 2 / TARGET_RATE:.2f} 秒", file=sys.stderr)
    speaker = Speaker(args.iface)
    volume = speaker.wait_ready()
    print(f"[声] 音声サービスに届いた。いまの音量: {volume}", file=sys.stderr)
    if args.check:
        _say({"status": "ready", "volume": volume})
        return 0
    if args.serve:
        _say({"status": "ready", "volume": volume,
              "seconds": round(len(pcm) / 2 / TARGET_RATE, 2)})
        serve(speaker, pcm)
        return 0
    speaker.play(pcm)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as exc:
        # ⚠️ 握りつぶさない。理由を 1 行の JSON で返してから落ちる
        _say({"status": "error", "message": str(exc)})
        print(f"[声] 失敗: {exc}", file=sys.stderr)
        sys.exit(2)
