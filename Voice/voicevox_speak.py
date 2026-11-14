#!/usr/bin/env python3
"""VOICEVOX で作った日本語の音声を、G1 本体のスピーカーで鳴らす（操作PCで動かす）。

G1 内蔵の TTS（`TtsMaker`）は日本語を読めない（2026-11-14 実機、Voice/VOICEVOX.md）。
そこで操作PCの VOICEVOX CORE（`voicevox_core`）で WAV を作り、PC2 に送って `PlayStream` で流す。

    操作PC: 文 ──voicevox_core──> WAV（24kHz・モノラル。PC2 で 16kHz に直して送る）
            ──scp──> PC2: /tmp/g1_voice_tts/ ──ssh──> g1_speaker.py --wav（PlayStream）

- PC から音声の DDS は叩かない。PC2 の中で鳴らす（g1_speaker.py の注意書きを参照）
- 音量は変えない（共用の機体）
- PC2 では /tmp の下だけを使う（共用の機体の運用ルール）

前提: `voicevox_core` の Python パッケージと、ONNX Runtime・辞書・音声モデルが
`--core-dir`（既定 ~/voicevox_core/voicevox_core）にあること。入れ方は Voice/VOICEVOX.md の
「入れる」。`voicevox_core` を入れた venv の python で動かす。

使い方:
    PY=~/voicevox_core/venv/bin/python
    # WAV を作るだけ（PC2 に送らない）
    $PY Voice/voicevox_speak.py --no-send "こんにちは、私はジーワンです"
    # 作って PC2 に送る（鳴らさない）。最後に鳴らすコマンドを出す
    $PY Voice/voicevox_speak.py "こんにちは、私はジーワンです"
    # 作って送って鳴らす
    $PY Voice/voicevox_speak.py --play "こんにちは、私はジーワンです"
    # 話者（スタイル ID）の一覧
    $PY Voice/voicevox_speak.py --list-speakers

⚠️ VOICEVOX は話者（キャラクター）ごとに利用規約がある。外に出す場面（発表・動画など）では
「VOICEVOX:剣崎雌雄」のようなクレジット表記が要る。話者ごとの規約を確かめること。
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import List, Optional

DEFAULT_CORE_DIR = Path.home() / "voicevox_core" / "voicevox_core"
# 読み込む音声モデル（VVM）のファイル名パターン。既定は置いてあるもの全部。
# 0.vvm = 四国めたん・ずんだもん・春日部つむぎ・雨晴はう、4.vvm = 玄野武宏・剣崎雌雄
DEFAULT_MODEL = "*.vvm"
# 既定の声: 剣崎雌雄（ノーマル）を少し低く・抑揚控えめに（2026-11-14 に実機で聞き比べて決めた）
DEFAULT_STYLE = 21
DEFAULT_PITCH = -0.06
DEFAULT_INTONATION = 0.8
DEFAULT_HOST = "unitree@192.168.0.82"   # PC2 の Wi-Fi 側。有線なら --host unitree@192.168.123.164
REMOTE_DIR = "/tmp/g1_voice_tts"
SPEAKER_PY = Path(__file__).resolve().parent / "g1_speaker.py"
OUT_DIR = Path(__file__).resolve().parent / "out"   # 作った WAV の置き場（git 管理外）


def _find_one(base: Path, pattern: str, what: str) -> Path:
    """base の下から pattern に合うものを 1 つ探す。無ければ理由を出して終わる。"""
    hits = sorted(base.glob(pattern))
    if not hits:
        print(f"[voicevox] {what}が見つかりません: {base}/{pattern}", file=sys.stderr)
        print("[voicevox] 入れ方は Voice/VOICEVOX.md の「入れる」", file=sys.stderr)
        raise SystemExit(1)
    return hits[0]


def make_synthesizer(core_dir: Path, model: str):
    """ONNX Runtime・辞書・音声モデルを読み込んだ Synthesizer を返す。"""
    # ⚠️ import はここで行う。--help を voicevox_core の無い python でも出せるように
    from voicevox_core.blocking import Onnxruntime, OpenJtalk, Synthesizer, VoiceModelFile

    ort_lib = _find_one(core_dir / "onnxruntime" / "lib", "libvoicevox_onnxruntime.so*", "ONNX Runtime ")
    dict_dir = _find_one(core_dir / "dict", "open_jtalk_dic_utf_8-*", "Open JTalk の辞書")
    _find_one(core_dir / "models", f"**/{model}", f"音声モデル {model} ")

    synth = Synthesizer(Onnxruntime.load_once(filename=str(ort_lib)), OpenJtalk(str(dict_dir)))
    for vvm in sorted((core_dir / "models").glob(f"**/{model}")):
        with VoiceModelFile.open(str(vvm)) as f:
            synth.load_voice_model(f)
    return synth


def list_speakers(synth) -> None:
    """読み込んだモデルに入っている話者と、スタイル ID の一覧を出す。"""
    for meta in synth.metas():
        styles = ", ".join(f"{st.name}={st.id}" for st in meta.styles)
        print(f"{meta.name}: {styles}")


def synthesize(synth, text: str, style: int, speed: float, pitch: float = 0.0, intonation: float = 1.0) -> bytes:
    """文を 24kHz・モノラル・16bit の WAV にする。

    ⚠️ voicevox_core は 24000 の倍数しか出せない（16000 を入れると InvalidQueryError）。
    PlayStream が受ける 16kHz への変換は、PC2 の g1_speaker.py が送る前に行う。
    """
    query = synth.create_audio_query(text, style)
    query.output_stereo = False
    query.speed_scale = speed
    query.pitch_scale = pitch            # 0.0 が標準。負で低く、正で高く（-0.15〜0.15 くらいが目安）
    query.intonation_scale = intonation  # 1.0 が標準。小さいと平板（落ち着いた感じ）、大きいと抑揚が強い
    return synth.synthesis(query, style)


def _run(cmd: List[str], timeout: float) -> None:
    print(f"[voicevox] $ {' '.join(cmd)}", flush=True)
    subprocess.run(cmd, check=True, timeout=timeout)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("text", nargs="?", default=None, help="喋らせる文")
    parser.add_argument("--style", type=int, default=DEFAULT_STYLE, help="VOICEVOX のスタイル ID（--list-speakers）")
    parser.add_argument("--speed", type=float, default=1.0, help="話す速さ（1.0 が標準）")
    parser.add_argument("--pitch", type=float, default=DEFAULT_PITCH, help="声の高さ（0.0 が VOICEVOX の標準、負で低く）")
    parser.add_argument("--intonation", type=float, default=DEFAULT_INTONATION, help="抑揚（1.0 が VOICEVOX の標準、小さいと平板）")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="読み込む音声モデル（VVM のファイル名パターン）")
    parser.add_argument("--core-dir", type=Path, default=DEFAULT_CORE_DIR, help="ダウンロードツールの出力先")
    parser.add_argument("--host", default=DEFAULT_HOST, help="PC2 の ssh 宛先")
    parser.add_argument("--play", action="store_true", help="PC2 に送ったあと、そのまま鳴らす")
    parser.add_argument("--no-send", action="store_true", help="WAV を作るだけで PC2 に送らない")
    parser.add_argument("--list-speakers", action="store_true", help="話者の一覧を出して終わる")
    args = parser.parse_args(argv)

    synth = make_synthesizer(args.core_dir, args.model)
    if args.list_speakers:
        list_speakers(synth)
        return 0
    if not args.text or not args.text.strip():
        parser.error("喋らせる文を指定してください")

    wav = synthesize(synth, args.text, args.style, args.speed, args.pitch, args.intonation)
    OUT_DIR.mkdir(exist_ok=True)
    local = OUT_DIR / "speech.wav"
    local.write_bytes(wav)
    seconds = (len(wav) - 44) / 2 / 24000
    print(f"[voicevox] {local} を作りました（約 {seconds:.1f} 秒、スタイル {args.style}）", flush=True)
    if args.no_send:
        return 0

    _run(["ssh", "-o", "ConnectTimeout=8", args.host, f"mkdir -p {REMOTE_DIR}"], timeout=30)
    _run(["scp", "-o", "ConnectTimeout=8", str(SPEAKER_PY), str(local), f"{args.host}:{REMOTE_DIR}/"], timeout=60)
    play_cmd = f"python3 {REMOTE_DIR}/g1_speaker.py --wav {REMOTE_DIR}/speech.wav"
    if not args.play:
        print("[voicevox] PC2 に送りました。鳴らすには:", flush=True)
        print(f"    ssh {args.host} '{play_cmd}'", flush=True)
        return 0
    _run(["ssh", "-o", "ConnectTimeout=8", args.host, play_cmd], timeout=seconds + 60)
    print("[voicevox] 鳴らし終わりました（聞こえたかは耳で確かめてください）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
