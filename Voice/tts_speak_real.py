#!/usr/bin/env python
"""実機G1に内蔵TTSで指定した文言を喋らせる、最小のワンショットスクリプト。

`AudioClient.TtsMaker(text, speaker_id)`はDDS経由でG1内蔵のTTSエンジンを呼び出す。
追加インストール・外部API・PC2側の中継サーバは一切不要（`unitree_sdk2py`のみ）。
対話パイプライン（段階3、`Voice/dialogue/`）のような複雑な構成が要らない、
最も手軽な発話手段。

話者IDについて（実機の記録）:
  - 話者0は`code=0`で受理されても無音だった（Console班、2026-10-01）
  - 話者1は鳴った（同上。英語の文で確認。日本語は未確認）
  そのため既定は話者1にしている。

PC2（G1内蔵のUbuntu機、system python3.8）の上で動かすのを推奨する。操作PCから
音声のDDSを叩くと、カメラのDDSと同時に使ったときにエラー3102/3104が出た記録がある
（`G1_HuggingFace/UI/Voice/README.md`）。PC2ではインターフェースは`eth0`。

使い方:
  # 鳴らさずに、音声サービスに届くか（音量が読めるか）だけ確かめる
  python3 tts_speak_real.py --network-interface eth0 --check
  # 喋らせる
  python3 tts_speak_real.py --network-interface eth0 "こんにちは"
  python3 tts_speak_real.py --network-interface eth0 --speaker-id 0 "こんにちは"
"""
import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


def _code_of(ret) -> int:
    """SDKの版によって`TtsMaker`等の戻り値が`code`だけ／`(code, data)`のどちらもあるので、codeを取り出す。"""
    if isinstance(ret, tuple):
        return ret[0]
    return ret


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("text", nargs="?", default=None, help="喋らせる文言（--check のときは不要）")
    parser.add_argument(
        "--network-interface",
        default="eth0",
        help="G1の内部ネットワーク(192.168.123.x)のインターフェース名。PC2ならeth0（`ip -br a`で確認）",
    )
    parser.add_argument("--speaker-id", type=int, default=1, help="話者ID(TtsMakerの第2引数)。話者0は無音の記録あり")
    parser.add_argument(
        "--volume",
        type=int,
        default=None,
        help="再生前に音量を設定する(0-100)。共用の機体なので、指定しない（今の音量のまま）のを推奨",
    )
    parser.add_argument("--check", action="store_true", help="鳴らさずに音量を読むだけ（音声サービスに届くかの確認）")
    args = parser.parse_args()

    if not args.check and not args.text:
        parser.error("喋らせる文言を指定してください（確認だけなら --check）")

    print(f"[tts] DDSを初期化します（インターフェース {args.network_interface}）", flush=True)
    ChannelFactoryInitialize(0, args.network_interface)

    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()

    code, data = audio_client.GetVolume()
    print(f"[tts] GetVolume: code={code} data={data}", flush=True)
    if args.check:
        if code != 0:
            print("[tts] 音声サービスに届きませんでした", flush=True)
            raise SystemExit(1)
        print("[tts] 音声サービスに届きました（鳴らしていません）", flush=True)
        return

    if args.volume is not None:
        print(f"[tts] 音量を {args.volume} にします", flush=True)
        audio_client.SetVolume(args.volume)

    print(f"[tts] 喋らせます: {args.text!r}（話者 {args.speaker_id}）", flush=True)
    code = _code_of(audio_client.TtsMaker(args.text, args.speaker_id))
    print(f"[tts] TtsMaker: code={code}", flush=True)
    if code != 0:
        print("[tts] TtsMaker が失敗しました", flush=True)
        raise SystemExit(1)

    # TtsMaker()は再生開始を指示するだけで、完了を待たない。また code=0 は「受理された」
    # という意味で、実際に鳴った証拠ではない（話者0は code=0 でも無音だった）。
    # 文言の長さに応じた待ち時間の目安を入れる（厳密な同期ではない）。
    time.sleep(max(1.5, len(args.text) * 0.3))
    print("[tts] 終わりました（鳴ったかは耳で確かめてください）", flush=True)


if __name__ == "__main__":
    main()
