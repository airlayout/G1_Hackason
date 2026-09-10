#!/usr/bin/env python
"""実機G1に内蔵TTSで指定した文言を喋らせる、最小のワンショットスクリプト。

`AudioClient.TtsMaker(text, speaker_id)`はDDS経由でG1内蔵のTTSエンジンを呼び出す。
追加インストール・外部API・PC2側の中継サーバは一切不要（`unitree_sdk2py`のみ）。
対話パイプライン（段階3、`Voice/real/dialogue/`）のような複雑な構成が要らない、
最も手軽な発話手段。

前提:
  - G1本体の電源が入っていること。
  - 操作側PCがG1と同一サブネット(192.168.123.x, x != 164)のstatic IPを持ち、
    Ethernetで直結されていること（`Common/network/setup_ethernet_for_g1.sh`参照）。

使い方:
  python Voice/real/tts_speak_real.py --network-interface enp3s0 "こんにちは"
  python Voice/real/tts_speak_real.py --network-interface enp3s0 --speaker-id 0 --volume 50 "見つけたよ"
"""
import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("text", help="喋らせる文言")
    parser.add_argument(
        "--network-interface",
        default="enp3s0",
        help="G1と直結しているネットワークインターフェース名(`ip -br a`で確認)",
    )
    parser.add_argument("--speaker-id", type=int, default=0, help="話者ID(TtsMakerの第2引数)")
    parser.add_argument(
        "--volume",
        type=int,
        default=None,
        help="再生前に音量を設定する(0-100)。指定しない場合は現在の音量のまま",
    )
    args = parser.parse_args()

    print(f"Initializing DDS on interface {args.network_interface}...", flush=True)
    ChannelFactoryInitialize(0, args.network_interface)

    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()

    if args.volume is not None:
        print(f"Setting volume to {args.volume}...", flush=True)
        audio_client.SetVolume(args.volume)

    print(f"Speaking: {args.text!r}", flush=True)
    code, _ = audio_client.TtsMaker(args.text, args.speaker_id)
    if code != 0:
        print(f"TtsMaker failed (code={code}).", flush=True)
        raise SystemExit(1)

    # TtsMaker()は再生開始を指示するだけで、完了を待たない。文言の長さに応じた
    # 待ち時間の目安を入れる(厳密な同期ではない)。
    time.sleep(max(1.5, len(args.text) * 0.3))
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
