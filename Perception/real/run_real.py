#!/usr/bin/env python3
"""Perception / real エントリスクリプト。

実機G1側で `python src/lerobot/robots/unitree_g1/run_g1_server.py --camera` を
起動した後、そのZMQストリームに接続してYOLO検出を回す。
接続先IPはconfigs/config.yamlのsource.zmq.server_addressで指定する
(G1_HuggingFace/README.mdの実機接続手順を参照)。
"""
import argparse
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from common.pipeline import run_pipeline  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Perception / real: G1 YOLO検出パイプライン")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "configs" / "config.yaml"),
        help="設定ファイル(YAML)のパス",
    )
    parser.add_argument(
        "--source",
        choices=["zmq", "video", "webcam"],
        help="config.yamlのsource.typeを上書きする",
    )
    parser.add_argument(
        "--server-address",
        help="config.yamlのsource.zmq.server_addressを上書きする(実機G1のIPアドレス)",
    )
    parser.add_argument(
        "--video-path",
        help="--source video のときに使う動画ファイルパス(config.yamlのsource.video.pathを上書き)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="処理するフレーム数の上限(動作確認用、指定しなければ無制限)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    with open(args.config, encoding="utf-8") as f:
        config = yaml.safe_load(f)

    if args.source:
        config["source"]["type"] = args.source
    if args.server_address:
        config["source"].setdefault("zmq", {})["server_address"] = args.server_address
    if args.video_path:
        config["source"].setdefault("video", {})["path"] = args.video_path

    run_pipeline(config, max_frames=args.max_frames)


if __name__ == "__main__":
    main()
