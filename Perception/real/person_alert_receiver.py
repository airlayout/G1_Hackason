#!/usr/bin/env python3
"""G1 の person_alert.py が配信する「人を検出した」出来事を受け取り、写真を保存して表示する(PC で動かす)。

    G1_HuggingFace/venv/bin/python Perception/real/person_alert_receiver.py --host 192.168.0.82

PC から G1 へつなぐ(ZMQ の SUB)。G1 の生きている合図が --stale 秒届かなければ警告する(G1 が止まった・つながらない)。
受け取りを止めている間の出来事は届かない(G1 の _local/perception/person_alert/ にだけ残る)。
写真は --out-dir(既定 _local/perception/person_alert_received/)に、G1 と同じファイル名で保存する。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="192.168.0.82", help="G1(PC2)のアドレス。有線なら 192.168.123.164")
    p.add_argument("--port", type=int, default=5560)
    p.add_argument("--out-dir", type=Path, default=REPO_ROOT / "_local" / "perception" / "person_alert_received")
    p.add_argument("--stale", type=float, default=15.0, help="G1 の合図がこの秒数届かなければ警告する")
    p.add_argument("--max-seconds", type=float, help="この秒数で終える(動作確認用。既定は Ctrl-C まで)")
    return p.parse_args()


def main() -> int:
    import zmq

    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVTIMEO, 1000)  # Ctrl-C と合図の途切れを見るため、1 秒ごとに戻る
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{args.host}:{args.port}")
    print(f"[receiver] tcp://{args.host}:{args.port} を待つ。保存先 {args.out_dir}。Ctrl-C で終了")

    t_start = time.time()
    last_beat: float | None = None
    stale_warned = False
    received = 0
    try:
        while args.max_seconds is None or time.time() - t_start < args.max_seconds:
            try:
                parts = sock.recv_multipart()
            except zmq.Again:
                waited = time.time() - (last_beat or t_start)
                if waited > args.stale and not stale_warned:
                    print(f"[receiver] ⚠️ G1 から {waited:.0f} 秒何も届かない（person_alert.py が動いているか・アドレスを確かめる）")
                    stale_warned = True
                continue
            kind, info = parts[0], json.loads(parts[1])
            if kind == b"heartbeat":
                if last_beat is None or stale_warned:
                    print(f"[receiver] G1 とつながった（{info['detector']}、{info['fps']} fps、出来事 {info['events']} 件）")
                last_beat, stale_warned = time.time(), False
            elif kind == b"event" and len(parts) >= 3:
                path = args.out_dir / info["photo"]
                path.write_bytes(parts[2])
                received += 1
                confs = " ".join(f"{d['confidence']:.2f}" for d in info["detections"][:5])
                print(f"[receiver] 🧍 {info['time_text']}（G1 の時計）人 {info['count']} 人（{confs}）→ {path}")
    except KeyboardInterrupt:
        pass
    sock.close()
    print(f"[receiver] 終了: 受け取った写真 {received} 枚")
    return 0


if __name__ == "__main__":
    sys.exit(main())
