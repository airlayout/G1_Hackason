"""G1 のマイクの音を録って WAV に保存する（エレベーターの到着の音を確かめるため）。何も送らない（受けるだけ）。

    # 有線でつないだノート PC（WSL のミラーモード）。口の名前から IP を調べる。30 秒（Ctrl+C でそこまで保存）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/mic_record.py --network-interface eth5 --seconds 30
    # IP を直接（ラボ PC など）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/mic_record.py --local-ip 192.168.123.99 --label arrive_front

マイクは UDP マルチキャスト 239.168.123.161:5555（16 kHz・モノラル・16 bit）。common/chime.py の説明を参照。
録っている間は 0.5 秒ごとに、音の大きさ（dBFS。フルスケールが 0）と、一番強い周波数を出す。
終わったら _local/button_press/sound/<日時>[_<label>].wav と .json（録り始めの時刻・パケットの数・途切れ・見つけた高い音）を残す。
解析は real/chime_analyze.py（スペクトログラムの図と、高い音の時刻・高さ）。

うまくいかないとき:
- 「パケットが届かない」: 有線の口・IP が違う、マイクが起きていない、WSL のミラーモードで届かない（なら PC2 で録る）
- 「音が 0 のまま」: アプリかリモコンでマイクを起こす（wake-up）。内蔵の音声アシスタントを使っているとこうなることがある
- マイクを使う他のチーム（音声の対話）と同時に使うと競合した記録がある（Voice/FAILURES.md）
- ノイズキャンセリングがかかっているらしい（未確認）。チャイムが弱められていないかを、録った音で確かめる
"""

from __future__ import annotations

import argparse
import fcntl
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.chime import (  # noqa: E402
    MIC_GROUP, MIC_PORT, SAMPLE_RATE, Chunk, MicTimeout, detect_tones, level_dbfs, open_mic_socket, peak_hz, receive,
    save_wav, to_samples, write_meta,
)
from common.config import REPO_ROOT  # noqa: E402

GAP_S = 0.1  # パケットの間がこれより空いたら途切れとして数える


def iface_ip(name: str) -> str:
    """ネットワークの口の名前 → IPv4 アドレス（Linux の SIOCGIFADDR）。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packed = fcntl.ioctl(s.fileno(), 0x8915, struct.pack("256s", name[:15].encode()))
    except OSError as e:
        raise SystemExit(f"[mic] ❌ 口 {name} の IP が分からない（{e}）。--local-ip で直接渡す") from e
    finally:
        s.close()
    return socket.inet_ntoa(packed[20:24])


def summarize(chunks: list[Chunk]) -> dict[str, float | int]:
    """パケットの数・量・途切れ（間が GAP_S を超えた回数と最大）。"""
    ts = np.array([c.t for c in chunks])
    gaps = np.diff(ts) if len(ts) > 1 else np.zeros(0)
    return {"packets": len(chunks), "bytes": sum(len(c.data) for c in chunks),
            "mean_packet_bytes": float(np.mean([len(c.data) for c in chunks])) if chunks else 0.0,
            "gaps_over_0_1s": int(np.sum(gaps > GAP_S)), "max_gap_s": float(gaps.max()) if len(gaps) else 0.0}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group()
    src.add_argument("--network-interface", help="G1 につないでいる有線の口（例 eth5）。IP を調べて使う")
    src.add_argument("--local-ip", help="有線の IP（192.168.123.x。既定 192.168.123.99）")
    p.add_argument("--seconds", type=float, default=30.0, help="録る長さ [秒]（Ctrl+C でそこまで保存）")
    p.add_argument("--label", default="", help="ファイル名に付ける言葉（例 arrive_front・arrive_back・press）")
    p.add_argument("--group", default=MIC_GROUP)
    p.add_argument("--port", type=int, default=MIC_PORT)
    p.add_argument("--timeout", type=float, default=3.0, help="最初のパケットを待つ秒数")
    p.add_argument("--out-dir", type=Path, default=REPO_ROOT / "_local" / "button_press" / "sound")
    args = p.parse_args()

    local_ip = iface_ip(args.network_interface) if args.network_interface else (args.local_ip or "192.168.123.99")
    print(f"[mic] {args.group}:{args.port} を {local_ip} で受ける（{args.seconds:.0f} 秒、Ctrl+C で止めて保存）")
    sock = open_mic_socket(local_ip, args.group, args.port, args.timeout)
    started = time.time()
    shown = [started]

    def show(chunks: list[Chunk]) -> None:
        if chunks[-1].t - shown[0] < 0.5:
            return
        shown[0] = chunks[-1].t
        recent = to_samples(chunks[-max(1, len(chunks) // 20):])[-SAMPLE_RATE // 2:]
        print(f"[mic] {chunks[-1].t - started:5.1f} s  大きさ {level_dbfs(recent):6.1f} dBFS  "
              f"一番強い周波数 {peak_hz(recent):6.0f} Hz")

    try:
        chunks = receive(sock, args.seconds, show)
    except MicTimeout as e:
        print(f"[mic] ❌ {e}")
        return 1
    finally:
        sock.close()

    x = to_samples(chunks)
    info = summarize(chunks)
    print(f"[mic] 受けた: {info['packets']} パケット、{len(x) / SAMPLE_RATE:.1f} 秒分（1 パケット平均 {info['mean_packet_bytes']:.0f} バイト）、"
          f"0.1 秒より長い途切れ {info['gaps_over_0_1s']} 回（最大 {info['max_gap_s']:.2f} 秒）")
    if len(x) and not np.any(x):
        print("[mic] ⚠️ 音が 0 のまま。アプリかリモコンでマイクを起こす（wake-up）。内蔵の音声アシスタントを使っているとこうなることがある")
    tones = detect_tones(x)
    for tone in tones:
        print(f"[mic] 高い音: {tone.start_s:.2f}〜{tone.end_s:.2f} s（{tone.duration_s:.2f} 秒）、{tone.peak_hz:.0f} Hz、"
              f"ふだんより {tone.above_db:.0f} dB")
    if not tones:
        print("[mic] 高い音（1〜6 kHz で、ふだんより 15 dB 以上・0.1 秒以上）は見つからなかった")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    stem = time.strftime("%Y%m%d_%H%M%S", time.localtime(started)) + (f"_{args.label}" if args.label else "")
    wav = args.out_dir / f"{stem}.wav"
    save_wav(wav, x)
    write_meta(wav.with_suffix(".json"), {
        "started_at": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(started)), "started_epoch": started,
        "first_packet_epoch": chunks[0].t, "local_ip": local_ip, "group": args.group, "port": args.port,
        "sample_rate": SAMPLE_RATE, "seconds": len(x) / SAMPLE_RATE, "level_dbfs": level_dbfs(x),
        "all_zero": bool(len(x) and not np.any(x)), "label": args.label, **info}, tones)
    print(f"[mic] 保存: {wav}（と .json）。図は real/chime_analyze.py {wav}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
