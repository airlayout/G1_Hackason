"""Foxglove Bridge に最小の WebSocket クライアントで繋ぎ、実際に流れているか確かめる。

ブラウザを開く前の切り分け用。ブラウザで見えないとき、原因が
「橋が落ちている」「トンネルが切れている」「トピックが出ていない」のどれかを
依存パッケージ無し（標準ライブラリのみ）で判定できる。

    bash tunnel_foxglove.sh
    python3 check_foxglove_stream.py /unitree/slam_mapping/points 5
    python3 check_foxglove_stream.py /utlidar/cloud_livox_mid360 5

トンネルを張らず PC2 を直接叩くこともできる（CLI は混在コンテンツの制限を
受けないので、ブラウザと違って ws:// で直接繋がる）:

    python3 check_foxglove_stream.py --host 192.168.123.164 /utlidar/cloud_livox_mid360 10
    python3 check_foxglove_stream.py --host 192.168.123.164 --list
    python3 check_foxglove_stream.py --host 192.168.123.164 /a,/b,/c 30   # 複数まとめて

2026-09-03 の実測: 生LiDAR 10.13Hz / 4.5MB/s、地図 9.97Hz / 0.4MB/s。
encoding=cdr（バイナリのまま）で届く。
2026-09-13 の実測（PC2 直結・実機稼働中）: 生LiDAR 9.9Hz / 4.39MB/s。
このとき PC2 側の foxglove_bridge は **0.06 コア**しか使わない（RViz2 は 2.38 コア）。
"""
import argparse, sys

from foxglove_ws import FoxgloveClient


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("topics", nargs="?", default="/utlidar/cloud_livox_mid360",
                    help="トピック名。カンマ区切りで複数指定できる")
    ap.add_argument("duration", nargs="?", type=float, default=5.0)
    ap.add_argument("--host", default="127.0.0.1", help="既定はトンネル前提の localhost")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--list", action="store_true", help="チャンネル一覧を出して終わる")
    args = ap.parse_args()

    client = FoxgloveClient(args.host, args.port)
    print("  ハンドシェイク OK (%s:%d)" % (args.host, args.port))
    channels = client.collect_channels()
    if client.server_info:
        print("  serverInfo: name=%s capabilities=%s"
              % (client.server_info.get("name"), client.server_info.get("capabilities")))

    if args.list:
        for topic in sorted(channels):
            print("  %-52s %s" % (topic, channels[topic].get("schemaName", "")))
        print("  合計 %d チャンネル" % len(channels))
        return 0

    wanted = [t.strip() for t in args.topics.split(",") if t.strip()]
    missing = [t for t in wanted if t not in channels]
    for topic in missing:
        print("  **%s が advertise されなかった**" % topic)
    by_id = client.subscribe([t for t in wanted if t in channels])
    if not by_id:
        return 1
    for topic in by_id.values():
        ch = channels[topic]
        print("  advertise: %s  schema=%s  encoding=%s"
              % (ch["topic"], ch["schemaName"], ch["encoding"]))
    print("  subscribe 送信 -> %.1f 秒受信する" % args.duration)

    stats, elapsed = client.receive(args.duration, by_id)
    client.close()
    total = 0
    for topic in sorted(stats):
        count, nbytes = stats[topic]
        total += nbytes
        print("  %-44s %5d 件 / %6.2f MB => %5.2f Hz, %5.2f MB/s"
              % (topic, count, nbytes / 1e6, count / elapsed, nbytes / 1e6 / elapsed))
    if len(stats) > 1:
        print("  %-44s %5s   %6s    %5s  %5.2f MB/s"
              % ("合計", "", "", "", total / 1e6 / elapsed))
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
