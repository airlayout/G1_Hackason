#!/usr/bin/env python
"""[エントリポイント] 実機 G1 の LiDAR 点群を収録して `scans.npz` にする。**受信のみ。**

想定している運用: G1 は**手動で歩かせる**（リモコン等）。操作 PC は Ethernet で
繋いで点群を受け取るだけで、ロボットには一切コマンドを送らない。
そのため歩行制御・`LocoClient`・elastic band といった話は一切出てこない。

出力は `Mapping/sim/record_scans.py` と同じ形式なので、地図化は同じ
`Mapping/run_slam.py` で処理できる（真値の姿勢は実機には存在しないので入れない。
`run_slam.py` はその場合、誤差評価をスキップして地図だけ出す）。

前提:
  1. 操作 PC と G1 が Ethernet で直結され、疎通していること
     （`Common/network/README.md` の手順 1〜2）
  2. LiDAR のトピック名が分かっていること
     （`python Mapping/real/discover_topics.py` で調べる）

使い方:
    python Mapping/real/record_scans_real.py --topic rt/utlidar/cloud --duration 90
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from Mapping.real.pointcloud2 import describe, to_xyz  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="実機 G1 の LiDAR 点群を収録する（受信のみ）")
    parser.add_argument("--topic", required=True, help="LiDAR の DDS トピック名（discover_topics.py で調べる）")
    parser.add_argument(
        "--network-interface",
        default="enp3s0",
        help="G1 と繋がっている Ethernet インターフェース名",
    )
    parser.add_argument("--domain-id", type=int, default=0, help="DDS ドメイン ID")
    parser.add_argument("--duration", type=float, default=90.0, help="収録する秒数")
    parser.add_argument("--out", default="Mapping/data/scans_real.npz", help="出力先の npz")
    parser.add_argument(
        "--max-hz",
        type=float,
        default=10.0,
        help="収録する上限フレームレート[Hz]。LiDAR がこれより速く流してきたら間引く",
    )
    args = parser.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

    scan_points: list[np.ndarray] = []
    scan_times: list[float] = []
    lock = threading.Lock()
    state = {"received": 0, "described": False, "start": 0.0, "next_keep": 0.0}

    def on_cloud(msg) -> None:  # type: ignore[no-untyped-def]
        now = time.monotonic()
        with lock:
            state["received"] += 1
            if not state["described"]:
                print(f"[lidar] {describe(msg)}", flush=True)
                state["described"] = True
            if now < state["next_keep"]:
                return  # --max-hz を超えるぶんは捨てる
            state["next_keep"] = now + 1.0 / args.max_hz
            try:
                points = to_xyz(msg)
            except ValueError as e:
                print(f"[warn] デコードできないフレームを飛ばした: {e}", flush=True)
                return
            scan_points.append(points.astype(np.float32))
            scan_times.append(now - state["start"])

    ChannelFactoryInitialize(args.domain_id, args.network_interface)
    subscriber = ChannelSubscriber(args.topic, PointCloud2_)
    subscriber.Init(on_cloud, 10)

    state["start"] = time.monotonic()
    print(f"[record] {args.topic} を {args.duration:.0f} 秒間収録する", flush=True)
    print("[record] この間に G1 を手動で歩かせて、部屋をひと回りさせること", flush=True)

    deadline = state["start"] + args.duration
    last_report = 0.0
    while time.monotonic() < deadline:
        time.sleep(0.5)
        elapsed = time.monotonic() - state["start"]
        if elapsed - last_report >= 5.0:
            last_report = elapsed
            with lock:
                kept, received = len(scan_points), state["received"]
            print(f"[record] {elapsed:5.1f}s  保存 {kept} フレーム / 受信 {received}", flush=True)

    subscriber.Close()

    if not scan_points:
        print("", flush=True)
        print(f"[error] {args.topic} から 1 フレームも受信できなかった。", flush=True)
        print("[error] python Mapping/real/discover_topics.py でトピック名を確認すること。", flush=True)
        raise SystemExit(1)

    counts = np.array([len(p) for p in scan_points], dtype=np.int64)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    # gt_poses は入れない。実機に姿勢の真値は無いので、run_slam.py は誤差評価を飛ばす。
    np.savez_compressed(
        out_path,
        points=np.concatenate(scan_points, axis=0),
        counts=counts,
        times=np.array(scan_times, dtype=np.float64),
    )

    print("", flush=True)
    print(f"[result] {len(scan_points)} フレーム / {counts.sum()} 点 を保存: {out_path}", flush=True)
    print(f"[result] 1 フレームあたりの点数: 平均 {counts.mean():.0f} / 最小 {counts.min()}", flush=True)
    print(f"[next] python Mapping/run_slam.py --scans {out_path}", flush=True)


if __name__ == "__main__":
    main()
