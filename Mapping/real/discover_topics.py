#!/usr/bin/env python
"""[エントリポイント] G1 が DDS に流しているトピックを列挙する。**受信のみ。**

LiDAR のトピック名は `unitree_sdk2py` のどこにも書かれていない（同梱の LiDAR 関連
定義は Go2 用の `rt/utlidar/switch` だけ）。名前を推測して総当たりするより、
DDS の組み込みトピック（DCPSPublication）を読んで**実際に publish されている
ものを一覧する**ほうが確実なので、そうしている。

このスクリプトはロボットに一切コマンドを送らない。読むだけなので、
G1 が手動操作で歩いている最中に実行しても安全。

前提: 操作 PC と G1 が Ethernet で直結され、同一サブネットにいること
（`Common/network/README.md` の手順 1〜2）。

使い方:
    python Mapping/real/discover_topics.py --network-interface enp3s0
"""
from __future__ import annotations

import argparse
import time


def main() -> None:
    parser = argparse.ArgumentParser(description="G1 が publish している DDS トピックを列挙する")
    parser.add_argument(
        "--network-interface",
        default="enp3s0",
        help="G1 と繋がっている Ethernet インターフェース名（Common/network/ の設定と合わせる）",
    )
    parser.add_argument("--domain-id", type=int, default=0, help="DDS ドメイン ID")
    parser.add_argument("--duration", type=float, default=10.0, help="探索する秒数")
    args = parser.parse_args()

    from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
    from cyclonedds.domain import DomainParticipant
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    # 先に SDK 側で Domain を作らせる。ネットワークインターフェースを CycloneDDS の
    # 設定に反映するのはこの呼び出しなので、自前の DomainParticipant より前に必要。
    ChannelFactoryInitialize(args.domain_id, args.network_interface)
    participant = DomainParticipant(args.domain_id)
    reader = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)

    print(f"[discover] {args.network_interface} で {args.duration:.0f} 秒間さがす...", flush=True)
    found: dict[str, str] = {}
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        for sample in reader.take(N=100):
            topic = getattr(sample, "topic_name", None)
            if topic and topic not in found:
                found[topic] = getattr(sample, "type_name", "?")
                print(f"  [+] {topic}  ({found[topic]})", flush=True)
        time.sleep(0.2)

    print("", flush=True)
    if not found:
        print("[result] トピックが 1 つも見つからなかった。次を確認すること:", flush=True)
        print("  - G1 の電源が入っていて、Ethernet で繋がっているか", flush=True)
        print("  - python Common/network/check_g1_connectivity.py が READY になるか", flush=True)
        print(f"  - --network-interface {args.network_interface} が実際の口と合っているか", flush=True)
        return

    print(f"[result] {len(found)} 件のトピックが見つかった", flush=True)
    clouds = {t: k for t, k in found.items() if "PointCloud" in k}
    if clouds:
        print("[result] 点群トピックの候補:", flush=True)
        for topic, type_name in clouds.items():
            print(f"  {topic}  ({type_name})", flush=True)
        first = next(iter(clouds))
        print("", flush=True)
        print(f"[next] python Mapping/real/record_scans_real.py --topic {first}", flush=True)
    else:
        print("[result] PointCloud2 のトピックは見つからなかった。", flush=True)
        print("[result] LiDAR が OFF になっている可能性がある（Go2 では rt/utlidar/switch で切り替える）。", flush=True)


if __name__ == "__main__":
    main()
