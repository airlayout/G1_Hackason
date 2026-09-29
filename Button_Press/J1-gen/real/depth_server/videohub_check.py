"""Unitree の videohub（PC2 でカラーのカメラ /dev/video4 を開いているサービス）から、カラー画像を受け取れるかを確かめる。

videohub のプログラムの中に rt/api/videohub/request と rt/api/videohub/response という名前があった
（2026-09-29）。これは unitree_sdk2py の Go2 用 VideoClient が使う「videohub」サービスと同じ形なので、
そのまま G1 でも使えるかを試す。画像を 1 枚ずつ頼んで JPEG を受け取るだけで、videohub の設定や
ほかのメンバーへの配信は変えない。

PC2 の中で動かす（DDS は PC2 の eth0 のネットワークで話す。ノート PC までは届かない）。

    python3 ~/button_press/real/depth_server/videohub_check.py
    python3 ~/button_press/real/depth_server/videohub_check.py --iface eth0 --count 30

受け取った 1 枚目を ~/button_press/videohub_sample.jpg に保存する。
PC2 のシステムの Python 3.8 で動くように書く。
"""

from __future__ import annotations

import argparse
import os
import sys
import time


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--iface", default="eth0", help="DDS で使うネットワーク（videohub の cyclonedds.xml は eth0）")
    p.add_argument("--count", type=int, default=30, help="頼む枚数")
    p.add_argument("--timeout", type=float, default=3.0, help="1 枚あたりの待ち時間 [秒]")
    p.add_argument("--out", default=os.path.expanduser("~/button_press/videohub_sample.jpg"))
    args = p.parse_args()

    import cv2
    import numpy as np
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient

    ChannelFactoryInitialize(0, args.iface)
    client = VideoClient()
    client.SetTimeout(args.timeout)
    client.Init()
    print(f"[videohub_check] {args.iface} で videohub に画像を頼む（{args.count} 枚）")

    got = 0
    t0 = time.time()
    for n in range(1, args.count + 1):
        code, data = client.GetImageSample()
        if code != 0:
            print(f"[videohub_check] ❌ {n} 枚目: 受け取れない（code={code}）")
            if got == 0:
                return 1
            continue
        raw = bytes(data)
        img = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            print(f"[videohub_check] ❌ {n} 枚目: {len(raw)} バイト受け取ったが JPEG として読めない（先頭 {raw[:8]!r}）")
            continue
        got += 1
        if got == 1:
            os.makedirs(os.path.dirname(args.out), exist_ok=True)
            with open(args.out, "wb") as f:
                f.write(raw)
            h, w = img.shape[:2]
            print(f"[videohub_check] ✅ 1 枚目: {w}x{h}、JPEG {len(raw) / 1024:.0f} KB、"
                  f"{time.time() - t0:.2f} 秒。保存: {args.out}")
    dt = time.time() - t0
    print(f"[videohub_check] {got}/{args.count} 枚を {dt:.1f} 秒で受け取った（{got / dt:.1f} 枚/秒）")
    return 0 if got else 1


if __name__ == "__main__":
    sys.exit(main())
