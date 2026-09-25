#!/usr/bin/env python3
"""巡回路のウェイポイントを**いまの機体の位置から**記録する。

## なぜ手で書かないのか

`room_a_map.pgm` は 2026-09-07 取得で現状と合っていない（HANDOVER ②）。
地図の画像を見て座標を決めても、そこが実際に通れる場所とは限らない。
**機体を実際にその場所へ持って行って、そのときの `map→base_link` を拾う**のが
いちばん確実で、地図のずれも `map→odom` の補正も込みで正しい値になる。

## 使い方

Nav2 を上げて **§7 の地図照合まで済ませた状態**（＝ `map→odom` が入っている状態）で:

    python3 record_waypoints.py -o patrol_room_a.yaml

機体を回りたい場所へ移動させ、そのつど **Enter**。`q` + Enter で保存して終わる。
名前を付けたいときは Enter の代わりに名前を打つ。

    [1] 位置を記録する(Enter) / 名前を入れる / q=保存して終了 > 入口
    ✅ 入口 (2.13, -4.55, 87deg)

⚠️ **機体は「巡回で通ってほしい向き」に向けておくこと。** yaw も一緒に記録する。
⚠️ **`map→odom` が入る前に記録すると座標がまるごとずれる。** 先に §7 を済ませること。
   （このスクリプトは `map→odom` が恒等変換のままだと警告を出す）
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", required=True, help="書き出す yaml")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    # ⚠️ **1.3 秒を超えると機体が FAULT に落ちる**（velocity_smoother の
    # velocity_timeout 1.0 + cmd_timeout 0.30。findings/patrol_mode.md §6）。
    # 既定はノード側の 0 に任せる。
    ap.add_argument("--dwell", type=float, default=None,
                    help="各点の dwell_s を明示する。⚠️ 1.3秒を超えると FAULT に落ちる")
    args = ap.parse_args()

    rclpy.init()
    node = Node("record_waypoints")
    buf = Buffer()
    # ⚠️ 戻り値を捨てると購読ごと GC される。**必ず参照を保持すること**
    listener = TransformListener(buf, node)
    assert listener is not None

    def lookup(target: str, source: str):
        deadline = node.get_clock().now().nanoseconds + 3_000_000_000
        while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                return buf.lookup_transform(target, source, Time())
            except Exception:  # noqa: BLE001 - 出るまで待つだけ
                continue
        return None

    if lookup(args.map_frame, args.base_frame) is None:
        print(f"❌ {args.map_frame}→{args.base_frame} の TF が来ない。"
              "Nav2 と g1_slam_odom_tf.py が動いているか確認すること", file=sys.stderr)
        return 1

    mo = lookup(args.map_frame, "odom")
    if mo is not None:
        t = mo.transform.translation
        if abs(t.x) < 1e-6 and abs(t.y) < 1e-6 and abs(yaw_of(mo.transform.rotation)) < 1e-6:
            print("⚠️ map→odom が恒等変換のまま。**§7 の地図照合をまだやっていないのでは。**")
            print("   このまま記録すると、次のセッションでは座標がまるごとずれる。")

    points = []
    print(f"記録先: {args.out}  （{args.map_frame} 座標系）")
    try:
        while True:
            label = input(f"[{len(points) + 1}] 記録=Enter / 名前を入れる / q=保存して終了 > ").strip()
            if label.lower() == "q":
                break
            tf = lookup(args.map_frame, args.base_frame)
            if tf is None:
                print("⚠️ TF が取れなかった。もう一度")
                continue
            t = tf.transform.translation
            yaw = math.degrees(yaw_of(tf.transform.rotation))
            name = label or f"wp{len(points) + 1}"
            pt = {"name": name, "x": round(t.x, 3), "y": round(t.y, 3),
                  "yaw_deg": round(yaw, 1)}
            if args.dwell is not None:
                pt["dwell_s"] = args.dwell
            points.append(pt)
            print(f"  ✅ {name} ({pt['x']}, {pt['y']}, {pt['yaw_deg']}deg)")
    except (EOFError, KeyboardInterrupt):
        print()

    if not points:
        print("1点も記録していないので書き出さない")
        return 1

    with open(args.out, "w", encoding="utf-8") as f:
        f.write(f"# {args.base_frame} の実位置から記録した巡回路（record_waypoints.py）\n")
        f.write("# ⚠️ map→odom（§7 の地図照合）が入った状態で記録したもの。\n")
        f.write("#    地図を作り直したら記録もやり直すこと。\n")
        f.write(f"frame_id: {args.map_frame}\n")
        f.write("waypoints:\n")
        for p in points:
            extra = f", dwell_s: {p['dwell_s']}" if "dwell_s" in p else ""
            f.write(f"  - {{name: {p['name']}, x: {p['x']}, y: {p['y']}, "
                    f"yaw_deg: {p['yaw_deg']}{extra}}}\n")
    print(f"📄 {len(points)} 点を {args.out} に書いた")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
