#!/usr/bin/env python3
"""`/projected_map` の変化を**時刻つきで**記録する。段 6 の計測器。

## なぜ要るか

計画 `docs/plan/2026-09-16-growing-map-octomap.md` の段 6 の合否は**時間**である:

| 見るもの | 合格 |
|---|---|
| 机を新しい場所に置く → 2D に現れる | **30 秒以内** |
| 元の場所の幽霊が消える | その前を 1 回通って **60 秒以内** |

スクリーンショットでは時刻が取れない。占有セルの集合を毎回保存して、
**基準（最初に受けた地図）に対して現れた／消えたセル**を数え、秒で出す。

⚠️ **合計のセル数では分からない。**現れたセルと消えたセルが打ち消し合う
（2026-09-16 に種の残存を測ったときに確認した。合計 +1,942 の裏で 446 消えていた）。
だから集合の差を取る。

## 使い方（コンテナの中で。Mac に rclpy は無い）

    docker exec -u ubuntu -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI="$DDS" rviz bash -lc \\
      'source /opt/ros/humble/setup.bash && \\
       python3 /work/.../quickstart/watch_projected_map.py --seconds 180 \\
         --out /work/.../runs/_live/<時刻>/projected'

    # 机を置く場所だけ見る（xy の範囲[m]。map 系）
    ... --box 1.0 3.0 -1.0 1.0

出るもの:
- `<out>/timeline.tsv` … 1 行 1 メッセージ。経過秒・占有・現れた・消えた
- `<out>/first.pgm` `<out>/last.pgm` ＋ `.yaml` … 前後の地図（既存の読み手が読める）
- `<out>/events.txt` … しきい値を初めて超えた時刻（合否に使う）

⚠️ **AP 構成では中継が 0.5 Hz に間引いている**（`foxglove_to_ros.py`）。
30 秒の判定には十分だが、`--box` を小さく取ると 1 サンプルの重みが大きい。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dump_grids import write_layer  # noqa: E402

# 「現れた／消えた」と言い切る**最大の連結塊**のセル数。
# ⚠️⚠️ **合計セル数で判定してはいけない。**2026-09-17 に実機で踏んだ:
# 何も動かしていない数分間で合計 29 セルが「現れ」30 セルが「消えた」が、
# 中身は **18〜22 個の 1〜9 セルの塊**に散っていた ＝ 光線の当たり方の揺れ（雑音）。
# しきい値 20 を合計に当てたら**雑音で発火した**（経過 6.9 s と 25.2 s）。
# 机 1 台は 0.1 m 格子で **1 箇所に 25〜80 セルの塊**として出る。塊で見れば分離できる。
APPEAR_CELLS = 25
VANISH_CELLS = 25
# これ以下の塊は数えない（雑音の実測上限は 9 セルだった）
NOISE_CLUSTER = 12


def largest_cluster(cells: "set[tuple[int, int]]") -> "tuple[int, tuple[float, float] | None]":
    """8 近傍で連結した塊のうち最大のセル数と、その重心[m]を返す。

    scipy は使わない。⚠️ **コンテナの scipy は numpy2 と噛み合わない**ので、
    ここが落ちると計測そのものが止まる。対象は毎回数十セルなので素朴な BFS で足る。
    """
    remaining = set(cells)
    best: "list[tuple[int, int]]" = []
    while remaining:
        seed = remaining.pop()
        blob, queue = [seed], [seed]
        while queue:
            cx, cy = queue.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbour = (cx + dx, cy + dy)
                    if neighbour in remaining:
                        remaining.discard(neighbour)
                        blob.append(neighbour)
                        queue.append(neighbour)
        if len(blob) > len(best):
            best = blob
    if not best:
        return 0, None
    xs = [c[0] * 0.1 + 0.05 for c in best]
    ys = [c[1] * 0.1 + 0.05 for c in best]
    return len(best), (sum(xs) / len(xs), sum(ys) / len(ys))


def map_qos() -> QoSProfile:
    """中継が LATCHED で出し直すので TRANSIENT_LOCAL で購読する。

    ⚠️ 機体側の `octomap_server` は `latch:=false`（VOLATILE）だが、
    こちらが見るのは**中継が republish した側**なので TRANSIENT_LOCAL が正しい。
    有線構成で機体の DDS を直接見るときは VOLATILE に落とすこと（`--volatile`）。
    """
    return QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def occupied_keys(grid: OccupancyGrid,
                  box: "tuple[float, float, float, float] | None") -> "set[tuple[int, int]]":
    """占有セルを世界座標の整数キーにする。原点が違っても比べられる。"""
    data = np.asarray(grid.data, dtype=np.int16).reshape(
        grid.info.height, grid.info.width)
    rows, cols = np.nonzero(data >= 65)
    resolution = grid.info.resolution
    x = grid.info.origin.position.x + (cols + 0.5) * resolution
    y = grid.info.origin.position.y + (rows + 0.5) * resolution
    if box is not None:
        keep = (x >= box[0]) & (x <= box[1]) & (y >= box[2]) & (y <= box[3])
        x, y = x[keep], y[keep]
    keys = np.floor(np.c_[x, y] / resolution).astype(np.int64)
    return set(map(tuple, keys.tolist()))


class Watcher(Node):
    def __init__(self, topic: str, box, transient_local: bool) -> None:
        super().__init__("watch_projected_map")
        self.box = box
        self.baseline: "set[tuple[int, int]] | None" = None
        self.first: "OccupancyGrid | None" = None
        self.last: "OccupancyGrid | None" = None
        self.rows: "list[tuple[float, int, int, int]]" = []
        self.began = time.time()
        self.appeared_at: "float | None" = None
        self.vanished_at: "float | None" = None
        self.appeared_where: "tuple[float, float] | None" = None
        self.vanished_where: "tuple[float, float] | None" = None
        qos = map_qos()
        if not transient_local:
            qos.durability = QoSDurabilityPolicy.VOLATILE
        self.create_subscription(OccupancyGrid, topic, self._on_map, qos)

    def _on_map(self, message: OccupancyGrid) -> None:
        keys = occupied_keys(message, self.box)
        elapsed = time.time() - self.began
        if self.baseline is None:
            self.baseline, self.first = keys, message
            print(f"[watch] 基準 {len(keys):,} セル（経過 {elapsed:.1f} s）", flush=True)
        appeared_cells = keys - self.baseline
        vanished_cells = self.baseline - keys
        appeared, at_appeared = largest_cluster(appeared_cells)
        vanished, at_vanished = largest_cluster(vanished_cells)
        self.last = message
        self.rows.append((elapsed, len(keys), len(appeared_cells), len(vanished_cells),
                          appeared, vanished))
        if self.appeared_at is None and appeared >= APPEAR_CELLS:
            self.appeared_at = elapsed
            self.appeared_where = at_appeared
            print(f"[watch] ★ 現れた: 塊 {appeared} セル @({at_appeared[0]:+.2f},"
                  f"{at_appeared[1]:+.2f}) / 経過 {elapsed:.1f} s", flush=True)
        if self.vanished_at is None and vanished >= VANISH_CELLS:
            self.vanished_at = elapsed
            self.vanished_where = at_vanished
            print(f"[watch] ★ 消えた: 塊 {vanished} セル @({at_vanished[0]:+.2f},"
                  f"{at_vanished[1]:+.2f}) / 経過 {elapsed:.1f} s", flush=True)
        if len(self.rows) % 5 == 0:
            print(f"[watch] {elapsed:6.1f} s  占有 {len(keys):6,}  "
                  f"現れた {len(appeared_cells):4,}(塊 {appeared:3,})  "
                  f"消えた {len(vanished_cells):4,}(塊 {vanished:3,})", flush=True)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", type=Path, required=True, help="書き出し先のディレクトリ")
    p.add_argument("--topic", default="/projected_map")
    p.add_argument("--seconds", type=float, default=180.0, help="見る時間[s]")
    p.add_argument("--box", type=float, nargs=4, default=None,
                   metavar=("X0", "X1", "Y0", "Y1"),
                   help="この xy の範囲[m]だけ見る（map 系）。机の周りに絞るとき")
    p.add_argument("--volatile", action="store_true",
                   help="VOLATILE で購読する。機体の octomap_server を直接見るとき")
    a = p.parse_args()

    a.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    box = tuple(a.box) if a.box else None
    node = Watcher(a.topic, box, transient_local=not a.volatile)
    print(f"[watch] {a.topic} を {a.seconds:.0f} 秒見る"
          + (f" / 範囲 x {box[0]}〜{box[1]} y {box[2]}〜{box[3]}" if box else " / 全域"),
          flush=True)

    deadline = time.time() + a.seconds
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    if node.first is None:
        print("[watch] ⛔ /projected_map が 1 通も来なかった。"
              "中継（AP 構成）か QoS（--volatile）を疑う", file=sys.stderr)
        return 1

    write_layer(a.out, "first", node.first)
    write_layer(a.out, "last", node.last)
    header = ("elapsed_s\toccupied\tappeared\tvanished"
              "\tappeared_cluster\tvanished_cluster\n")
    (a.out / "timeline.tsv").write_text(
        header + "".join(f"{t:.2f}\t{o}\t{ap}\t{va}\t{ac}\t{vc}\n"
                         for t, o, ap, va, ac, vc in node.rows))

    lines = [f"受けた地図 {len(node.rows)} 通 / {a.seconds:.0f} 秒",
             f"基準の占有 {len(node.baseline):,} セル",
             f"最後の占有 {node.rows[-1][1]:,} セル "
             f"（現れた {node.rows[-1][2]:,}・最大の塊 {node.rows[-1][4]:,} / "
             f"消えた {node.rows[-1][3]:,}・最大の塊 {node.rows[-1][5]:,}）",
             f"現れた（塊 {APPEAR_CELLS} セル以上）: "
             + (f"{node.appeared_at:.1f} s @{node.appeared_where}"
                if node.appeared_at is not None else "起きなかった"),
             f"消えた（塊 {VANISH_CELLS} セル以上）: "
             + (f"{node.vanished_at:.1f} s @{node.vanished_where}"
                if node.vanished_at is not None else "起きなかった")]
    (a.out / "events.txt").write_text("\n".join(lines) + "\n")
    print("\n" + "\n".join(lines))
    print(f"\n[watch] 出力: {a.out}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
