#!/usr/bin/env python3
"""測位の合否を数字で出す。**再生でも実機でも同じ基準で測る。**

    docker exec -u ubuntu -e ... rviz bash -c \
      "source /opt/ros/humble/setup.bash && \
       python3 /work/G1_Hackason/Mapping/real/quickstart/measure_localization.py --sec 60"

## 何を測るか

| 量 | 出どころ | 合格 |
|---|---|---|
| ICP 品質 | `/lidar_odometry/pose_quality` | 中央値 ≥ 0.70・最小 ≥ 0.50 |
| TF の頻度 | `/tf` の `map -> odom` | ≥ 8 Hz |
| base_link の z | `/lidar_odometry/pose` | 参考（振れ幅を見る） |

基準は `docs/plan/2026-09-08-global-localization-and-move.md` §4 で
**測る前に決めたもの**。閾値を引数で緩められるようにしていないのは意図的。

## ⚠️ 使うときの注意

- **再生では `--sim-time` が要る。** /clock を使う側と時計が合わないと、
  購読はできても「何 Hz か」が実時間になり、倍速再生で読み間違える。
  頻度は**壁時計**で測るので（再生の倍速をそのまま反映する）、
  等速再生（`nav_stack.sh --rate 1`）で測ること
- **段 C（再定位）で使うときは `--sec` を復帰時間より長く取る。**
  ずらした直後の低品質が中央値を押し下げるので、`--settle` で先頭を捨てられる
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

# 合否（測る前に決めた値。ここを緩めないこと）
QUALITY_MEDIAN_MIN = 0.70
QUALITY_ABS_MIN = 0.50
TF_HZ_MIN = 8.0


class Measure(Node):
    def __init__(self, settle_sec: float) -> None:
        super().__init__("measure_localization")
        self._settle_sec = settle_sec
        self._t0 = time.monotonic()
        self.quality: list[float] = []
        self.map_odom_stamps: list[float] = []
        self.pose_z: list[float] = []
        self.pose_xy: list[tuple[float, float]] = []

        # MOLA 側は既定の QoS（reliable/volatile）で出す
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Float32, "/lidar_odometry/pose_quality", self._on_quality, sensor_qos)
        self.create_subscription(Odometry, "/lidar_odometry/pose", self._on_pose, sensor_qos)
        self.create_subscription(TFMessage, "/tf", self._on_tf, sensor_qos)

    def _settled(self) -> bool:
        return (time.monotonic() - self._t0) >= self._settle_sec

    def _on_quality(self, msg: Float32) -> None:
        if self._settled():
            self.quality.append(float(msg.data))

    def _on_pose(self, msg: Odometry) -> None:
        if self._settled():
            p = msg.pose.pose.position
            self.pose_z.append(float(p.z))
            self.pose_xy.append((float(p.x), float(p.y)))

    def _on_tf(self, msg: TFMessage) -> None:
        if not self._settled():
            return
        # MOLA が出すのは map -> odom（rep105）。頻度は**壁時計**で数える
        for tr in msg.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                self.map_odom_stamps.append(time.monotonic())


def _dip_report(values: list[float], threshold: float, hz_hint: float) -> None:
    """閾値を割った標本を「単発の落ち込み」と「持続」に分けて出す。

    **単発（1〜2 スキャン）と持続は意味が違う。** 前者は 7% ある stamp=0 の
    スキャンや deskew の失敗で 1 枚だけ品質が落ちるもので、次のスキャンで戻る。
    後者は**測位を失っている**ので、閾値を割ったかどうかより重い。
    合否の閾値は動かさない（§4）。ここは中身を説明するためだけの出力である。
    """
    below = [i for i, v in enumerate(values) if v < threshold]
    if not below:
        print(f"  {threshold} を割った標本: なし")
        return
    runs: list[list[int]] = [[below[0]]]
    for i in below[1:]:
        if i == runs[-1][-1] + 1:
            runs[-1].append(i)
        else:
            runs.append([i])
    longest = max(len(r) for r in runs)
    sec = longest / hz_hint if hz_hint > 0 else float("nan")
    print(
        f"  {threshold} を割った標本: {len(below)}/{len(values)} 件"
        f"（{100 * len(below) / len(values):.1f}%）を {len(runs)} 個の塊に分けた。"
        f"最長の連続は {longest} スキャン ≈ {sec:.2f} 秒"
    )
    for r in sorted(runs, key=len, reverse=True)[:3]:
        vals = [values[i] for i in r]
        print(f"    - {len(r)} スキャン連続 / 最小 {min(vals):.3f}（標本 {r[0]}〜{r[-1]}）")


def _stats(name: str, values: list[float], unit: str = "") -> None:
    if not values:
        print(f"  {name}: **1 件も来ていない**")
        return
    ordered = sorted(values)
    p05 = ordered[max(0, int(0.05 * len(ordered)) - 1)]
    print(
        f"  {name}: n={len(values)} 中央値={statistics.median(values):.3f}{unit} "
        f"p05={p05:.3f}{unit} 最小={min(values):.3f}{unit} 最大={max(values):.3f}{unit} "
        f"振れ幅={max(values) - min(values):.3f}{unit}"
    )


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sec", type=float, default=60.0, help="測る秒数（settle を除く）")
    p.add_argument("--settle", type=float, default=5.0,
                   help="先頭で捨てる秒数。段 C では復帰待ちに使う")
    p.add_argument("--sim-time", action="store_true",
                   help="再生（/clock）に合わせる。**再生では付けること**")
    args = p.parse_args(argv)

    rclpy.init()
    node = Measure(args.settle)
    if args.sim_time:
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    deadline = time.monotonic() + args.settle + args.sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    print()
    print(f"== 測位の実測（{args.sec:.0f} 秒 / 先頭 {args.settle:.0f} 秒は捨てた）==")
    _stats("ICP 品質", node.quality)
    if node.quality:
        _dip_report(node.quality, QUALITY_ABS_MIN, len(node.quality) / args.sec)
    _stats("base_link の z", node.pose_z, " m")
    if node.pose_xy:
        import math as _m
        x0, y0 = node.pose_xy[0]
        disp = [_m.dist((x0, y0), q) for q in node.pose_xy]
        xs = [q[0] for q in node.pose_xy]; ys = [q[1] for q in node.pose_xy]
        print(f"  xy のふらつき: 開始点からの最大 {max(disp):.3f} m / 最後 {disp[-1]:.3f} m "
              f"（x {min(xs):+.2f}〜{max(xs):+.2f} / y {min(ys):+.2f}〜{max(ys):+.2f}）")
        print("    ↑ **機体が静止している区間で測ること。**段 C（再定位）の残差は"
              "この値と比べないと意味が無い")

    hz = 0.0
    if len(node.map_odom_stamps) >= 2:
        span = node.map_odom_stamps[-1] - node.map_odom_stamps[0]
        hz = (len(node.map_odom_stamps) - 1) / span if span > 0 else 0.0
    print(f"  map -> odom の /tf: n={len(node.map_odom_stamps)} {hz:.2f} Hz（壁時計）")

    print()
    print("== 合否（docs/plan/2026-09-08-global-localization-and-move.md §4）==")
    checks: list[tuple[str, bool, str]] = []
    if node.quality:
        med, lo = statistics.median(node.quality), min(node.quality)
        checks.append((f"ICP 品質 中央値 ≥ {QUALITY_MEDIAN_MIN}", med >= QUALITY_MEDIAN_MIN, f"{med:.3f}"))
        checks.append((f"ICP 品質 最小 ≥ {QUALITY_ABS_MIN}", lo >= QUALITY_ABS_MIN, f"{lo:.3f}"))
    else:
        checks.append(("ICP 品質が取れた", False, "0 件"))
    checks.append((f"map->odom の /tf ≥ {TF_HZ_MIN} Hz", hz >= TF_HZ_MIN, f"{hz:.2f} Hz"))

    for label, ok, got in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}  実測 {got}")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if all(ok for _, ok, _ in checks) else 1


if __name__ == "__main__":
    sys.exit(main())
