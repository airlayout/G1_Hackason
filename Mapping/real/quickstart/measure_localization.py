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
| **真値とのずれ** | `--traj` を渡したとき | **中央 ≤ 0.30 m** |

## ⚠️ **ICP 品質は測位が合っている証拠にならない**（2026-09-08 に 2 度踏んだ）

段 C で「間違った場所での ICP 品質は 0.95〜1.00 で平常の 0.851 より高い」ことが
分かっている。さらに再生では、**MOLA が古い姿勢から始まって真値から 10〜19 m
ずれたまま品質だけ高い**状態になった（`nav_stack.sh` の `--delay` の注記を参照）。

再生には真値（`mola_floor0/traj.txt`）があるのだから、**必ず突き合わせる。**

    ... measure_localization.py --sec 40 --sim-time \
        --traj /work/G1_Hackason/Mapping/real/runs/<id>/mola_floor0/traj.txt

⚠️ 経路までの最短距離で測ってはいけない。軌跡が同じ場所を往復するので、
**時刻で突き合わせる**こと（この道具はそうしている）。

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
import bisect
import math
import statistics
from pathlib import Path
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
# 真値とのずれ。robot_radius と同じ値。これを超えると costmap の判定が無意味になる
TRUTH_XY_MAX = 0.30


class Measure(Node):
    def __init__(self, settle_sec: float) -> None:
        super().__init__("measure_localization")
        self._settle_sec = settle_sec
        self._t0 = time.monotonic()
        self.quality: list[float] = []
        self.map_odom_stamps: list[float] = []
        self.pose_z: list[float] = []
        self.pose_xy: list[tuple[float, float]] = []
        self.pose_t: list[float] = []          # ヘッダ時刻。真値との突き合わせに使う

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
            self.pose_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

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


def _truth_errors(pose_t: list[float], pose_xy: list[tuple[float, float]],
                  traj_path: str) -> "list[float] | None":
    """記録の真値（TUM）と**時刻で**突き合わせ、xy のずれ[m]を返す。"""
    rows = []
    for line in Path(traj_path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 3:
            rows.append((float(parts[0]), float(parts[1]), float(parts[2])))
    if not rows:
        print(f"  ⚠️ 真値が読めない: {traj_path}")
        return None
    rows.sort()
    ts = [r[0] for r in rows]
    inside = [(t, q) for t, q in zip(pose_t, pose_xy) if ts[0] <= t <= ts[-1]]
    if not inside:
        print(f"  ⚠️ 真値と重なる時刻が無い（姿勢 {pose_t[0]:.1f}〜{pose_t[-1]:.1f} / "
              f"真値 {ts[0]:.1f}〜{ts[-1]:.1f}）。--sim-time を付け忘れていないか")
        return None
    errors = []
    for t, (x, y) in inside:
        i = bisect.bisect_left(ts, t)
        i = min(max(i, 1), len(rows) - 1)
        t0, x0, y0 = rows[i - 1]
        t1, x1, y1 = rows[i]
        w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        errors.append(math.dist((x, y), (x0 + w * (x1 - x0), y0 + w * (y1 - y0))))
    return errors


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
    p.add_argument("--traj", help="記録の真値（TUM）。渡すと時刻で突き合わせてずれを出す。"
                                  "**再生では必ず渡すこと**（品質は合っている証拠にならない）")
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

    if args.traj:
        errors = _truth_errors(node.pose_t, node.pose_xy, args.traj)
        if errors:
            med = statistics.median(errors)
            _stats("真値とのずれ", errors, " m")
            checks.append((f"真値とのずれ 中央 ≤ {TRUTH_XY_MAX} m",
                           med <= TRUTH_XY_MAX, f"{med:.3f} m"))
            if med > TRUTH_XY_MAX:
                print("  ⚠️ **測位が別の場所に食いついている。この実行の測定は全部無効。**")
                print("     再生開始と MOLA の起動がずれている疑い（G1_BAG_DELAY を増やす）")
        else:
            checks.append(("真値と突き合わせられた", False, "標本 0"))
    else:
        print("  ⚠️ --traj を渡していないので、測位が合っているかは分からない"
              "（ICP 品質は証拠にならない）")

    for label, ok, got in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}  実測 {got}")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if all(ok for _, ok, _ in checks) else 1


if __name__ == "__main__":
    sys.exit(main())
