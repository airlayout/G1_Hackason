#!/usr/bin/env python3
"""**わざとずらして再定位できるか**を測る。これが手動版の大域再定位の心臓。

    docker exec -u ubuntu -e ... rviz bash -c \
      "source /opt/ros/humble/setup.bash && \
       python3 /work/G1_Hackason/Mapping/real/quickstart/check_relocalize.py"

## なぜ要るか

MOLA の ROS ブリッジは **`/initialpose` を購読する**
（`BridgeROS2.h`: `relocalize_from_topic = "/initialpose"  //!< Default in RViz`）。
つまり **RViz2 の「2D Pose Estimate」で再定位できる**。サービス
`/relocalize_near_pose` も出る（2026-09-08 に実機なしで購読者 1・サービス在を確認）。
ここが通れば「記録開始点の近くに置かないと収束しない」という制約が消える。

## 測り方（docs/plan/2026-09-08-global-localization-and-move.md §4 段 C）

1. 何もせずに ICP 品質の平常値を見る
2. 現在の姿勢を **1 m / 20° ずらして** `/initialpose`（またはサービス）に投げる
3. **10 秒以内に品質が 0.70 以上へ戻る**か

## ⚠️⚠️ z を 0 で投げてはいけない（**RViz2 はそれを送る**）

`base_link` の map 系での z は **約 1.30 m**（`mola_floor0` は床が z≈0 なので）。
`geometry_msgs/PoseWithCovarianceStamped` の z に 0 を入れると
**毎回 1.3 m 下に投げ込む**ことになる。2026-09-08 の実測では、横のずらし量を
0.3 / 0.5 / 1.0 m と変えても**最終位置は真値から一律 1.2〜1.3 m ずれ**、
しかも ICP 品質は 0.96 と高いままだった（＝ずれ量ではなく z が効いていた）。

**RViz2 の「2D Pose Estimate」は z=0 を送る。** つまり段 H で人がクリックしても
このずれが入る。`--z rviz` でその再現ができる。

## ⚠️ 二面から見ないと意味が無い

「品質が高いまま」は **成功にも「無視された」にも見える。**
だから受理の証拠（姿勢の飛び、または品質の落ち込み）を**別に**取る。
歩行中は姿勢が自然に動くので、**1 ステップの飛び**で判定する
（歩行は 0.1 秒で 0.05 m 程度、ずらしは 1 m なので分離できる）。
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32

QUALITY_RECOVER = 0.70        # §4 段 C
RECOVER_SEC = 10.0            # §4 段 C
# 受理の証拠とみなす 1 ステップの姿勢の飛び[m]。歩行の 1 ステップ（〜0.05m）と分ける
JUMP_EVIDENCE_M = 0.30


def _qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=50,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class Reloc(Node):
    def __init__(self, sim_time: bool) -> None:
        super().__init__(
            "check_relocalize",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, sim_time)],
        )
        self.samples: list[tuple[float, float]] = []          # (t, quality)
        self.pose_z = 0.0
        self.poses: list[tuple[float, float, float, float]] = []   # (t, x, y, yaw)
        self.create_subscription(Float32, "/lidar_odometry/pose_quality", self._on_q, _qos())
        self.create_subscription(Odometry, "/lidar_odometry/pose", self._on_p, _qos())
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def _on_q(self, msg: Float32) -> None:
        self.samples.append((time.monotonic(), float(msg.data)))

    def _on_p(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.poses.append((time.monotonic(), p.x, p.y, yaw))
        self.pose_z = float(p.z)

    def spin_for(self, sec: float) -> None:
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def make_msg(node: Reloc, x: float, y: float, z: float, yaw: float) -> PoseWithCovarianceStamped:
    m = PoseWithCovarianceStamped()
    m.header.frame_id = "map"
    m.header.stamp = node.get_clock().now().to_msg()
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = x, y, z
    m.pose.pose.orientation.z = math.sin(yaw / 2.0)
    m.pose.pose.orientation.w = math.cos(yaw / 2.0)
    # RViz の 2D Pose Estimate が入れる程度の共分散（位置 0.25, yaw 0.07 rad^2）
    for i, v in ((0, 0.25), (7, 0.25), (35, 0.0685)):
        m.pose.covariance[i] = v
    return m



class Stalled(RuntimeError):
    """測位が止まった（再生が終端に達した等）。**古い値で測り続けないための門番。**

    2026-09-08 に踏んだ: bag が終端に達すると `/lidar_odometry/pose` が止まるが、
    スクリプトは `poses[-1]` を読み続けるので**同じ残差が何度も出て、
    それらしい表になってしまった**（3 回連続で残差 -0.011 m が並んだ）。
    新しい標本が来ていない回は結果ではなく**エラー**として扱う。
    """


def _inject_and_watch(node: Reloc, args, tx: float, ty: float, tz: float, tyaw: float,
                      watch_sec: float) -> tuple[float, float, float]:
    """1 回投げて watch_sec 見る。戻りは (品質の中央値, 最後の x, 最後の y)。"""
    msg = make_msg(node, tx, ty, tz, tyaw)
    node.spin_for(0.3)
    for _ in range(3):
        msg.header.stamp = node.get_clock().now().to_msg()
        node.pub.publish(msg)
        node.spin_for(0.1)
    n0, p0 = len(node.samples), len(node.poses)
    node.spin_for(watch_sec)
    qs = [q for _, q in node.samples[n0:]]
    new_poses = node.poses[p0:]
    if len(qs) < 5 or len(new_poses) < 5:
        raise Stalled(
            f"投げた後に新しい標本がほとんど来ない（品質 {len(qs)} 件 / 姿勢 "
            f"{len(new_poses)} 件）。再生が終端に達したか MOLA が止まった")
    fx, fy = new_poses[-1][1], new_poses[-1][2]
    return (statistics.median(qs), fx, fy)


def _repeat_loop(node: Reloc, args, x: float, y: float, yaw: float, gz: float,
                 base_q: list[float]) -> int:
    """**成否が再現するかを数える。**1 回の結果で「使える」と言わないため。

    真値は最初に 1 度だけ掴む（機体が静止している区間で使うこと）。
    各回: 真値+ずらし を投げて見る → **真値そのものを投げて戻す** → 戻りも記録する。
    """
    ux, uy = math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2)
    print(f"[3] {args.repeat} 回繰り返す（真値 ({x:+.3f}, {y:+.3f}) を基準に固定）")
    print(f"{'回':>3} | {'ずらし後の残差':>13} {'距離':>7} {'品質中央':>8} | "
          f"{'戻した後の距離':>13} {'品質中央':>8}")
    off_res, off_dist, reset_dist = [], [], []
    done = 0
    for k in range(args.repeat):
        gx, gy = x + args.offset_m * ux, y + args.offset_m * uy
        try:
            q1, fx, fy = _inject_and_watch(node, args, gx, gy, gz,
                                           yaw + math.radians(args.offset_deg), args.watch_sec)
            res = (fx - x) * ux + (fy - y) * uy
            dist = math.dist((x, y), (fx, fy))
            q2, rx2, ry2 = _inject_and_watch(node, args, x, y, gz, yaw, args.watch_sec)
            rdist = math.dist((x, y), (rx2, ry2))
        except Stalled as exc:
            print(f"{k + 1:3d} | 打ち切り: {exc}")
            break
        off_res.append(res); off_dist.append(dist); reset_dist.append(rdist)
        done += 1
        print(f"{k + 1:3d} | {res:+13.3f} {dist:7.3f} {q1:8.3f} | {rdist:13.3f} {q2:8.3f}")

    if not off_res:
        print("\n  有効な回が 0。再生の残り時間を確認すること"
              "（記録は 591 秒。G1_BAG_OFFSET と watch_sec の積で足りるように）")
        node.destroy_node(); rclpy.shutdown()
        return 1
    args.repeat = done      # 以降のまとめは**有効な回数**で割る
    print()
    print("== まとめ ==")
    good = sum(1 for r in off_res if abs(r) <= 0.30)
    print(f"  ずらして投げた {args.repeat} 回のうち、投げた方向の残差 ≤ 0.30 m だったのは "
          f"**{good}/{args.repeat}** 回")
    print(f"  残差: 中央 {statistics.median(off_res):+.3f} m / "
          f"最小 {min(off_res):+.3f} / 最大 {max(off_res):+.3f}")
    print(f"  真値からの距離: 中央 {statistics.median(off_dist):.3f} m / 最大 {max(off_dist):.3f}")
    rgood = sum(1 for r in reset_dist if r <= 0.30)
    print(f"  **真値そのものを投げて戻した**とき ≤ 0.30 m だったのは {rgood}/{args.repeat} 回"
          f"（中央 {statistics.median(reset_dist):.3f} m）")
    print()
    print(f"  平常の ICP 品質 中央値は {statistics.median(base_q):.3f}。"
          f"上の「品質中央」がこれと同程度なら、**品質は成否を見分けられない**。")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if good == args.repeat else 1

def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--offset-m", type=float, default=1.0, help="ずらす距離[m]（§4 は 1 m）")
    p.add_argument("--offset-deg", type=float, default=20.0, help="ずらす角度[度]（§4 は 20°）")
    p.add_argument("--baseline-sec", type=float, default=8.0, help="平常値を見る秒数")
    p.add_argument("--watch-sec", type=float, default=25.0, help="投げた後に見る秒数")
    p.add_argument("--z", default="keep",
                   help="投げる z。'keep'=いまの推定 z を使う / 'rviz'=0.0（**RViz2 の "
                        "2D Pose Estimate はこれを送る**）/ 数値でも指定できる")
    p.add_argument("--use-service", action="store_true",
                   help="/initialpose の代わりに /relocalize_near_pose を使う")
    p.add_argument("--no-sim-time", action="store_true", help="実機で使うとき")
    p.add_argument("--repeat", type=int, default=1,
                   help="繰り返し回数。**1 回では判定できない**（2026-09-08 実測で成否が"
                        "再現しなかった）。各回は「ずらして投げる → 見る → 真値を投げて戻す」")
    args = p.parse_args(argv)

    rclpy.init()
    node = Reloc(sim_time=not args.no_sim_time)

    print(f"[1] 平常値を {args.baseline_sec:.0f} 秒見る")
    node.spin_for(args.baseline_sec)
    if not node.samples or not node.poses:
        print("  [FAIL] 測位が出ていない（品質か姿勢が 1 件も来ない）")
        node.destroy_node(); rclpy.shutdown()
        return 1
    base_q = [q for _, q in node.samples]
    print(f"  ICP 品質 中央値 {statistics.median(base_q):.3f} / 最小 {min(base_q):.3f}"
          f"（n={len(base_q)}）")

    _, x, y, yaw = node.poses[-1]
    th = yaw + math.radians(args.offset_deg)
    # 進行方向に対して横へずらす（前後だと歩行と混ざる）
    gx = x + args.offset_m * math.cos(yaw + math.pi / 2)
    gy = y + args.offset_m * math.sin(yaw + math.pi / 2)
    print(f"[2] いまの姿勢 ({x:+.3f}, {y:+.3f}, yaw {math.degrees(yaw):+.1f}deg) を "
          f"{args.offset_m} m / {args.offset_deg} deg ずらして投げる "
          f"-> ({gx:+.3f}, {gy:+.3f}, yaw {math.degrees(th):+.1f}deg)")

    if args.z == "keep":
        gz = node.pose_z
    elif args.z == "rviz":
        gz = 0.0
    else:
        gz = float(args.z)
    print(f"    投げる z = {gz:.3f} m（いまの推定 z は {node.pose_z:.3f} m / "
          f"--z rviz なら 0.0 = RViz2 の 2D Pose Estimate と同じ）")
    msg = make_msg(node, gx, gy, gz, th)
    n_q_before, n_p_before = len(node.samples), len(node.poses)
    t_pub = time.monotonic()

    if args.use_service:
        from mola_msgs.srv import RelocalizeNearPose
        cli = node.create_client(RelocalizeNearPose, "/relocalize_near_pose")
        if not cli.wait_for_service(timeout_sec=5.0):
            print("  [FAIL] /relocalize_near_pose が居ない")
            node.destroy_node(); rclpy.shutdown()
            return 1
        req = RelocalizeNearPose.Request(); req.pose = msg
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        if fut.done() and fut.result() is not None:
            print(f"  サービスの応答: accepted={fut.result().accepted}")
        else:
            print("  [FAIL] サービスが応答しない")
    else:
        # VOLATILE の購読者なので、接続が張れるまで少し待って複数回出す
        node.spin_for(0.5)
        for _ in range(3):
            msg.header.stamp = node.get_clock().now().to_msg()
            node.pub.publish(msg)
            node.spin_for(0.1)
        print(f"  /initialpose に publish した（購読者 {node.pub.get_subscription_count()} 件）")

    if args.repeat > 1:
        return _repeat_loop(node, args, x, y, yaw, gz, base_q)

    print(f"[3] {args.watch_sec:.0f} 秒見る")
    node.spin_for(args.watch_sec)

    after_q = [(t - t_pub, q) for t, q in node.samples[n_q_before:]]
    after_p = node.poses[n_p_before:]
    if not after_q:
        print("  [FAIL] 投げた後に品質が 1 件も来ない")
        node.destroy_node(); rclpy.shutdown()
        return 1

    # 受理の証拠: 1 ステップの姿勢の飛び
    jumps = [
        (after_p[i + 1][0] - t_pub,
         math.dist(after_p[i][1:3], after_p[i + 1][1:3]))
        for i in range(len(after_p) - 1)
    ]
    biggest = max(jumps, key=lambda j: j[1]) if jumps else (0.0, 0.0)
    dip = min(q for _, q in after_q)

    print()
    print("== 投げた後の様子 ==")
    print(f"  最大の 1 ステップ姿勢飛び: {biggest[1]:.3f} m（投げてから {biggest[0]:+.2f} 秒）")
    print(f"  品質の最小値: {dip:.3f}")
    head = [f"{t:+.1f}s:{q:.2f}" for t, q in after_q[:14]]
    print(f"  品質の推移: {' '.join(head)}")

    # 復帰時間: 3 標本連続で閾値以上になった最初の時刻
    recovered_at = None
    run = 0
    for t, q in after_q:
        run = run + 1 if q >= QUALITY_RECOVER else 0
        if run >= 3:
            recovered_at = t
            break

    # ⚠️ **「品質が高い」だけでは足りない。** align_to_map.py が fitness 0.949 で
    # yaw 37° ずれた前例がある（床は平らなのでどこでも合う）。
    # **投げた場所に居座っていないか**を別に確かめる。
    #
    # ⚠️ 「静止していたら位置を比べる」では判定できない。MOLA 自身の推定が
    # 立っているだけで 0.3 m ふらつくので（2026-09-08 実測）、静止の判定が通らない。
    # **投げた方向（真横）への残差**を見る。歩行も自然なふらつきも進行方向や
    # 等方な揺れなので、横 1 m の投げ込みとは分離できる。
    ux = math.cos(yaw + math.pi / 2)
    uy = math.sin(yaw + math.pi / 2)
    fx, fy = after_p[-1][1], after_p[-1][2]
    along = (fx - x) * ux + (fy - y) * uy          # 投げた方向の残差[m]
    lateral = math.dist((x, y), (fx, fy))
    print()
    print("== 投げた場所に居座っていないか（品質だけでは「自信を持って間違える」を落とせない）==")
    print(f"  投げる前 ({x:+.3f}, {y:+.3f}) → 最後 ({fx:+.3f}, {fy:+.3f})  距離 {lateral:.3f} m")
    print(f"  **投げた方向への残差: {along:+.3f} m**（投げた量は {args.offset_m:+.3f} m。"
          f"0 に近ければ ICP が引き戻した／{args.offset_m} m に近ければ居座っている）")

    print()
    print("== 合否（docs/plan/2026-09-08-global-localization-and-move.md §4 段 C）==")
    accepted = biggest[1] >= JUMP_EVIDENCE_M or dip < min(base_q)
    print(f"  [{'PASS' if accepted else 'FAIL'}] 投げた姿勢が**受理された証拠**がある"
          f"（飛び ≥ {JUMP_EVIDENCE_M} m または平常最小を下回る落ち込み）")
    ok_rec = recovered_at is not None and recovered_at <= RECOVER_SEC
    got = f"{recovered_at:.2f} 秒" if recovered_at is not None else "戻らない"
    print(f"  [{'PASS' if ok_rec else 'FAIL'}] {RECOVER_SEC:.0f} 秒以内に品質 "
          f"{QUALITY_RECOVER} 以上へ復帰  実測 {got}")
    if not accepted:
        print("  ⚠️ 受理の証拠が無い場合、「復帰した」は**何もしていない**のと区別できない。")
    ok_back = abs(along) <= 0.30
    print(f"  [{'PASS' if ok_back else 'FAIL'}] 投げた場所に居座っていない"
          f"（投げた方向の残差 ≤ 0.30 m）  実測 {along:+.3f} m / 投げた量 {args.offset_m:.2f} m")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if (accepted and ok_rec and ok_back) else 1


if __name__ == "__main__":
    sys.exit(main())
