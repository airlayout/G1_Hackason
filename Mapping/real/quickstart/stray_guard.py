#!/usr/bin/env python3
"""ゴールを投げた場所からの逸脱と、経過時間を見張って Nav2 をキャンセルする。

## なぜ要るのか

`check_navigation.py` にも `--max-stray` があるが、**あれは自分が投げたゴールしか守れない。**
RViz2 の「2D Goal Pose」からクリックすると保護が無くなる。しかも RViz のクリックには
**timeout が無い**ので、BT が 6 回リトライしながら Spin を繰り返すと数分動き続ける。

2026-09-09 に実機で踏んだ形（詳細は README-nav2 §7）:
**1.79m のゴールに対して累積 5.59m 動き 174° 回った。**ゴールの向きの指定が
機体と無関係な固定値だったため、位置は達成しても yaw 許容を満たせず、
その場旋回 → 進捗判定が並進しか数えないので失敗 → 復帰動作の Spin 90°。
**転倒もエラーも出ないので「なぜか着かない」としか見えない。**

このノードは**投げ方に関係なく**効く。常駐させておく。

## 何を見るか

**距離と時間（もとから）**

- `/goal_pose` を受けた瞬間の `map -> base_link` を「開始点」として覚える
- 開始点からの距離が **ゴールまでの直線 + `--margin`** を超えたらキャンセル
- `--max-abs` を超えたら（ゴールに関係なく）キャンセル
- ゴールを受けてから `--max-seconds` 経ったらキャンセル
- `/navigate_to_pose/_action/status` でゴールの終了を見て待機に戻る

**門番 G0 — 見かけの速さ（2026-09-13 に追加）**

- `map -> base_link` の連続 2 サンプルから **|Δxy|/Δt** を出し、
  **窓 `--g0-window` 秒の中央値**が **`上限 × --g0-factor`** を超えたらキャンセル

**`/tf` の途絶（同日に追加）**

- 見張り中に `map -> base_link` の**スタンプが `--max-tf-gap` 秒進まなければ**キャンセル

## ⚠️ 距離の首輪だけでは足りない（なぜ G0 を足したか）

`--max-abs` は既定 3.0 m で**頭打ち**になる。ゴールまで 4.35 m のクリックでは
**到達する前に必ず鳴る**ので、クリック運用では大きく取るしかない（計画は 15.0 m）。
ところが大きく取ると**本物の逸脱も見逃す**。

G0 は「推定姿勢が**機体に出せない速さで動いている**」を見るので、
**ゴールの遠近に依らない**。2026-09-10 の実機 3 本はどれも
「変位が幻」（その場旋回しか指令していないのに推定が 1.98 m 動く／見かけ最大 3.71 m/s）
で、これは距離の首輪では**本物の逸脱と区別できない**。

### しきい値の根拠（2026-09-13 に 2 日ぶんの静止で測り直した）

`eval_guard_speed.py` に **歩行 3 本**（`stage_20260910T182639_r{1,2,3}`）と
**静止 7 本**（09-10 の 1 本 ＋ 09-12 の 6 本）を食わせた結果:

| 係数 | 窓 | しきい | 静止に対する余裕 | 歩行が鳴るまで |
|---|---|---|---|---|
| 1.5 | 1.0 s | 0.54 m/s | **2.0 倍** | 0.7 / 1.0 / 1.8 s ⇒ **余裕不足で不採用** |
| **2.0** | **1.0 s** | **0.72 m/s** | **2.6 倍** | **1.4 / 2.2 / 2.3 s ⇒ 採用** |
| 3.0 | 1.0 s | 1.08 m/s | 4.0 倍 | 13.5 / 6.9 / 12.3 s（遅い）|

⚠️ **1 日ぶんの静止で決めてはいけない。**09-11 に 09-10 の静止だけで測ったときは
×1.5 が「採用」と出たが、別の日の静止を入れると余裕が 2.0 倍に落ちて不合格になる。
静止中の推定の震えは日によって 3 割動く。

⚠️ **窓は評価器の最短（0.5 s）ではなく 1.0 s を既定にしてある。**
0.5 s だと `/tf` が 10 Hz から落ちた瞬間に窓の中のサンプルが 3 本を切り、
**いちばん怪しいときに黙る**。1.0 s なら 3 Hz まで落ちても 3 本残る
（余裕も 2.2 → 2.6 倍に増える。代償は検知が 1.8 → 2.3 秒に延びること）。

## 使い方

    # 記録の再生（sim 時刻）
    python3 stray_guard.py --margin 0.5 --max-abs 3.0 --max-seconds 60

    # 実機の RViz2 クリック運用（**--no-sim-time が要る**）
    python3 stray_guard.py --no-sim-time --max-abs 15.0 --max-seconds 120

⚠️ **キャンセルは全ゴールに効く**（`cancel_nav.py` と同じ全ゼロ要求）。
複数人が同じ Nav2 にゴールを投げる運用では使わないこと。

⚠️ Nav2 と同じ DDS 設定で起こすこと。
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
from collections import deque
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

CANCEL_SERVICE = "/navigate_to_pose/_action/cancel_goal"
# 門番 G0 の既定。根拠は上の表（2026-09-13 に 2 日ぶんの静止で導出）
G0_FACTOR = 2.0
G0_WINDOW_S = 1.0
G0_MIN_SAMPLES = 3          # 窓の中にこれだけ無いと判定しない（評価器と同じ）
# yaml を読めなかったときだけ使う並進の上限[m/s]。**黙って 0 にしない**
FALLBACK_MAX_VEL = 0.30
NAV2_YAML_DEFAULT = "/work/G1_Hackason/Navigation/nav2/g1_nav2.yaml"
STATUS_TOPIC = "/navigate_to_pose/_action/status"
# ゴールが「まだ動いている」とみなす状態。これ以外になったら待機に戻る
ACTIVE = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
          GoalStatus.STATUS_CANCELING}


def read_max_vel(yaml_path: "Path | None") -> float:
    """機体に届く並進の上限[m/s]を Nav2 の yaml から読む。

    見るのは `velocity_smoother` の `max_velocity: [vx, vy, vyaw]`。
    **これが /cmd_vel に出る最後の関門**で、RPP の `desired_linear_vel` より硬い。
    速さは vx と vy の合成（横歩きも並進なので勝手に落とさない）。
    ⚠️ `eval_guard_speed.py` と**同じ読み方**にしてある。片方だけ変えないこと。
    """
    if yaml_path is None or not yaml_path.exists():
        print("⚠️ nav2 の yaml を読めないので上限 {} m/s を使う（{}）".format(
            FALLBACK_MAX_VEL, yaml_path))
        return FALLBACK_MAX_VEL
    for line in yaml_path.read_text().splitlines():
        head = line.split("#", 1)[0]
        if "max_velocity:" in head and "[" in head:
            nums = head.split("[", 1)[1].split("]", 1)[0].split(",")
            return math.hypot(float(nums[0]), float(nums[1]))
    print("⚠️ {} に max_velocity が無いので上限 {} m/s を使う".format(
        yaml_path, FALLBACK_MAX_VEL))
    return FALLBACK_MAX_VEL


class StrayGuard(Node):
    def __init__(self, args) -> None:
        super().__init__("stray_guard", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, not args.no_sim_time)])
        self.a = args
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.cli = self.create_client(CancelGoal, CANCEL_SERVICE)
        self.watch: dict | None = None       # 見張り中の情報。None なら待機
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.create_subscription(GoalStatusArray, STATUS_TOPIC, self._on_status, 10)
        self.create_timer(1.0 / max(args.rate, 0.1), self._tick)
        # ── 門番 G0（見かけの速さ）の状態 ──────────────────────────
        self.limit = read_max_vel(Path(args.nav2_yaml) if args.nav2_yaml else None)
        self.g0_threshold = self.limit * args.g0_factor
        self.last_stamp: float | None = None     # 直近に取り込んだ TF のスタンプ[s]
        self.last_xy: tuple[float, float] | None = None
        self.speeds: deque = deque()             # (区間の中央時刻, 見かけの速さ)
        self.g0_warned = -1e9                    # 待機中の警告を間引く
        self.get_logger().info(
            "[guard] 待機。余裕 {:.2f}m / 絶対上限 {:.2f}m / 時間上限 {:.0f}s".format(
                args.margin, args.max_abs, args.max_seconds))
        if args.no_g0:
            self.get_logger().warn("[guard] **門番 G0 は切ってある**（--no-g0）")
        else:
            self.get_logger().info(
                "[guard] 門番 G0: 上限 {:.3f} m/s x {:.1f} = **{:.2f} m/s** を "
                "窓 {:.1f}s の中央値で見る".format(
                    self.limit, args.g0_factor, self.g0_threshold, args.g0_window))
        self.get_logger().info(
            "[guard] /tf の途絶: 見張り中に {:.1f}s 進まなければ鳴る".format(args.max_tf_gap))

    # ── 位置 ─────────────────────────────────────────────────────────
    def _xy(self) -> tuple[float, float] | None:
        got = self._xy_stamped()
        return None if got is None else got[1]

    def _xy_stamped(self) -> "tuple[float, tuple[float, float]] | None":
        """`map -> base_link` を **TF 自身のスタンプつき**で返す。

        ⚠️ スタンプを使うのは、**同じ姿勢を二度数えないため**である。
        見かけの速さは |Δxy|/Δt なので、TF が更新されていないのに
        タイマの周期で割ると**速さが 0 に薄まり、G0 が黙る**。
        """
        try:
            tr = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        return (st, (tr.transform.translation.x, tr.transform.translation.y))

    # ── ゴールを受けた ───────────────────────────────────────────────
    def _on_goal(self, msg: PoseStamped) -> None:
        start = self._xy()
        if start is None:
            self.get_logger().error(
                "[guard] map -> base_link が引けない。**見張れないので投げ直すこと**")
            return
        gx, gy = msg.pose.position.x, msg.pose.position.y
        dist = math.hypot(gx - start[0], gy - start[1])
        leash = min(dist + self.a.margin, self.a.max_abs)
        self.watch = {"start": start, "goal": (gx, gy), "dist": dist,
                      "leash": leash, "t0": self.get_clock().now(), "fired": False}
        self.get_logger().info(
            "[guard] 見張り開始 開始点({:+.2f}, {:+.2f}) → ゴール({:+.2f}, {:+.2f}) "
            "直線 {:.2f}m / 上限 {:.2f}m / {:.0f}s".format(
                *start, gx, gy, dist, leash, self.a.max_seconds))

    # ── ゴールが終わった ─────────────────────────────────────────────
    def _on_status(self, msg: GoalStatusArray) -> None:
        if self.watch is None:
            return
        if any(s.status in ACTIVE for s in msg.status_list):
            return
        self.get_logger().info("[guard] ゴールが終了した。待機に戻る")
        self.watch = None

    # ── 門番 G0（見かけの速さ）──────────────────────────────────────
    def _sample_speed(self) -> None:
        """TF が進んでいたら 1 サンプル取り込み、窓からはみ出た分を捨てる。

        ⚠️ **評価器 `eval_guard_speed.py` と同じ作り**にしてある
        （連続 2 点の |Δxy|/Δt・窓の中央値・最低 3 本）。
        片方だけ変えると、しきい値の根拠が実測と結びつかなくなる。
        """
        got = self._xy_stamped()
        if got is None:
            return
        stamp, xy = got
        if self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if dt <= 1e-6:                 # 同じ姿勢。0 割りを作らない
                return
            v = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1]) / dt
            self.speeds.append((stamp - dt / 2.0, v))
        self.last_stamp, self.last_xy = stamp, xy
        lo = stamp - self.a.g0_window
        while self.speeds and self.speeds[0][0] < lo:
            self.speeds.popleft()

    def _g0_median(self) -> "float | None":
        """窓の中央値。**サンプルが足りなければ判定しない**（None を返す）。"""
        if len(self.speeds) < G0_MIN_SAMPLES:
            return None
        return statistics.median(v for _, v in self.speeds)

    # ── 監視 ─────────────────────────────────────────────────────────
    def _tick(self) -> None:
        if not self.a.no_g0:
            self._sample_speed()
        w = self.watch
        if w is None or w["fired"]:
            # 待機中でも G0 は見る。止める相手が居ないので**警告だけ**出す
            # （これが出るのは「ゴールを投げる前から推定が壊れている」形）
            if w is None and not self.a.no_g0:
                med = self._g0_median()
                now_s = self.get_clock().now().nanoseconds * 1e-9
                if med is not None and med > self.g0_threshold and \
                        now_s - self.g0_warned > 10.0:
                    self.g0_warned = now_s
                    self.get_logger().warn(
                        "[guard] ⚠️ 待機中だが見かけの速さ {:.2f} m/s > {:.2f} m/s。"
                        "**このまま投げないこと**".format(med, self.g0_threshold))
            return
        now = self._xy()
        if now is None:
            return
        d = math.hypot(now[0] - w["start"][0], now[1] - w["start"][1])
        elapsed = (self.get_clock().now() - w["t0"]).nanoseconds * 1e-9
        why = None
        med = None if self.a.no_g0 else self._g0_median()
        if med is not None and med > self.g0_threshold:
            # ⚠️ **距離より先に見る。**距離の首輪は「幻の変位」でも鳴るので、
            # 先に鳴らせてしまうと理由が「逸脱」に化けて切り分けを誤らせる
            why = ("見かけの速さ {:.2f} m/s（上限 {:.2f} m/s）。"
                   "**推定が機体に出せない速さで動いている＝測位の破綻**".format(
                       med, self.g0_threshold))
        elif self.last_stamp is not None and \
                (stall := self._tf_stall(w)) is not None:
            why = stall
        elif d > w["leash"]:
            why = "開始点から {:.2f}m（上限 {:.2f}m）".format(d, w["leash"])
        elif elapsed > self.a.max_seconds:
            why = "{:.0f}s 経過（上限 {:.0f}s）".format(elapsed, self.a.max_seconds)
        if why is None:
            return
        w["fired"] = True
        self.get_logger().error("[guard] **作動: " + why + "。キャンセルする**")
        self._cancel()

    def _tf_stall(self, w: dict) -> "str | None":
        """`map -> base_link` のスタンプが進まなくなっていないか。

        09-12 の実機では測位が崩れた最後に **`/tf` ごと止まった**（3.34 Hz -> 0）。
        止まると見かけの速さも距離も更新されないので、**G0 も首輪も黙る**。
        ここだけは「来ないこと」を鳴らす。

        ⚠️ **これが効くのは `--no-sim-time`（＝ 実機）だけ**である。
        記録の再生では `/clock` も一緒に止まるので、`now` が進まず gap も育たない。
        再生で「止まったのに鳴らない」のは仕様であって不具合ではない。
        """
        now_s = self.get_clock().now().nanoseconds * 1e-9
        gap = now_s - self.last_stamp
        if gap <= self.a.max_tf_gap:
            return None
        return ("map -> base_link が {:.1f}s 進んでいない（上限 {:.1f}s）。"
                "**測位が止まっている**".format(gap, self.a.max_tf_gap))

    def _cancel(self) -> None:
        if not self.cli.service_is_ready():
            # ここで待つと監視が止まるので待たない。次の tick で再試行される
            self.get_logger().error(
                "[guard] {} が居ない。**手で止めること**".format(CANCEL_SERVICE))
            if self.watch:
                self.watch["fired"] = False
            return
        fut = self.cli.call_async(CancelGoal.Request())   # 全ゼロ = 全ゴール
        fut.add_done_callback(self._cancelled)

    def _cancelled(self, fut) -> None:
        res = fut.result()
        if res is None:
            self.get_logger().error("[guard] キャンセルの応答が無い。**手で止めること**")
            return
        self.get_logger().error(
            "[guard] キャンセル完了 return_code={} / 対象 {} 件".format(
                res.return_code, len(res.goals_canceling)))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--margin", type=float, default=0.5,
                   help="ゴールまでの直線に足す余裕[m]。復帰動作の分だけ見ておく")
    p.add_argument("--max-abs", type=float, default=3.0,
                   help="開始点からの絶対上限[m]。遠いゴールでもここで必ず止める")
    p.add_argument("--max-seconds", type=float, default=60.0,
                   help="ゴールを受けてからの時間上限[s]。RViz のクリックには timeout が無い")
    p.add_argument("--rate", type=float, default=20.0,
                   help="監視の周期[Hz]。**TF の更新率より高くしておく**"
                        "（低いと G0 がサンプルを取りこぼす。TF 自身のスタンプで"
                        "重複は捨てるので、上げても二度数えにはならない）")
    p.add_argument("--g0-factor", type=float, default=G0_FACTOR,
                   help="門番 G0 のしきい値 = 並進の上限 x これ。"
                        "**2 日ぶんの静止で 2.0 を導出した（余裕 2.6 倍）**。"
                        "1.5 は静止との余裕が 2.0 倍しかなく不合格")
    p.add_argument("--g0-window", type=float, default=G0_WINDOW_S,
                   help="門番 G0 の窓[s]。中央値を取る幅。"
                        "0.5 にすると /tf が落ちたとき窓の中が 3 本を切って黙る")
    p.add_argument("--no-g0", action="store_true",
                   help="門番 G0 を切る。**切る理由を記録に残すこと**")
    p.add_argument("--max-tf-gap", type=float, default=3.0,
                   help="見張り中に map -> base_link のスタンプが"
                        "これだけ進まなければキャンセル[s]。"
                        "09-12 の崩壊は最後に /tf ごと止まった")
    p.add_argument("--nav2-yaml", default=NAV2_YAML_DEFAULT,
                   help="並進の上限（velocity_smoother の max_velocity）の出どころ")
    p.add_argument("--no-sim-time", action="store_true",
                   help="実機で使うときに付ける（/clock が無いので必須）")
    args = p.parse_args(argv)

    rclpy.init()
    node = StrayGuard(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ⚠️ SIGTERM は ExternalShutdownException で来る（KeyboardInterrupt は SIGINT だけ）。
        # 拾わないと停止のたびに traceback が出て、本物のエラーと見分けが付かなくなる
        node.get_logger().info("[guard] 停止します")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
