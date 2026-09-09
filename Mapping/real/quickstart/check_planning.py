#!/usr/bin/env python3
"""Nav2 が本当に経路を引けるかを数字で出す。**再生でも実機でも同じ基準で測る。**

    docker exec -u ubuntu -e ... rviz bash -c \
      "source /opt/ros/humble/setup.bash && \
       python3 /work/G1_Hackason/Mapping/real/quickstart/check_planning.py --tries 10"

## 何を測るか（docs/plan/2026-09-08-global-localization-and-move.md §4 段 B）

| 量 | 合格 |
|---|---|
| 3〜6 m 先の到達可能なゴールへ経路が引けるか | `SUCCEEDED` |
| 機体セルのコスト | **inscribed(99) でない**。床由来の致死セルが機体の周りに出ない |

## ⚠️ **1 回ごとに姿勢を読み直す。まとめて測ってはいけない**（2026-09-08 に踏んだ）

記録 20260906T135940_UiS_room_v3 には
**「開けた場所で静止している区間」が存在しない。**
歩行中（153 m を 591 秒で移動）か、`t+418s` 以降の**散らかった開始点で静止**の
どちらかしかない（開始点は静的地図で ±2.5 m の 42% が占有）。
最初は「機体の姿勢を 1 度読んでゴールを 20 回投げる」作りにしたので、
測っている間に機体が数 m 歩いてしまい、**毎回別の場所を測っていた**。
実機でも歩きながら投げるので、**姿勢とコストマップとゴールは 1 回ごとに揃える。**

## ⚠️ ほかに測り方で間違えたこと（2026-09-07）

- **`nav2_[a-z]` のような pkill で map_server を巻き添えにした。** 静的レイヤが空だと
  「経路が引けない」に見えるが原因は別。**先に `/map` の大きさを印字する**
- **8 連結の BFS で 4 連結の NavFn を評価して誤結論を出した。** 到達可能性を自前で
  判定しないこと。**planner に投げた結果だけを根拠にする**のがこのスクリプトの方針
"""
from __future__ import annotations

import argparse
import math
import statistics
from collections import deque
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

# コストマップの値（OccupancyGrid に載るとき 100=致死 / 99=内接）
LETHAL = 100
INSCRIBED = 99

GOAL_DISTANCES_M = (3.0, 4.0, 5.0, 6.0)
GOAL_BEARINGS_DEG = (0.0, 20.0, -20.0, 45.0, -45.0, 90.0, -90.0)


def _map_qos() -> QoSProfile:
    """Nav2 のコストマップと map_server は transient_local で出す。"""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class Planning(Node):
    def __init__(self, sim_time: bool) -> None:
        super().__init__(
            "check_planning",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, sim_time)],
        )
        self.grids: dict[str, OccupancyGrid] = {}
        for topic in ("/map", "/global_costmap/costmap", "/local_costmap/costmap"):
            self.create_subscription(
                OccupancyGrid, topic,
                lambda msg, t=topic: self.grids.__setitem__(t, msg), _map_qos())
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

    def spin_for(self, sec: float) -> None:
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def robot_pose(self) -> tuple[float, float, float] | None:
        """map -> base_link を最新の TF から取る（Time() = 最新）。"""
        try:
            tr = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception as exc:      # tf2 の例外は種類が多いのでまとめて拾う
            self.get_logger().warn(f"map -> base_link が引けない: {exc}")
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)


def cell_at(grid: OccupancyGrid | None, x: float, y: float) -> int | None:
    """map 系の (x, y) にあたるセルの値。範囲外・未受信は None。"""
    if grid is None:
        return None
    info = grid.info
    col = int((x - info.origin.position.x) / info.resolution)
    row = int((y - info.origin.position.y) / info.resolution)
    if not (0 <= col < info.width and 0 <= row < info.height):
        return None
    return grid.data[row * info.width + col]


def window_stats(grid: OccupancyGrid | None, x: float, y: float, radius_m: float) -> dict:
    """機体の周り radius_m 四方のセルを数える。"""
    empty = {"lethal": 0, "inscribed": 0, "unknown": 0, "free": 0, "total": 0, "worst": -1}
    if grid is None:
        return empty
    info = grid.info
    r = max(1, int(radius_m / info.resolution))
    c0 = int((x - info.origin.position.x) / info.resolution)
    r0 = int((y - info.origin.position.y) / info.resolution)
    counts = dict(empty)
    for rr in range(max(0, r0 - r), min(info.height, r0 + r + 1)):
        base = rr * info.width
        for cc in range(max(0, c0 - r), min(info.width, c0 + r + 1)):
            v = grid.data[base + cc]
            counts["total"] += 1
            if v < 0:
                counts["unknown"] += 1
            elif v >= LETHAL:
                counts["lethal"] += 1
            elif v >= INSCRIBED:
                counts["inscribed"] += 1
            elif v == 0:
                counts["free"] += 1
            counts["worst"] = max(counts["worst"], v)
    return counts


# 診断用の探索範囲。ゴールは 3〜6 m なので、20 m 四方あれば「回り込み」も入る
REACH_WINDOW_CELLS = 200


def reach_4conn(grid: OccupancyGrid | None, rx: float, ry: float,
                gx: float, gy: float) -> "tuple[bool | None, int]":
    """機体からゴールへ **4 連結** で辿り着けるかを見る。**診断であって合否ではない。**

    ⚠️ 2026-09-07 に、**8 連結の BFS で 4 連結の NavFn を評価して誤結論**を出した。
    合否は planner に投げた結果だけで決める（このスクリプトの方針）。ここは
    「失敗したのは壁の向こうを選んだからか、それとも planner 側か」を切り分けるだけ。

    `pick_goal` は「そのセルが致死でない」ゴールを選ぶだけで到達可能性は見ない。
    だから**壁の向こうの自由セル**が選ばれうる。そのときの失敗は地図の問題ではない。
    """
    if grid is None:
        return (None, 0)
    info = grid.info
    to_cell = lambda x, y: (int((x - info.origin.position.x) / info.resolution),
                            int((y - info.origin.position.y) / info.resolution))
    c0, r0 = to_cell(rx, ry)
    c1, r1 = to_cell(gx, gy)
    for c, r in ((c0, r0), (c1, r1)):
        if not (0 <= c < info.width and 0 <= r < info.height):
            return ("範囲外", 0)

    def open_cell(c: int, r: int) -> bool:
        v = grid.data[r * info.width + c]
        return v < INSCRIBED          # 未知(-1) も通れる扱い（track_unknown_space 既定）

    # ⚠️ **機体自身のセルが塞がっていると BFS は 1 セルも広がらない。**
    # これを「ゴールが壁の向こう」と読むと原因を取り違える（2026-09-08 に一度やった）。
    # この場合の失敗は §4 段 B の 2 つ目（機体セルが inscribed でない）と同じ話である。
    if not open_cell(c0, r0):
        return ("機体が囲まれている", 0)
    if not open_cell(c1, r1):
        return ("ゴールのセルが塞がっている", 0)

    lo_c, hi_c = max(0, c0 - REACH_WINDOW_CELLS), min(info.width, c0 + REACH_WINDOW_CELLS)
    lo_r, hi_r = max(0, r0 - REACH_WINDOW_CELLS), min(info.height, r0 + REACH_WINDOW_CELLS)
    seen = {(c0, r0)}
    queue = deque([(c0, r0)])
    while queue:
        c, r = queue.popleft()
        if (c, r) == (c1, r1):
            return ("同じ自由領域", len(seen))
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc, nr = c + dc, r + dr
            if lo_c <= nc < hi_c and lo_r <= nr < hi_r and (nc, nr) not in seen:
                if open_cell(nc, nr):
                    seen.add((nc, nr))
                    queue.append((nc, nr))
    return ("ゴールは壁の向こう", len(seen))


def _grid_dict(grid: "OccupancyGrid | None") -> dict:
    """OccupancyGrid を保存できる形にする。"""
    if grid is None:
        return {}
    i = grid.info
    return {
        "width": i.width, "height": i.height, "resolution": i.resolution,
        "origin": [i.origin.position.x, i.origin.position.y],
        "data": list(grid.data),
    }


def _frame(k: int, rx: float, ry: float, ryaw: float,
           vs: "int | None", vg: "int | None", vl: "int | None", w: dict,
           g: "OccupancyGrid | None", goal: "tuple | None",
           ok: bool, why: str, path_xy: list) -> dict:
    """--record 用に 1 回ぶんをまとめる。**測定には一切影響しない。**"""
    return {
        "try": k + 1,
        "robot": [rx, ry, ryaw],
        "cost_static": vs, "cost_global": vg, "cost_local": vl,
        "window": w,
        "goal": list(goal) if goal else None,
        "ok": bool(ok), "why": why,
        "path": [[x, y] for x, y in path_xy],
        "global_costmap": _grid_dict(g),
    }


def save_record(record_dir: Path, frames: list, static: "OccupancyGrid",
                meta: dict) -> None:
    """記録を JSON で書く。コストマップは嵩むので npz に分ける。"""
    import json

    import numpy as np

    record_dir.mkdir(parents=True, exist_ok=True)
    arrays = {}
    for f in frames:
        g = f.pop("global_costmap")
        if g:
            arrays[f"costmap_{f['try']:02d}"] = np.asarray(g.pop("data"), dtype=np.int16)
            f["global_costmap_info"] = g
    si = static.info
    arrays["static"] = np.asarray(static.data, dtype=np.int16)
    meta = dict(meta)
    meta["static_info"] = {
        "width": si.width, "height": si.height, "resolution": si.resolution,
        "origin": [si.origin.position.x, si.origin.position.y],
    }
    meta["frames"] = frames
    (record_dir / "planning.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=1))
    np.savez_compressed(record_dir / "planning_grids.npz", **arrays)
    print(f"[record] -> {record_dir}/planning.json + planning_grids.npz "
          f"（{len(frames)} 回）")


def pick_goal(node: Planning, rx: float, ry: float, ryaw: float):
    """機体の前方 3〜6 m で、global costmap 上で到達可能そうなゴールを 1 つ選ぶ。"""
    g = node.grids.get("/global_costmap/costmap")
    for dist in GOAL_DISTANCES_M:
        for bearing in GOAL_BEARINGS_DEG:
            th = ryaw + math.radians(bearing)
            gx, gy = rx + dist * math.cos(th), ry + dist * math.sin(th)
            v = cell_at(g, gx, gy)
            if v is not None and 0 <= v < INSCRIBED:
                return (gx, gy, dist, bearing, v)
    return None


def plan_once(node: Planning, gx: float, gy: float,
              planner_id: str) -> "tuple[bool, str, list[tuple[float, float]]]":
    """経路を 1 回引く。**引けた経路の点列も返す**（--record で図にするため）。"""
    goal = ComputePathToPose.Goal()
    goal.goal = PoseStamped()
    goal.goal.header.frame_id = "map"
    goal.goal.header.stamp = node.get_clock().now().to_msg()
    goal.goal.pose.position.x = gx
    goal.goal.pose.position.y = gy
    goal.goal.pose.orientation.w = 1.0
    goal.use_start = False
    goal.planner_id = planner_id

    send = node.planner.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    if not send.done() or send.result() is None:
        return (False, "送信が返ってこない", [])
    handle = send.result()
    if not handle.accepted:
        return (False, "planner がゴールを受け付けない", [])
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=15.0)
    if not res.done() or res.result() is None:
        return (False, "結果が返ってこない", [])
    poses = res.result().result.path.poses
    if not poses:
        return (False, "経路が空（failed to create plan）", [])
    length = sum(
        math.dist((poses[i].pose.position.x, poses[i].pose.position.y),
                  (poses[i + 1].pose.position.x, poses[i + 1].pose.position.y))
        for i in range(len(poses) - 1)
    )
    xy = [(pp.pose.position.x, pp.pose.position.y) for pp in poses]
    return (True, f"{len(poses)} 点 / 経路長 {length:.2f} m", xy)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--tries", type=int, default=10,
                   help="何回投げるか。1 回の成否で決めないため（実測で 5 回中 1〜4 回しか通らない日があった）")
    p.add_argument("--planner", default="GridBased",
                   help="planner_id。既定の GridBased(NavFn) が実測で最良（Smac2D は常用しない）")
    p.add_argument("--no-sim-time", action="store_true", help="実機で使うとき")
    p.add_argument("--record", metavar="DIR",
                   help="各回の姿勢・コストマップ・経路を保存する（動画にするため）。"
                        "測定そのものは変えない")
    p.add_argument("--robot-radius", type=float, default=0.30,
                   help="g1_nav2.yaml と揃えること。機体周りの窓の大きさに使う")
    p.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"), default=None,
                   help="**map 系の絶対座標のゴール 1 点**をそこへ投げる。"
                        "既定は pick_goal が前方 3〜6m から自分で選ぶが、"
                        "**歩かせる予定のゴールを歩く前に検算したいとき**はこちらを使う"
                        "（1 点に固定されるので --tries は同じゴールの繰り返しになる）")
    args = p.parse_args(argv)

    rclpy.init()
    node = Planning(sim_time=not args.no_sim_time)
    print("[check_planning] トピックと TF が来るのを待つ…")
    node.spin_for(8.0)

    static = node.grids.get("/map")
    if static is None:
        print("  [FAIL] /map が来ていない。map_server が落ちている可能性")
        node.destroy_node(); rclpy.shutdown()
        return 1
    i = static.info
    print(f"  /map: {i.width}x{i.height} @ {i.resolution:.3f} m "
          f"origin=({i.origin.position.x:.2f}, {i.origin.position.y:.2f})")
    if "/global_costmap/costmap" not in node.grids:
        print("  [FAIL] /global_costmap/costmap が来ていない")
        node.destroy_node(); rclpy.shutdown()
        return 1
    if not node.planner.wait_for_server(timeout_sec=10.0):
        print("  [FAIL] compute_path_to_pose のサーバが居ない")
        node.destroy_node(); rclpy.shutdown()
        return 1

    print()
    print(f"== 経路計画（planner_id={args.planner} / {args.tries} 回 / 毎回姿勢を読み直す）==")
    print("   凡例: 機体セル = static/global/local。static は事前地図、")
    print("         global は静的＋ライブ＋膨張、local はライブ＋膨張のみ")

    record_dir = Path(args.record) if args.record else None
    frames: list[dict] = []

    results: list[bool] = []
    robot_cells_global: list[int] = []
    boxed_in = 0
    causes: dict[str, int] = {}
    no_goal = 0
    for k in range(args.tries):
        node.spin_for(0.6)                      # 最新のコストマップと TF を取り込む
        pose = node.robot_pose()
        if pose is None:
            print(f"  {k + 1:2d}: TF が引けない")
            results.append(False)
            continue
        rx, ry, ryaw = pose
        g = node.grids.get("/global_costmap/costmap")
        vs = cell_at(static, rx, ry)
        vg = cell_at(g, rx, ry)
        vl = cell_at(node.grids.get("/local_costmap/costmap"), rx, ry)
        w = window_stats(g, rx, ry, args.robot_radius * 2)
        if vg is not None and vg >= INSCRIBED:
            boxed_in += 1
        if vg is not None:
            robot_cells_global.append(vg)

        if args.goal is not None:
            # 指定された 1 点。**到達可能かは見ない**（それが plan_once の仕事）
            gx0, gy0 = args.goal
            picked = (gx0, gy0, math.hypot(gx0 - rx, gy0 - ry),
                      math.degrees(math.atan2(gy0 - ry, gx0 - rx) - ryaw),
                      cell_at(g, gx0, gy0))
        else:
            picked = pick_goal(node, rx, ry, ryaw)
        if picked is None:
            no_goal += 1
            print(f"  {k + 1:2d}: ({rx:+6.2f},{ry:+6.2f}) 機体セル {vs}/{vg}/{vl} "
                  f"周り 致死{w['lethal']} 内接{w['inscribed']} → **到達可能なゴールが無い**")
            results.append(False)
            if record_dir is not None:
                frames.append(_frame(k, rx, ry, ryaw, vs, vg, vl, w, g,
                                     None, False, "到達可能なゴールが無い", []))
            continue
        gx, gy, dist, bearing, gv = picked
        ok, why, path_xy = plan_once(node, gx, gy, args.planner)
        results.append(ok)
        note = ""
        if not ok:
            verdict, seen = reach_4conn(g, rx, ry, gx, gy)
            note = f" [診断] {verdict}"
            if seen:
                note += f"（{seen:,}セル探索）"
            if verdict == "同じ自由領域":
                note += " → **planner 側の問題**"
            causes[verdict] = causes.get(verdict, 0) + 1
        print(f"  {k + 1:2d}: ({rx:+6.2f},{ry:+6.2f}) 機体セル {vs}/{vg}/{vl} "
              f"周り 致死{w['lethal']} 内接{w['inscribed']} → "
              f"ゴール ({gx:+6.2f},{gy:+6.2f}) {dist:.0f}m/{bearing:+.0f}deg(cost {gv}) "
              f"{'成功' if ok else '失敗'} {why}{note}")
        if record_dir is not None:
            frames.append(_frame(k, rx, ry, ryaw, vs, vg, vl, w, g,
                                 (gx, gy, dist, bearing, gv), ok, why + note, path_xy))

    n_ok = sum(results)
    print()
    n_fail = len(results) - n_ok
    if n_fail:
        print(f"== 失敗 {n_fail} 件の内訳（**診断**。合否には使わない）==")
        for label, n in sorted(causes.items(), key=lambda kv: -kv[1]):
            mark = "  ← ここが本当の問題" if label == "同じ自由領域" else ""
            print(f"  {label:26s}: {n}{mark}")
        if no_goal:
            print(f"  {'到達可能なゴールが無い':26s}: {no_goal}")
        print("  ⚠️ pick_goal は「そのセルが致死でない」ゴールを選ぶだけで、"
              "到達可能かは見ていない。「ゴールは壁の向こう」は測り方の副作用で、地図の問題ではない")
        print()
    print("== 合否（docs/plan/2026-09-08-global-localization-and-move.md §4 段 B）==")
    ok_plan = n_ok > 0
    print(f"  [{'PASS' if ok_plan else 'FAIL'}] 3〜6 m 先の到達可能なゴールへ経路が引ける  "
          f"実測 {n_ok}/{len(results)} 回成功")
    ok_cell = boxed_in == 0
    med = statistics.median(robot_cells_global) if robot_cells_global else -1
    print(f"  [{'PASS' if ok_cell else 'FAIL'}] 機体セルが inscribed(99) 未満  "
          f"実測 {len(results) - boxed_in}/{len(results)} 回（コストの中央値 {med}）")
    if no_goal:
        print(f"  参考: 到達可能なゴールが 1 つも取れなかった回が {no_goal}/{len(results)}")

    if record_dir is not None:
        save_record(record_dir, frames, static, {
            "planner": args.planner,
            "tries": args.tries,
            "robot_radius": args.robot_radius,
            "n_ok": n_ok,
            "boxed_in": boxed_in,
            "median_robot_cell": med,
            "pass_plan": ok_plan,
            "pass_cell": ok_cell,
        })

    node.destroy_node()
    rclpy.shutdown()
    return 0 if (ok_plan and ok_cell) else 1


if __name__ == "__main__":
    sys.exit(main())
