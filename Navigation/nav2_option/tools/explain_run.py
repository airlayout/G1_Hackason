#!/usr/bin/env python3
"""記録した rosbag から「何が起きたか」を時系列に要約する。

Planning.md Phase 2c の完了条件:

    > rosbag から Goal・経路・姿勢・速度指令・**停止理由**を追跡できる

この条件を「人が手で掘れば分かる」で満たしたことにすると、実機試験の現場で
結局分からない。**このスクリプトが答えを出せることをもって満たしたとする。**

出力するもの:

- 状態遷移(`DISCONNECTED`/`STANDBY`/`READY`/`NAVIGATING`/`FAULT`/`E_STOP`)と
  **`fault_reason`**(`cmd_timeout` / `operator_lost` / `tf_stale` / `sensor_stale` /
  `bridge_disconnected` / `e_stop`)
- Goal の受理・結末(成功/中断/取り消し)
- 速度指令が非ゼロだった区間と、ゼロに落ちた瞬間
- TF / センサー / heartbeat の健全性が変化した瞬間
- `/rosout` の WARN 以上

使い方:

    ./explain_run.py <bagディレクトリ>
    ./explain_run.py <bagディレクトリ> --verbose   # 全ログを出す

⚠️ ROS 環境を source した端末で実行すること(`rosbag2_py` を使う)。
"""

from __future__ import annotations

import argparse
import sys
from typing import Any

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover
    print(f"[explain] ROS 2 の Python パッケージが読めない: {exc}", file=sys.stderr)
    print("[explain] ROS 環境を source してから実行すること", file=sys.stderr)
    raise SystemExit(1)

# `/navigate_to_pose/_action/status` の GoalStatus。action_msgs/msg/GoalStatus より
GOAL_STATUS = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}
LOG_LEVEL = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def read_bag(path: str):
    """bag を時刻順に (topic, message, t_ns) で返す。"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        type_name = type_map.get(topic)
        if type_name is None:
            continue
        try:
            msg = deserialize_message(data, get_message(type_name))
        except Exception:  # 型が読めないトピックは飛ばす(要約には使わない)
            continue
        yield topic, msg, t_ns


def kv(status: Any, key: str) -> str | None:
    """DiagnosticStatus の KeyValue を引く。"""
    for item in getattr(status, "values", []):
        if item.key == key:
            return item.value
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="rosbag から巡回 1 回分の出来事を要約する")
    ap.add_argument("bag", help="rosbag2 のディレクトリ")
    ap.add_argument("--verbose", action="store_true", help="INFO ログも出す")
    args = ap.parse_args()

    events: list[tuple[float, str, str]] = []   # (相対秒, 種別, 本文)
    t0: int | None = None

    # 変化したときだけ記録するための直前値
    prev_state = prev_fault = prev_tf = prev_sensor = prev_hb = None
    prev_moving: bool | None = None
    goal_status: dict[bytes, int] = {}
    prev_plan_len: int | None = None
    topic_counts: dict[str, int] = {}

    for topic, msg, t_ns in read_bag(args.bag):
        if t0 is None:
            t0 = t_ns
        t = (t_ns - t0) / 1e9
        topic_counts[topic] = topic_counts.get(topic, 0) + 1

        if topic == "/g1/bridge_status":
            for st in msg.status:
                if st.name != "g1_cmd_router":
                    continue
                if st.message != prev_state:
                    events.append((t, "状態", f"{prev_state or '(開始)'} → **{st.message}**"))
                    prev_state = st.message
                for key, prev, label in (
                    ("fault_reason", prev_fault, "停止理由"),
                    ("tf", prev_tf, "TF"),
                    ("sensor", prev_sensor, "センサー"),
                    ("operator_heartbeat", prev_hb, "操作PC"),
                ):
                    cur = kv(st, key)
                    if cur is None:
                        continue
                    # ⚠️ **TF の理由文字列は毎回変わる**(要求時刻が進むため)。
                    # そのまま比較すると起動時に 50ms ごとの行で埋まるので、
                    # **健全か否かだけで変化を判定**し、本文は最初の1回だけ出す。
                    norm = "stale" if cur.startswith("stale") else cur
                    if norm == prev:
                        continue
                    shown = cur if len(cur) < 110 else cur[:107] + "..."
                    events.append((t, label, shown))
                    if key == "fault_reason":
                        prev_fault = norm
                    elif key == "tf":
                        prev_tf = norm
                    elif key == "sensor":
                        prev_sensor = norm
                    else:
                        prev_hb = norm

        elif topic in ("/cmd_vel_smoothed", "/cmd_vel"):
            tw = msg.twist if hasattr(msg, "twist") else msg
            moving = abs(tw.linear.x) > 1e-6 or abs(tw.linear.y) > 1e-6 or abs(tw.angular.z) > 1e-6
            if moving != prev_moving:
                if moving:
                    events.append((t, "速度指令", f"{topic} 非ゼロになった "
                                                  f"(vx={tw.linear.x:.3f}, wz={tw.angular.z:.3f})"))
                else:
                    events.append((t, "速度指令", f"{topic} **ゼロになった**"))
                prev_moving = moving

        elif topic == "/navigate_to_pose/_action/goal":
            p = msg.goal.pose.pose.position
            events.append((t, "Goal", f"受理 ({p.x:.2f}, {p.y:.2f})"))

        elif topic == "/navigate_to_pose/_action/status":
            for s in msg.status_list:
                gid = bytes(s.goal_info.goal_id.uuid)
                if goal_status.get(gid) != s.status:
                    goal_status[gid] = s.status
                    name = GOAL_STATUS.get(s.status, str(s.status))
                    if name in ("SUCCEEDED", "CANCELED", "ABORTED"):
                        events.append((t, "Goal", f"**{name}**"))
                    elif name == "ACCEPTED":
                        events.append((t, "Goal", "ACCEPTED"))

        elif topic == "/plan":
            # 再計画のたびに出ると埋まるので、最初と、点数が 20% 以上変わったときだけ。
            n = len(msg.poses)
            if prev_plan_len is None or abs(n - prev_plan_len) > max(5, prev_plan_len * 0.2):
                events.append((t, "経路", f"{n} 点の global path"))
                prev_plan_len = n

        elif topic == "/g1/estop":
            if msg.data:
                events.append((t, "E-STOP", "**/g1/estop に true**"))
            else:
                events.append((t, "E-STOP", "/g1/estop に false(停止要求の取り下げ)"))

        elif topic == "/rosout":
            level = LOG_LEVEL.get(msg.level, str(msg.level))
            if msg.level >= 30 or args.verbose:
                events.append((t, f"log:{level}", f"[{msg.name}] {msg.msg}"))

    if t0 is None:
        print("[explain] bag が空", file=sys.stderr)
        return 1

    events.sort(key=lambda e: e[0])
    print(f"=== {args.bag}")
    print(f"=== 記録長: {events[-1][0]:.1f} 秒 / トピック {len(topic_counts)} 種")
    print()
    for t, kind, text in events:
        print(f"  {t:7.2f}s  {kind:<10} {text}")

    # --- 結論を出す ---------------------------------------------------------
    print()
    print("=== まとめ ===")
    # ⚠️ **停止は1回とは限らない。** 巡回中に何度も止まって復帰していることがあるので、
    # 最後の1件だけ出すと経緯を見落とす。**全部を時刻付きで並べる。**
    stops = [e for e in events
             if (e[1] == "停止理由" and e[2] not in ("", None))
             or (e[1] == "E-STOP" and "true" in e[2])]
    if stops:
        print(f"  ⚠️ **停止 {len(stops)} 回**:")
        for t, kind, text in stops:
            label = "e_stop(手動)" if kind == "E-STOP" else text
            print(f"       t={t:7.2f}s  {label}")
    else:
        print("  停止の記録なし(FAULT / E_STOP には至っていない)")

    goals = [e for e in events if e[1] == "Goal" and "**" in e[2]]
    if goals:
        print(f"  Goal の結末: {', '.join(g[2].replace('**', '') for g in goals)}")

    # 記録漏れの検出。**これが無いと「記録したつもり」で現場に出てしまう**
    core = ["/g1/bridge_status", "/rosout", "/cmd_vel_smoothed", "/tf"]
    nav2 = ["/navigate_to_pose/_action/status", "/plan"]
    missing_core = [r for r in core if r not in topic_counts]
    if missing_core:
        print(f"  ⚠️ **追跡に必要なトピックが記録されていない: {', '.join(missing_core)}**")
    present_nav2 = [r for r in nav2 if r in topic_counts]
    missing_nav2 = [r for r in nav2 if r not in topic_counts]
    if not present_nav2:
        # Nav2 を動かしていない試験(teleop など)ではこれが正常
        print("  （Nav2 のトピックが1つも無い。Nav2 を起動していない記録と判断した）")
    elif missing_nav2:
        print(f"  ⚠️ **Nav2 は動いていたのに記録漏れがある: {', '.join(missing_nav2)}**"
              "（`--include-hidden-topics` を忘れていないか）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
