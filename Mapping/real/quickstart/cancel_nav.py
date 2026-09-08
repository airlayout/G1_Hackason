#!/usr/bin/env python3
"""走っている `navigate_to_pose` のゴールを全部キャンセルする。

## 何のためにあるか

`check_navigation.py` の 1 回は `--timeout` を**満了するまで終わらない**。
長距離では 900 s なので、**噛んで動かないと分かってからも 15 分待たされる**。
このスクリプトでゴールをキャンセルすると `navigate()` の
`result_future` がすぐ完了し、その回は「失敗（CANCELED）」として記録されて
**次の回へ進む**。最後の回でキャンセルすれば計測そのものが終わり、
`navigation.json` が書き出される。

2026-09-08 の実測では、噛んだ状態の 2 本ぶん（1,800 s の待ち）を
**約 30 分ぶん短縮**した。

## なぜ「全部」なのか

`goal_id` を全ゼロ・`stamp` を 0 にすると action の規約で
**「全部のゴール」**の意味になる。`check_navigation.py` 側は goal_id を
外に出していないので、狙い撃ちはできない。同時に 1 つしか投げないので問題ない。

## ⚠️ 使うときの約束

- **キャンセルは「時間切れ」ではない。** 記録上は `error_code=0` の失敗になる。
  作業ログには**打ち切ったことと、打ち切る前に確かめた根拠を必ず書く**
  （`why_not_moving.py` で全方位 0 m を確認した、など）。
  「900 s 待って着かなかった」と書いたら嘘になる
- **最低 1 本は満了まで測る。** 「待っても動かない」は満了した回でしか言えない
- `pkill` で計測プロセスを殺してはいけない。`check_navigation.py` は
  記録を**回ごとに上書き保存**するようになったので直近までは残るが、
  キャンセルなら区間の終端まで揃う

    source IsaacSim_Env/env.sh
    python3 Mapping/real/quickstart/cancel_nav.py
"""
from __future__ import annotations

import sys

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node

SERVICE = "/navigate_to_pose/_action/cancel_goal"
RETURN_CODES = {0: "ERROR_NONE", 1: "ERROR_REJECTED",
                2: "ERROR_UNKNOWN_GOAL_ID", 3: "ERROR_GOAL_TERMINATED"}


def main() -> int:
    rclpy.init()
    node = Node("cancel_nav")
    cli = node.create_client(CancelGoal, SERVICE)
    if not cli.wait_for_service(timeout_sec=10.0):
        print(f"[NG] {SERVICE} が居ない。Nav2 が起動しているか確かめる")
        return 1
    # goal_id 全ゼロ / stamp 0 = 全部のゴール
    fut = cli.call_async(CancelGoal.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    res = fut.result()
    if res is None:
        print("[NG] 応答が無い")
        return 1
    print(f"[OK] return_code={res.return_code}"
          f"({RETURN_CODES.get(res.return_code, '?')}) "
          f"キャンセルしたゴール {len(res.goals_canceling)} 件")
    rclpy.shutdown()
    return 0 if res.return_code == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
