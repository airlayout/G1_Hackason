# EStop 設計書 — G1 安全停止

状態: 実装済み（ロジックと擬似ロボットでのテストのみ。**実機未検証**）
関連決定: `Navigation/nav2_stable/safety/README.md` の D-13（異常時に自動で Damp/ZeroTorque へ遷移させない）

## 1. 目的

ZB2-BE102（1NC）の緊急停止ボタンを Jetson の USB に繋ぎ、押されたら G1 を
**危険な動作をしない状態で静止** させる。トルクをゼロにする「切る」停止ではない。

## 2. 安全停止の定義

### 不変条件

1. 停止要求から **新しい動作を始めない**（歩行・腕・ナビの指令を遮断: `permits_motion=False`）
2. 歩行は **速度 0 で立ったまま静止**（`StopMove`）。直立への姿勢直しはしない
3. 腕は **現在の関節角を保持**。プリセット姿勢へは動かさない
4. ハンドは **握りを現状維持**。開かない・強めない
5. 停止は **ラッチ**。ボタンを戻しても自動では再開しない
6. **不明は停止側**: USB 抜け・断線・読み取り失敗・センサ不明はボタン押下と同じ
7. **自動でダンピング/トルクゼロにしない**（D-13）。ダンピングは操作者の明示操作のみ

### 状態

| 状態 | 歩行 | 腕 | ハンド | 意味 |
|---|---|---|---|---|
| ARMED | 上位に任せる | 上位 | 上位 | 通常 |
| STOPPING | 速度0 | 保持 | 現状維持 | 停止要求後、静止待ち |
| SAFE_HOLD | 速度0 | 保持 | 現状維持 | **目標状態**。立位で静止 |
| UNSTABLE | 速度0 | 保持 | 現状維持 | 静止できない/傾いた。**警報**。操作者の判断待ち |
| DAMPED | ダンピング | 保持 | 現状維持 | 操作者が `damp` を指示した |

### 遷移

| # | 遷移 | 条件 |
|---|---|---|
| 1 | ARMED → STOPPING | ボタン開 / USB 抜け / 読み取り失敗（即時） |
| 2 | STOPPING → SAFE_HOLD | 傾き ≤10°、角速度 ≤0.1 rad/s（速度が取れれば ≤0.05 m/s）を 0.5 s 連続 |
| 3 | STOPPING → UNSTABLE | 傾き ≥35° |
| 4 | STOPPING → UNSTABLE | 要求から 3 s で #2 を満たさない |
| 5 | SAFE_HOLD → UNSTABLE | 傾き ≥35° |
| 6 | UNSTABLE → SAFE_HOLD | 傾き <35° で #2 の静止条件を 0.5 s 連続（立ち直った） |
| 7 | STOPPING/SAFE_HOLD/UNSTABLE → DAMPED | 操作者の `damp`（ARMED では不可） |
| 8 | 非 ARMED → ARMED | `reset`: ボタンが 2 s 連続で健全 **かつ** 操作者の入力 |

観測値が `None`（不明）のときは静止判定を満たさない。SAFE_HOLD には永久に入れず、#4 で UNSTABLE になる。

### なぜ自動でダンピングしないか

D-13 に従う。ダンピングは足を出す回復動作もできず崩れ落ちるため、人が状況を見て選ぶ。
UNSTABLE ではゼロ速度を保ち、警報（ログ）を出し続ける。

## 3. 構成

```
USBボタン ─▶ ButtonSource ─┐
                           ├─▶ monitor.tick ─▶ SafeStop.step ─▶ Command ─▶ RobotPort.apply
ロボット  ─▶ RobotPort.observe ┘                 ▲
操作者(stdin: reset/damp) ───────────────────────┘
```

| モジュール | 責務 |
|---|---|
| `estop/safe_stop.py` | 純粋な状態遷移。時刻・入力を引数で受け、副作用なし |
| `estop/io.py` | `ButtonSource`（`SerialLoopSource`）、`RobotPort`（`G1Port`） |
| `estop/monitor.py` | 100 Hz ループ。遷移を `[estop]` ログに出す（停止イベント記録の元） |
| `estop/__main__.py` | `python -m estop --port ... --iface ...` |

Sim / DDS / ROS2 の差し替えは `RobotPort` の実装で行う（プロジェクトの抽象層方針に合わせた）。
`unitree_sdk2py` と `serial` は関数内 import で、Mac でもモジュールを読める。

## 4. ハードウェア

NC 接点を USB-シリアル変換器の DTR 出力 → 接点 → CTS 入力に入れる。
押下・断線・USB 抜けのどれでも CTS が落ちる/例外になり、停止側に倒れる。

## 5. 実機で決めること（未確認・未実装）

| 項目 | 状況 |
|---|---|
| DTR/CTS の極性、無接続時の CTS の読み | 変換器ごとに違う。「押す前 True / 押した後 False / USB 抜きで False」を実測 |
| `G1Port` | `unitree_sdk2py` の API 名は想定。実機未検証 |
| 腕・ハンドの保持 | `G1Port` では未実装。上位層が `permits_motion` を見て指令を止める前提 |
| 閾値（35°、3 s、0.05 m/s など） | すべて未実測の仮置き。転倒直前の傾きと停止時間を実測する |
| 胴体の水平速度 | IMU だけでは得られない。`G1Port` は角速度と傾きで判定 |
| しゃがみ・座り・横たわり中の `StopMove` | 挙動未確認。立位のみを想定 |
| 純正リモコン/物理停止手段 | 本機能とは独立に常に使える状態にしておく（safety README 大原則 1） |

## 6. 限界

ソフトの停止は Jetson・ソフト・DDS・USB のどれかが落ちると効かない。
ハードの非常停止（電源遮断・純正リモコン）は別に持つ。

## 7. テスト

```bash
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

`tests/test_safe_stop.py`（遷移 14 件）と `tests/test_monitor.py`（擬似ボタン/ロボットでの結線 4 件）。
