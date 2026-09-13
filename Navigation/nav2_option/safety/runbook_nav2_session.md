# 当日手順書 — 実機で RViz と地図を使って Nav まで

**このファイルは当日そのまま見ながら進めるためのもの。** 上から順に実行する。

各段に **✅判定** を書いてある。**通らなければ次に進まない。**
詰まったら §9 の「よくある詰まり」を見る。

> ⚠️ **最初に読むこと**
>
> - **純正リモコンが唯一の最終防衛線。** ソフトウェアの `/g1/estop` は通信が生きている
>   前提の機構で、通信断では**送ることすらできない**。リモコンを持つ人を必ず置く
> - **指令を止めても約2秒動き続ける**（§3.3.1 の実測）。**前方に 2m 以上**空ける
> - 使える前進速度は **0.25〜0.30 m/s と狭い**（U-12）。この帯の外では
>   「動かない」か「意図しない方向へ進む」

---

## 0. 役割と持ち物

| 役割 | 担当 | 責任 |
|---|---|---|
| **停止係** | | **純正リモコンを持ち、機体から目を離さない。** 他の作業をしない |
| 操作係 | | 操作PC。RViz / Goal 送信 / heartbeat |
| 記録係 | | rosbag の開始と停止、気づいたことの記録 |

- 純正リモコン（**充電済みであることを確認**）
- 操作PC（Docker イメージ `g1-mapping-visualization:local` が入っていること）
- 会場 room_a の**前方 2m 以上**が空いていること（[test_area.md](test_area.md)）

---

## 1. 機体の準備（動かさない）

1. G1 を**地図を作ったときと同じ空間**に置く。おおよその位置と向きを覚えておく
2. 直立させる（純正リモコン）
3. **⚠️ ここから §6 まで機体は動かさない**

✅判定: 機体が直立して静止している。停止係がリモコンを持っている

---

## 2. SDK 側プロセス（発進ゲートは閉じたまま）

PC2 で。**ROS 環境を source していない端末**で行う（D-07）。

```bash
# 発進ゲートが閉じていることを確認する
grep '^G1_ARM=' /etc/default/g1-sdk-bridge     # → G1_ARM=  (空であること)
sudo systemctl start g1-sdk-bridge
journalctl -u g1-sdk-bridge -n 20 --no-pager
```

✅判定: `発進ゲートが閉じています(--arm 無し)` が出ている。
`/run/g1_bridge/cmd.sock` と `state.sock` がある

配置がまだなら [../deploy/README.md](../deploy/README.md) を参照。

---

## 3. 内蔵 SLAM を起動する（`1801`）

これが無いと `/unitree/slam_mapping/odom` が出ず、**TF が組めない**。

```bash
python3 tools/send_slam_api.py 1801
```

✅判定: `{"succeed":true, ..., "info":"Successfully started mapping."}`

```bash
ros2 topic hz /unitree/slam_mapping/odom     # → 約 9〜10 Hz
```

📌 座位のままでも通る。終了時は `1901`（§8）。
⚠️ このスクリプトは **1801 と 1901 しか送れない**（D-29）。移動系 API は構造的に送れない

---

## 4. 記録を開始する（Nav2 より先）

**何かあったとき原因を追えるのは記録だけ。** 必ず先に回す。

```bash
tools/record_nav2_run.sh --output runs/$(date +%Y%m%d_%H%M%S)_nav2
```

✅判定: `プロファイル=diag` と `トピック数=19` が出ている

---

## 5. ROS 側を起動する

### 5.1 ⚠️ **機体を静止させたまま** launch する

```bash
ros2 launch g1_navigation navigation.launch.py \
    backend:=real \
    map:=<room_a_map.yaml のパス>
```

⚠️ **起動時の自動校正は静止状態で行う。** 歩行中に校正すると歩容の上下動を拾い、
センサーの傾きを 11° 台と誤る（静止時の実測は 3.81°、A-10j）。

✅判定: ログに以下が出ている

```
base_link->livox_frame に yaw +180.00° を足した
自動校正した: ... map系で上向きが +z から 0.0xx° (0に近ければ成功)
```

⚠️ **`yaw +180.00°` が出ていなければ止める。** 自動校正は重力で roll/pitch しか
決められず、逆さ取付（U-09）のため yaw が 180° ずれる。0 のままだと
**点群が反対向きに出て、障害物が実際と逆側に現れる**（A-10l）。

### 5.2 全ノードが active か

```bash
for n in map_server planner_server controller_server behavior_server bt_navigator velocity_smoother; do
  echo "$n: $(ros2 lifecycle get /$n)"; done
```

✅判定: 6つとも `active [3]`

### 5.3 cmd_router の状態

```bash
ros2 topic echo /g1/bridge_status --once
```

この時点では **`STANDBY`** のはず（heartbeat がまだ無いため）。

---

## 6. 操作PC 側：heartbeat と RViz

### 6.1 heartbeat を開始する

```bash
g1_heartbeat_sender --host <PC2のIP>
```

⚠️ **これを止めるとロボットが止まる**（D-31）。巡回中に Ctrl-C しないこと。
操作PCのスリープも途絶と見なされる。

✅判定: PC2 側で `cmd_router` が **`READY`** になる

```bash
ros2 topic echo /g1/bridge_status --once | grep -A1 operator_heartbeat   # → alive
```

**`READY` にならない場合**、`tf` と `sensor` の値を見る（A-10h）。
`STANDBY のまま待機中: TF=... センサー=...` が理由を出している。

### 6.2 RViz

```bash
G1_DOMAIN_ID=<PC2と同じ> tools/rviz_operate.sh
```

✅判定: 地図・LiDAR の点群・`base_link` の軸が見えている

⚠️ **何も出ないときは §9 の①②を先に疑う**（`--ipc host` と RMW）

---

## 7. 自己位置を地図に合わせる

`map→odom` の初期値を求める。**局所探索の出発点になるので、ここが合っていないと
その後ずっと合わない。**

```bash
python3 tools/find_map_offset.py --yaw-range 180 --yaw-step 3
```

✅判定: `20cm以内` が **80% 以上**。残差 yaw が **±5° 以内**

出た `dx dy yaw` を控える。

- **連続 localization を使わない場合**: launch を止め、
  `g1_slam_odom_tf.py --map-to-odom <dx> <dy> <yaw_rad>` を渡して再起動
- **連続 localization を使う場合**（推奨）:
  ```bash
  python3 tools/map_localizer.py --initial <dx> <dy> <yaw_rad>
  ```
  かつ `g1_slam_odom_tf.py` 側は `--no-map-to-odom` にする

```bash
ros2 topic echo /g1/localizer_status --once
```

✅判定: `message: localized`、`reason: 採用`

⚠️ `not_updating` が続く場合は**そのまま走らせない**。`reason` を見る

---

## 8. ⚠️⚠️ ここから機体が動く

### 8.1 最終の安全確認（**声に出して読み上げる**）

- [ ] 停止係がリモコンを持ち、機体を見ている
- [ ] **前方 2m 以上**空いている
- [ ] 経路上に人がいない
- [ ] RViz で自己位置が**実際の位置と一致している**（目視）
- [ ] `/g1/bridge_status` が `READY`
- [ ] `/g1/localizer_status` が `localized`

### 8.2 発進ゲートを開く

```bash
sudo sed -i 's/^G1_ARM=.*/G1_ARM=--arm/' /etc/default/g1-sdk-bridge
sudo systemctl restart g1-sdk-bridge
```

✅判定: `⚠️⚠️ 発進ゲートが開いています(--arm)` が出ている

### 8.3 走行を許可する

```bash
ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}"
```

拒否されたら**理由が message に入っている**（heartbeat 未受信 / TF・センサー異常）。

### 8.4 最初の Goal は **短く**

RViz の **「2D Goal Pose」**で、**1〜2m 先**を指定する。いきなり遠くを指さない。

✅判定: 機体が歩き出し、RViz の経路に沿って進み、止まる

### 8.5 段階的に伸ばす

1〜2m → 3〜5m → 障害物を回る経路、と**一歩ずつ**広げる。
各回で **RViz の自己位置と実際の位置のずれ**を目視で確認する。

---

## ⛔ 中止条件（1つでも該当したら即座にリモコンで停止）

- 経路上に人や物が入った
- **RViz の自己位置が実際とずれている**（localization が壊れている）
- 機体が経路から外れて進む
- `/g1/bridge_status` が `FAULT` / `E_STOP` になったのに機体が止まらない
- `/g1/localizer_status` が `not_updating` のまま走っている
- 異音・異常な歩容
- **判断に迷ったら止める**

停止後は §8.2 を逆に実行してゲートを閉じる（`G1_ARM=` を空に戻して `restart`）。

---

## 9. よくある詰まり

| 症状 | 疑うこと |
|---|---|
| **RViz に何も出ない** | ① `--ipc host`（`rviz_operate.sh` は指定済み）② `RMW_IMPLEMENTATION` と `ROS_DOMAIN_ID` が PC2 と一致しているか。`view_map_rviz.sh` は CycloneDDS なので真似ない |
| **`READY` にならない** | `bridge_status` の `tf` / `sensor` / `operator_heartbeat` を見る。理由が入っている |
| **`enable_navigation` が拒否される** | heartbeat 未受信 / TF・センサー異常 / 状態が `READY` でない。`message` に理由が出る |
| **Goal を送っても動かない** | `/cmd_vel_smoothed` が流れているか。`NAVIGATING だが指令が届いていない` の WARN が出ていないか |
| **点群が反対向き** | `--lidar-yaw 180` が効いているか（§5.1） |
| **自己位置が合わない** | §7 をやり直す。`find_map_offset.py` の `20cm以内` を見る |
| **急に止まった** | `/g1/bridge_status` の `fault_reason`。`operator_lost` なら heartbeat が途絶えている |
| **FAULT から戻したい** | `ros2 service call /g1/clear_fault std_srvs/srv/Trigger {}` |
| **E_STOP から戻したい** | `/g1/estop` に **false** を送ってから `/g1/clear_estop`（両方必要） |

---

## 10. 撤収

```bash
# ① 発進ゲートを閉じる(最優先)
sudo sed -i 's/^G1_ARM=.*/G1_ARM=/' /etc/default/g1-sdk-bridge
sudo systemctl restart g1-sdk-bridge
# ② 記録を止める
pkill -f 'ros2 bag record'
# ③ 内蔵SLAM を止める
python3 tools/send_slam_api.py 1901
# ④ ROS 側と SDK 側
sudo systemctl stop g1-sdk-bridge
```

### その場で記録を読む

```bash
python3 tools/explain_run.py runs/<今日の記録>
```

**止まった理由が時刻付きで出る。** 記憶が新しいうちに確認しておく。

---

## 11. この日に取りたいデータ

配線が通ることの確認だけで終わらせない。**次に繋がる数字**を持ち帰る。

| # | 測ること | なぜ |
|---|---|---|
| ① | **既知点に戻したときの自己位置のずれ** | U-16（歩行中の odometry 精度）。「何メートル走れるか」が決まる |
| ② | localization **有り / 無し**で同じ Goal をやった差 | 連続補正の効果。`--no-map-to-odom` の有無で切り替えられる |
| ③ | **heartbeat を切ってからの前進距離** | D-31 の受入試験。目標 0.35m 以内（従来の実測は 0.85m） |
| ④ | Goal 到達時の位置誤差 | Goal 許容誤差 0.20m に対して十分か |
| ⑤ | Orin の CPU 負荷 | U-14。localizer を足した状態で余裕があるか |

⚠️ ①②は**同じ場所・同じ Goal** で比べないと意味が無い。床に印を付けておく。
