# 当日手順書 — 実機で RViz と地図を使って Nav まで

**このファイルは当日そのまま見ながら進めるためのもの。** 上から順に実行する。

📌 **前回どこまで進んだか・次に何をするかは [../HANDOVER.md](../HANDOVER.md) を先に見ること。**

各段に **✅判定** を書いてある。**通らなければ次に進まない。**
詰まったら §9 の「よくある詰まり」を見る。

> ⚠️ **最初に読むこと**
>
> - **純正リモコンが唯一の最終防衛線。** ソフトウェアの `/g1/estop` は通信が生きている
>   前提の機構で、通信断では**送ることすらできない**。リモコンを持つ人を必ず置く
> - **指令を止めても約2秒動き続ける**（§3.3.1 の実測）。**前方に 2m 以上**空ける
> - 使える前進速度は **0.25〜0.30 m/s と狭い**（U-12）。この帯の外では
>   「動かない」か「意図しない方向へ進む」
> - ⚠️⚠️ **人は機体から 2m 以上離れる。後ろも含む。**
>   LiDAR は 360° 見ており、**追従して歩くと、その人が常に障害物として costmap に
>   立ち続ける**（マークも一緒に移動するので機体は「どこへ行っても囲まれている」）。
>   2026-09-15 に実測で 5m以内の点の 68.7% が障害物帯に入り、経路追従が潰れた。
>   **機体を見るのは目で。近づきたければ機体が止まってから。**
> - **内蔵SLAM は約16分で勝手に止まる**（U-17）。巡回の前に必ず入れ直す（§3）

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

⚠️ **`ssh g1` でログインすると `ros:foxy(1) noetic(2) ?` と聞かれる**（`~/.bashrc` の
fishros ブロック）。**そのまま Enter を押す。** `case` に既定節が無いので空なら何も
source されない。`1` を選ぶと Foxy + CycloneDDS 0.10.2 + `LD_LIBRARY_PATH=/usr/local/lib`
が入り、**D-06/D-07 が防ごうとしている汚染そのものが起きる**。

```bash
echo "ROS_DISTRO=[$ROS_DISTRO]"                # → [] であること
# 発進ゲートが閉じていることを確認する
grep '^G1_ARM=' /etc/default/g1-sdk-bridge     # → G1_ARM=  (空であること)
sudo systemctl start g1-sdk-bridge
journalctl -u g1-sdk-bridge -n 20 --no-pager
```

⚠️ **`sudo` はパスワードを聞かれる**（`unitree` は sudo グループだが NOPASSWD ではない）。
配置がまだなら [../deploy/README.md](../deploy/README.md) の手順を先に実行する。

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

### ⚠️⚠️ 内蔵SLAM は勝手に止まることがある（2026-09-15 実測）

機体を立てたまま放置しただけで、**`1801` から 12〜17 分で odom と points が両方止まった**。
`/slam_info` は `"info": "not init"` に戻る。LiDAR の生点群は 9.97Hz で流れ続けるので
**気づきにくい**。バッテリ 77% / CPU 55% / 59℃ と余裕のある状態で起きた。原因は未特定。

**`1801` を再送すれば完全に復帰する**（odom も TF も戻る）。巡回中は監視すること:

```bash
setsid nohup bash tools/watch_slam_alive.sh > /dev/null 2>&1 &   # 60秒ごとに /tmp/slam_watch.log
```

⚠️⚠️ **落ちたら §7 をやり直すこと。** 再起動すると **odom 原点が機体の現在地にリセット**
されるため、§7 で求めた `map→odom` が無効になる。機体が 1801 を送った場所から動いて
いれば、その移動量ぶん飛ぶ。`map_localizer.py` は局所探索なので窓を超えて追従できない。

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

**PC2 で、pixi の Humble 環境から起動する**（D-30 の PC2 側実体。
[../deploy/pc2_humble/](../deploy/pc2_humble/) 参照）。

```bash
cd ~/g1_nav2/pc2_humble
~/.pixi/bin/pixi run bash -lc '
  source ~/g1_nav2/g1_ws/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 launch g1_navigation navigation.launch.py \
      backend:=real \
      map:=<room_a_map.yaml のパス>'
```

⚠️ **`RMW_IMPLEMENTATION` は必ず明示する**（A-10e の教訓）。点群は
**FastDDS のほうが安定**（2026-09-15 実測: FastDDS 9.98Hz / CycloneDDS 8.18Hz、
最大遅れ 0.301s）。RViz 側（`rviz_operate.sh`、既定 FastDDS）と揃うこと。

⚠️ **起動時の自動校正は静止状態で行う。** 歩行中に校正すると歩容の上下動を拾い、
センサーの傾きを 11° 台と誤る（静止時の実測は 3.81°、A-10j）。

✅判定: ログに以下が出ている

```
base_link->livox_frame に yaw +180.00° を足した
自動校正した: ... map系で上向きが +z から 0.0xx° (0に近ければ成功)
```

### 5.1.1 ⚠️⚠️ `lidar_yaw` は **0 か 180 かを毎回その場で確かめる**

**定数では決められない**（2026-09-15 に実機で判明）。自動校正の水平化は逆さ取付だと
**約180°の回転**になり、その**回転軸が傾きの方位で決まる**ため、**姿勢によって
点群の X が反転したりしなかったりする**。

| 実測 | 傾きの方位 | 正しい `lidar_yaw` |
|---|---|---|
| 2026-09-09 の立位（傾き 3.81°） | +8.7° | **180** |
| 2026-09-15（傾き 19.04°） | -100.4° | **0** |

⚠️⚠️ **地図の一致率では判別できない。** `base_link→livox` を 180° 回しても
`map→odom` が打ち消すので、**どちらでも 95〜97% 合ってしまい、`base_link` の向きだけが
反転する**。**判別できるのは RViz の目視だけ**:

> **`base_link` の赤軸（+X＝前方）が、実機の正面と同じ方向を指しているか。**
> 逆を向いていたら、`lidar_yaw` を 0 と 180 で入れ替えて §5 からやり直す。
> （RViz の軸色は X=赤 / Y=緑 / Z=青。`livox_frame` の赤軸は `base_link` の赤軸と
> 反対を向くのが正常＝逆さ取付のため）

```bash
# 入れ替えるとき
ros2 launch g1_navigation navigation.launch.py backend:=real map:=<地図> lidar_yaw:=0
```

📌 恒久対策は「既知の取付（X 前方・roll 180°）を先に適用し、残差だけを水平化する」。
未実施なので、当日は上の目視で決めること。

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

この時点で **`READY`** になる。**`STANDBY` ではない**（2026-09-15 に実機で確認。
以前この行は「heartbeat がまだ無いので STANDBY のはず」と書いていたが**誤り**だった）。

`STANDBY→READY` のゲートは **TF とセンサーの健全性だけ**で決まる
（`cmd_router_node.cpp:288-298`）。heartbeat が効くのは `enable_navigation` を
叩いた瞬間（同 `:563`）なので、**heartbeat 無しで READY でも走り出せない**。

⚠️ **`READY` でも `tf` / `sensor` の値は必ず見ること。** READY は一方通行で、
後から TF が stale になっても **`READY` の表示は変わらない**（内蔵SLAM が落ちた
ときに実際にそうなった。§3 の警告を参照）。

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

⚠️ **`--yaw-range 180` は必須。** 既定の ±20° だと**偽のピークを掴む**。
2026-09-15 の実機で、真値（yaw 159°、20cm以内 97%）に対し既定の探索が
**yaw 3.0° を 99%（真値より高スコア）で返した**。スコアだけでは正誤を判別できない。

✅判定: `20cm以内` が **80% 以上**。かつ **RViz で位置と向きの両方が実機と一致**
していること（**向きの確認を省かない**。§5.1.1 のとおり、向きが 180° 逆でも
一致率は 95〜97% になる）

⚠️ **「残差 yaw が ±5° 以内」は実機では成り立たない**（以前ここにあった判定は誤り）。
このツールが返すのは**残差ではなく絶対値の `map→odom`** で、実機では odom 原点が
機体の現在地なので **159° のような任意の値**になる。±5° になったのは、地図原点と
odom 原点が一致していた B2 の bag 検証に固有の条件だった。
**適用後に測り直しても同じ値が返るのが正常**（2026-09-15 に確認: 159° → 162°）。

出た `dx dy yaw`（yaw は**度**。`--map-to-odom` に渡すときは**ラジアン**）を控える。

- **連続 localization を使わない場合**: launch を止め、`map_to_odom` 引数を付けて上げ直す
  ```bash
  ros2 launch g1_navigation navigation.launch.py backend:=real map:=<地図> \
      map_to_odom:="<dx> <dy> <yaw_rad>"
  ```
- **連続 localization を使う場合**（推奨）: launch を **`map_to_odom:=none`** で上げ、
  ```bash
  python3 tools/map_localizer.py --initial <dx> <dy> <yaw_rad>
  ```

⚠️ `map_to_odom` 引数は 2026-09-15 に追加した。**それ以前の launch にはこの引数が無く、
`--map-to-odom` / `--no-map-to-odom` をどちらも渡せなかった**（実機で気づいた）。

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

✅判定: **`pgrep -af g1_sdk_bridge_real_server` の末尾に `--arm` が付いている**

⚠️ **journal の `⚠️⚠️ 発進ゲートが開いています(--arm)` は当てにしない。**
2026-09-15 に、再起動後のプロセスがこの行を journal に出さなかった。
**確実なのはコマンドライン。**

⚠️ **`/etc/default` を書き換えただけでは効かない。** `restart` が要る（設計どおり。
「再起動したら勝手に動けるようになっていた」を防ぐため）。実際に書き換えただけで
「開いたか？」と確認して、開いていなかったことがある。

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
| **点群が反対向き / `base_link` の赤軸が実機の正面と逆** | `lidar_yaw` を 0 と 180 で入れ替える（§5.1.1）。**地図の一致率では判別できない** |
| **自己位置が合わない** | §7 をやり直す。`find_map_offset.py` の `20cm以内` を見る |
| **急に止まった** | `/g1/bridge_status` の `fault_reason`。`operator_lost` なら heartbeat が途絶えている |
| **TF が消えた / planner が Extrapolation Error** | **内蔵SLAM が落ちている**。`ros2 topic hz /unitree/slam_mapping/odom` が無反応なら `1801` を再送し、**§7 をやり直す**（§3 の警告） |
| **costmap が機体の周りを埋める** | 機体に密着した物・人・支持具。`tools/why_costmap.py` で方位と距離を出す。2026-09-15 は 5m以内の 71.7% が障害物帯に入り、死角半径が 0.04m（正常は 0.91〜1.12m）だった |
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
