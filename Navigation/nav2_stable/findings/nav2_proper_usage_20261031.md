# Nav2 の機能を正しい使い方に直した（2026-10-31、ブランチ `Dev/Nav2Proper`）

**目的: Nav2 が標準で持っている仕組みで解ける問題を、シェルのループや設定値の我慢で
しのいでいた箇所を、Nav2 本来の使い方に置き換える。**

⚠️ **モック（Humble + navigation2 の Docker）でのみ確認した。実機では未確認。**
実機で最初に見ることは §4 にまとめた。

## 1. 何を変えたか

| # | 問題 | これまで | 変更 |
|---|---|---|---|
| 1 | 歩くほど足元に古い観測が溜まり `collision ahead` で止まる（[collision_ahead_cause_20260925.md](collision_ahead_cause_20260925.md)） | 20 秒ごとに `clear_entirely_local_costmap` を叩くループを手で回した（本物の障害物も消える） | local costmap を **VoxelLayer → ObstacleLayer** |
| 2 | 復帰動作のその場旋回（Spin）が内蔵 SLAM をずらす。後退（BackUp）は試していない動作 | 「外すと bt_navigator が起動しない」ので外せなかった（HANDOVER 落とし穴 8） | **G1 用の BT** に差し替え、Spin と BackUp を BT と behavior_server の両方から外した |
| 3 | 人の前で止まる仕組みが、経路追従の失敗と区別できない | RPP の衝突判定のまま `failure_tolerance: 10.0` で粘った | **Collision Monitor** を velocity_smoother と g1_cmd_router の間に入れた |
| 4 | Collision Monitor（Humble）は止めてから `stop_pub_timeout` 秒で 0 速度の publish もやめる → g1_cmd_router が `cmd_timeout` で FAULT | — | `stop_pub_timeout: 3600.0` |
| 5 | BT の Wait は 5 秒間指令を出さない → Goal 実行中なので `cmd_timeout` で FAULT。以前 `nav2_activity_status_topics` に `/wait` を入れていたのは、**逆の効き方**をしていた（Goal が走っている扱いになるだけで、止める効果は無かった） | — | g1_cmd_router に `nav2_silent_status_topics`（既定 `/wait/_action/status`）。Wait の実行中だけ `cmd_timeout` を止める。上限 `max_silent_suspend_s`（10 秒） |
| 6 | global costmap が保存地図しか知らず、ふさがれた通路を通る経路を引き続ける | 障害物層を外していた（`/scan` が無いため） | 点群の ObstacleLayer を戻した |
| 7 | MID-360 が「満杯」と「1 点だけ」のフレームを交互に出す。最新 1 通しか見ない Collision Monitor と costmap が周期的に目隠しになる | — | `scripts/cloud_gate.py` で 100 点未満のフレームを捨てて `/g1/cloud_dense` に中継。g1_cmd_router のセンサー鮮度もこちらを見る |

`failure_tolerance` は 10.0 → 5.0 にした。人の前の停止は Collision Monitor が受け持ち、
消し残りは BT が FollowPath の失敗時に消すので、長く粘る理由が減ったため（実機で詰めた値ではない）。

### 指令の流れ

```text
controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed
  → collision_monitor → /cmd_vel_safe → g1_cmd_router → SDK
behavior_server(wait のみ) ─┘(/cmd_vel_nav)
/utlidar/cloud_livox_mid360 → cloud_gate → /g1/cloud_dense → local/global costmap, collision_monitor, g1_cmd_router(鮮度)
```

`collision_monitor:=false` で Collision Monitor を外し、g1_cmd_router が `/cmd_vel_smoothed` を
直接読む元の配線に戻せる。

## 2. なぜ ObstacleLayer で消し残りが消えるのか（1 の理由）

VoxelLayer は**3 次元で**レイトレースする。センサーは床から 1.21m にあるので、
4m 先の壁（高さ 0.5m）へ向かうレイは、機体の 1.5m 前では高さ約 0.94m を通る。
**床近く（0.1〜0.3m）のボクセルをレイが一度も通らない**ので、そこに立ったマークは
永久に残る。MID-360 の死角（床の高さで半径 0.91〜1.12m）の中はなおさらである。

ObstacleLayer は**レイを床に投影して 2 次元で**消す。足元の格子も、その先の何かへ
向かうレイが通るたびに消える。処理は「消す → 付ける」の順なので、**いま見えている
障害物は消えない**。消えるのは「その方向に何も見えていない」格子だけ。
findings §5 の案 (a)「死角の中は信用しない」を、層の選び方だけで実現したことになる。

失うのは 3 次元の情報（机の下をくぐれる等）だが、2 次元の歩行計画ではもともと使っていない。

## 3. モックでの確認（`tools/mock_nav2_proper_test.sh`）

Docker イメージは Humble + `ros-humble-navigation2`（PC2 と同じ Humble）。

| # | 内容 | 結果 |
|---|---|---|
| ① | 全ライフサイクルノード（collision_monitor を含む）が active。BT は `navigate_to_pose_g1.xml`、behavior_plugins は `['wait']`、g1_cmd_router は `/cmd_vel_safe` を読む | ✅ |
| ② | 機体の 0.55m 前に人を立たせる（costmap には載らない距離＝Collision Monitor だけが見える） | ✅ `/cmd_vel_smoothed` 0.28 → `/cmd_vel_safe` 0.0。15 秒後も `NAVIGATING`。人が去ると **Goal succeeded** |
| ② 対照 | 同じ試験を `stop_pub_timeout: 1.0`（既定）で | ✅ **`cmd_timeout` で FAULT → Goal 取り消し**（4 の変更が効いていることの裏付け） |
| ③ | 人を 120 秒立たせ、BT を Wait まで進める | ✅ `Running wait` 1 回、その間も `NAVIGATING`。人が去ると **Goal succeeded** |
| ④ | 床から 0.1〜0.3m の障害物を 1.5m 前に見せたあと取り除き、壁だけを見せ続ける（センサー高 1.21m） | ✅ ObstacleLayer: 100 → **0（消えた）**／旧 VoxelLayer: 100 → **100（残った）** |

📌 ③ の対照（`nav2_silent_status_topics` を空にすると FAULT になるか）は取っていない。
launch がこのパラメータを公開していないため。② 対照と同じ仕組み（指令が途切れると FAULT）
なので、結果は推して知れるが、実測ではない。

### 試験を書いていて踏んだこと

- **3 次元の距離で `obstacle_min_range` が効く。** 人の点を高さ 0.9m まで取ったら、
  base_link から 0.65m 前でも 3 次元では 0.9m を超えて costmap に載り、Collision Monitor より
  先に RPP が止めた（1 回目は Collision Monitor を試せていなかった）
- **点群を 1 秒でも途切れさせると `sensor_stale` で FAULT。** fake_sensor_publisher を止めてから
  人の点群を出すと、Python の起動の間に途切れる。重ねて流すこと

## 4. 停止範囲が壁にかからないか（受け取った地図での確認）

`tools/check_stop_zone_on_route.py` で、`room_a_map_20260911_edited.yaml` と
`patrol_room_a.yaml` の 6 点の間に（NavFn の代わりに）最小コスト経路を引き、
前向きに歩かせたとき停止範囲に入る占有セルを数えた。

| 停止範囲 | 壁が入る区間（巡回路 24.4m のうち） |
|---|---|
| 前方 0.35〜1.0m・左右 ±0.25m（**採用**） | **0.00m** |
| 前方 0.35〜1.0m・左右 ±0.45m | 1.55m |
| 前方 0.35〜2.0m・左右 ±0.60m | 10.75m（道具が壁を検出できることの確認） |

横方向の余裕は約 0.2m。⚠️ 地図の占有セルは高さを持たないので安全側（多めに数える）の近似。
経路も NavFn + RPP の実際とは数十 cm ずれうる。

📌 10/21 計測の 3D 点群地図（`room_a_3d_maps_20261021.zip`、1,128,103 点）も受け取ったが、
巡回路は 9/11 版の地図座標で書かれており座標系が合わないので、今回は使っていない。
巡回路を 10/21 版に移すときは、高さ 0.15〜1.0m の点だけで同じ確認をやり直すとよい。

## 5. 実機で最初に確かめること

1. **停止範囲に人が入ったとき、本当に点が入るか。** 停止範囲（前方 0.35〜1.0m）は
   LiDAR の死角（床の高さで半径 0.91〜1.12m）とほぼ重なる。人の胴は見える想定だが未確認。
   RViz に `/g1/collision_stop_zone`（Polygon）と `/g1/cloud_dense` を出し、
   機体の前に人が立って `Robot to stop due to StopZone polygon` が出ることを**武装前に**確かめる
2. **`cloud_gate` が満杯のフレームを通しているか。** 30 秒ごとに
   `[cloud_gate] 直近30秒: 通過 N / 破棄 M` が出る。通過が約 150（5Hz）なら正常
3. **消し残りで止まらないか。** 20 秒ごとに消すループは**回さずに**巡回させ、
   `collision ahead` の回数と `tools/count_costmap.py` の footprint 内 lethal を見る
4. 止まりすぎるときは、まず `collision_monitor:=false` で切り分ける

## 6. 戻し方

| 戻したいもの | やること |
|---|---|
| Collision Monitor | launch に `collision_monitor:=false` |
| local costmap の層 | `nav2_params.yaml` の local の `plugins` を `["voxel_layer", "inflation_layer"]`（設定は残してある） |
| BT | `bt_navigator` の `default_*_bt_xml` を消し、**behavior_server に spin / backup を戻す**（戻さないと bt_navigator が起動しない） |
| global の障害物層 | global の `plugins` から `obstacle_layer` を外す |
