# Mapping/real/ubuntu — デモ機で LiDAR を見せる（デモ実演2）

Ubuntu ゲーミング PC（`192.168.123.200`）から G1 の LiDAR を RViz2 に出す。
`Demo/02_lidar.sh` がここを呼ぶ。直接叩いてもよい。

| スクリプト | 実機 | 何をするか |
|---|---|---|
| `lidar_replay.sh` | 不要 | 記録した 591 秒を再生して RViz2 に出す |
| `fetch_bag.sh` | 不要 | Mac から記録（3.0GB）を持ってくる |
| `lidar_live.sh` | **要** | 実機の点群をライブで出す（経路(A)） |
| `check_lidar.sh` | **要** | トピックが見えるか・**型が解決できるか**を調べる |
| `g1_lidar.rviz` | — | 上記が共有する RViz2 の表示設定 |

## 2 つの経路

**経路(A) — G1 の DDS を購読する（第一候補）**
ロボットが既に配信している `/utlidar/cloud_livox_mid360` を読むだけ。
**ロボット内部と競合しない。**

⚠️ Unitree の DDS は ROS 2 の type hash を載せない。Humble では 31fps で出せた実績が
あるが（2026-09-06）、**Jazzy では未検証**。`check_lidar.sh` で型が解決できるか先に見る。

**経路(B) — Livox ドライバを直接繋ぐ（保険）**
`Teleop/vendor/g1-starter-kit/setup/install_livox.sh` を jazzy で通す
（上流の `build.sh` は jazzy 対応済み）。ただし **LiDAR の占有がロボット内部と競合する**。
キットの `docs/05-lidar.md` に「ロボットを再起動し起動直後に実行」とあり、
デモの手順としては弱い。(A) が駄目なときだけ倒す。

⚠️ **OMEN に Livox 環境は入っていない**（2026-09-10 実測。`~/ws_livox` と
`~/.local/lib/liblivox_lidar_sdk_shared.so` が両方とも不在）。(B) に倒すならビルドが先。

## 記録について

`Mapping/real/runs/20260906T135940_UiS_room_v3/raw/rosbag2/`（3.0GB / 136,047 msg / 591 秒）。
`Mapping/real/runs/` は `.gitignore` 済みなので git には乗らない。`fetch_bag.sh` で持ってくる。

入っているのは 4 トピックだけで、**`/tf` は入っていない**。

| トピック | frame_id | 件数 |
|---|---|---|
| `/unitree/slam_mapping/points` | `map` | 5,890 |
| `/unitree/slam_mapping/odom` | `map` | 5,890 |
| `/utlidar/cloud_livox_mid360` | `livox_frame` | 5,900 |
| `/utlidar/imu_livox_mid360` | `livox_frame` | 118,367 |

frame が 2 つに割れているので、`lidar_replay.sh` が `map -> livox_frame` の**静的 TF**を
出して両方見えるようにしている。恒等変換なので**生の点群は原点に貼り付く**（ロボットに
追従しない）。見せたい絵は「SLAM が積んだ地図」のほう。

## 踏んだ罠（消さないこと）

- **`--loop` を使わない。** 巻き戻すと時刻が戻って TF が全部死ぬ（`TF_OLD_DATA` の山）
- **`ros2 topic list` には `--no-daemon` を付ける。** `ros2 daemon` は先に起動したときの
  DDS 設定でグラフをキャッシュし、**嘘をつく**
- **`ping` で機器の生死を見ない。** 「コマンドが無い」と「届かない」が同じ NG になる。
  `Demo/lib.sh` の `tcp_probe`（`/dev/tcp`）を使う
- **`ROS_DOMAIN_ID` が経路で違う。** 実機を購読するときは **0**（Unitree DDS に合わせる）、
  再生は Demo の既定（42）で完結させる
- **純正 SLAM の `frame_id="map"` は map ではない。** 実体はロボット基準（＝odom）。
  他の地図と重ねるとずれる

## 未検証（2026-09-10 時点）

- Jazzy が Unitree の DDS 型を解決できるか（`check_lidar.sh` で判る）
- Jazzy の rosbag2 が version 4 / sqlite3 の bag を読めるか（`lidar_replay.sh` で判る）
- `offered_qos_profiles: ""` が Jazzy のパーサを通るか
