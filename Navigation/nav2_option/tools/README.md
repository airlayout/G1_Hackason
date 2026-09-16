# tools/

nav2_option トラックの実機作業で使う道具。**どれも実機を動かす指令は出さない**
（例外は `send_slam_api.py` だが、移動 API は構造的に送れないようにしてある）。

## 動く場所が2種類ある

| 動かす場所 | 理由 | 該当スクリプト |
|---|---|---|
| **操作PC（このPC）の Docker コンテナ内** | このホストに ROS 2 が無い。Mapping のイメージ（Nav2 を追加済み）を使う | `grab_*.py` / `record_odom_events.py` / `watch_builtin_slam.py` / `read_slam_info.py` / `check_imu_attitude.py` / `check_gravity_tf.py` / `g1_slam_odom_tf.py` / `send_goal_watch.py` / `run_nav2_live.sh` / `view_map_rviz.sh` |
| **PC2（`ssh g1`）の system python3.8** | `unitree_sdk2py` が入っているのはこちら（pixi の 3.11 には入らない） | `send_slam_api.py` / `record_leg_joints.py` |
| どちらでも（ROS 不要） | numpy/scipy だけで動く | `pointcloud_to_occupancy_grid/` / `match_scan_to_map_2d.py` |

コンテナ内で動かすときの定型（詳細は
[../findings/g1_dds_sensors.md](../findings/g1_dds_sensors.md) §5）:

```bash
docker run --rm --network host --ipc host \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=0 \
  -e CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp3s0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>' \
  -v <出力先>:/work g1-mapping-visualization:local \
  bash -c 'source /opt/ros/humble/setup.bash && python3 /work/<script>'
```

## 一覧

### 実機の状態を見る（購読のみ）

| スクリプト | 用途 |
|---|---|
| `read_slam_info.py` | `/slam_info` の `ctrl_info`/`robot_data` を読む。SLAM の状態（`ctrName`）・バッテリ・CPU が分かる |
| `check_imu_attitude.py` | IMU の重力から**センサーの傾き**を出す。姿勢は同じ「座位」でも変わるので、計測前に毎回確認する |
| `watch_builtin_slam.py` | 内蔵 SLAM の odom/points が流れているかとレート・静止ドリフトを見る |
| `check_gravity_tf.py` | 重力を TF で変換し、`map←livox_frame` が重力整列しているかを検算する |
| `grab_live_cloud.py` | 生点群を N スキャン取って `.npy` に落とす |
| `grab_cloud_and_imu.py` | 点群と IMU 重力を同時に取る（水平化に使う） |
| `fit_floor.py` | ↑ で取った点群に床平面を当て、**センサーの傾きと高さ**を出す。**立っているかの判定はこれで行う**（IMU の傾きだけでは全高が出ているか分からない）。立位の基準は 3.81° / 1.213m |
| `why_costmap.py` | LocalCostmap が機体の周りを塗る理由を切り分ける。z のヒストグラムと方位分布を出し、「床の水平の狂い（全方位に一様）」と「近くの物や人（方位が偏る）」を区別する |
| `watch_slam_alive.sh` | **内蔵SLAM の生存監視。** 60秒ごとに odom の有無を `/tmp/slam_watch.log` に記録する。2026-09-15 に**約16分で勝手に止まる**ことが分かったため用意した（3回とも 941〜1022 秒） |
| `count_costmap.py` | LocalCostmap の lethal セルを距離帯ごとに数える。**footprint の内側に lethal があると Nav2 は経路を出せない**ので、Goal を送る前の判定に使う |
| `hb_stability.sh` | heartbeat の `operator_heartbeat_age_s` を集計する。**その網で `operator_timeout_s` が足りるか**を決めるための道具（Q12） |
| `make_fastdds_peers.sh` | マルチキャストが通らない網（スマホのテザリング等）向けに FastDDS の initial peers 設定を作る。⚠️ **マルチキャストアドレスも peer に入れること**（入れないと機体が見えなくなる） |

### 計測（実機を動かす試験の記録側）

| スクリプト | 用途 |
|---|---|
| `record_leg_joints.py` | **PC2 で動かす。** LowState の脚12関節を **約1000Hz** で記録。歩容が成立したか・いつ止まったかを ms 分解能で測れる（支持具で胴体が並進しなくても測れるのが利点） |
| `record_odom_events.py` | 内蔵 SLAM の odom を記録し、**動いた区間ごとに Δx/Δy/Δyaw と「機体前方からの進行方向のずれ」**を出す。U-08/U-10/U-11/U-12 の計測に使った |

⚠️ **`record_odom_events.py` は事前に `send_slam_api.py 1801` で内蔵 SLAM を起動しておく必要がある**
（終わったら `1901`）。odom は 9.1Hz で、**歩容の振動を分離できない**ので瞬間速度は信用しないこと
（実測: 平均 0.21〜0.24 m/s に対し瞬間値 0.52〜0.55 m/s）。

### 実機へ送る（移動 API は送れない）

| スクリプト | 用途 |
|---|---|
| `send_slam_api.py` | **PC2 で動かす。** `slam_operate` の **1801(建図開始)/1901(SLAM終了) だけ**を送る。`unitree_sdk2py` の RPC は `_RegistApi()` した api-id しか送れない性質を使い、**1102(移動) は登録しないので送れない**（D-29） |

### 地図

| スクリプト | 用途 |
|---|---|
| `pointcloud_to_occupancy_grid/` | 点群地図 → Nav2 `map_server` 形式。床検出・レイトレーシング・軌跡カーブ入り（A-7） |
| `match_scan_to_map_2d.py` | 重力整列済みの点群を地図へ **(x,y,yaw) の3自由度**で大域照合する。曖昧さをピーク比で報告する |

⚠️ `match_scan_to_map_2d.py` で**静止した1視点の点群を合わせるのは原理的に弱い**
（MID-360 の視野は -7°〜+52° で上向きに偏り、見える点の61%が近い天井）。
詳細と失敗の記録は [../findings/g1_dds_sensors.md](../findings/g1_dds_sensors.md) §6.3。

### Nav2 / 可視化

| スクリプト | 用途 |
|---|---|
| `g1_slam_odom_tf.py` | 内蔵 SLAM の odom を **TF(`odom→base_link`) と `/odom`** に変換する。内蔵 SLAM は TF を出さないので必須。`--auto-level` が起動時に IMU と SLAM 姿勢から `base_link→livox_frame` を逆算する |
| `nav2_live_wiring.yaml` | 実機データで Nav2 の配線を通すための **Humble 用**設定（本番は Jazzy 向けの `g1_ws/.../nav2_params.yaml`） |
| `run_nav2_live.sh` | 上記一式を順に起動する。⚠️ `set -u` を使うと ROS の `setup.bash` が落ちる |
| `send_goal_watch.py` | `NavigateToPose` にゴールを送り `/cmd_vel` が出るかを見る |
| `view_map_rviz.sh` / `view_map.rviz` | 生成した地図を RViz2 で表示する（Linux ホスト用。Mac は Mapping 側の VNC 経路） |
| `publish_trajectory_path.py` | mapping 軌跡を `nav_msgs/Path` で配信（地図の自由空間に乗っているかの目視確認用） |

## モックでの回帰テスト（実機不要）

| ツール | 用途 |
|---|---|
| `mock_deadlock_test.sh` | **旋回デッドロックの再現と修正確認。** `old` で 2026-09-15 の現象を再現し、`new` で直ることを確認する。モックのビルドから Goal 到達まで自動 |

```bash
./tools/mock_deadlock_test.sh old    # 60秒経っても 1mm も動かない（再現）
./tools/mock_deadlock_test.sh new    # Reached the goal!（修正確認）
```

⚠️ **モックはコンテナ内でビルドすること**（ホストのバイナリは `GLIBCXX` が合わず動かない）。
⚠️ **`heartbeat_required:=false` が要る**（既定 true だと `enable_navigation` が拒否される）。
詳細: [../findings/mock_deadlock_repro.md](../findings/mock_deadlock_repro.md)

## 巡回の記録と事後解析（A-10i、Phase 2c 完了条件）

| ツール | 用途 |
|---|---|
| `record_nav2_run.sh` | 巡回1回分を rosbag に記録する。`--profile diag`(既定、約3.8MB/分) / `full`(生点群込み、GB級) |
| `explain_run.py` | 記録を時系列に要約し、**なぜ止まったのか**を特定する |

```bash
# 記録(Nav2 より先に立ち上げてよい)
./record_nav2_run.sh --output runs/$(date +%Y%m%d_%H%M%S)_nav2

# 事後解析
./explain_run.py runs/20260913_120000_nav2
```

⚠️ `--include-hidden-topics` が無いと `/navigate_to_pose/_action/*` が黙って
記録されない。`explain_run.py` が記録漏れを警告するので、**録った直後に一度流すこと。**

## 実機で Nav2 を運用する（A-10k）

| ツール | 用途 |
|---|---|
| `nav2_operate.rviz` | 運用用の RViz 設定。地図・costmap・経路・LiDAR・**2D Goal Pose ツール** |
| `rviz_operate.sh` | 操作PC 側で RViz を起動する |

```bash
./rviz_operate.sh                 # 既定(ROS_DOMAIN_ID=0 / FastDDS)
G1_DOMAIN_ID=3 ./rviz_operate.sh
./rviz_operate.sh stop
```

⚠️ **RViz に何も出ないときに真っ先に疑う2つ**

1. `ROS_DOMAIN_ID` が PC2 と一致しているか
2. `RMW_IMPLEMENTATION` が PC2 と一致しているか（既定は FastDDS。
   `view_map_rviz.sh` は CycloneDDS なので真似ると食い違う）

Docker で動かす場合は **`--ipc host` が必須**。無いと FastDDS の共有メモリ転送が
成立せず、**トピックは見えるのにデータが流れない**（`rviz_operate.sh` は指定済み）。

⚠️ `view_map.rviz` は地図閲覧用で、Goal ツールも costmap も入っていない。
運用には `nav2_operate.rviz` を使うこと。
