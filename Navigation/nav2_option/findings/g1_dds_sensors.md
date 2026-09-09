# G1 実機の DDS トピック・センサー疎通調査（2026-09-09）

| 項目 | 内容 |
|---|---|
| 目的 | 実機を**動かさずに**できるセンサー疎通確認。Phase 2a（LIO/Localization）の前提を固める |
| 実施環境 | 操作PC（Ubuntu 24.04、x86_64）→ 有線 → G1 内蔵スイッチ |
| 手法 | 購読のみ（`ros2 topic list/info/hz/echo`）。**コマンド送信は一切していない** |

## 1. 接続構成（実測）

| ホスト | 役割 | SSH | 備考 |
|---|---|---|---|
| `192.168.123.222` | 操作PC（このPC）| — | NetworkManager 接続 `g1-link`（`enp3s0`、gateway なし）を `nmcli connection up g1-link` で有効化 |
| `192.168.123.161` | Operation and Control Computing Unit（運控PC）| **不可**（22番が connection refused）| Planning.md の「ユーザーアクセス不可」と整合 |
| `192.168.123.164` | Development Computing Unit（Jetson Orin NX）| 可（`unitree` ユーザー、`~/.ssh/config` に `g1` エイリアス）| aarch64 / Ubuntu 20.04 / ROS 2 Foxy + Noetic |

⚠️ **`g1-link` は放っておくと切断される**（本調査中に一度落ちた。物理リンクは carrier=1 のまま
NetworkManager 側が「切断済み」になる）。疎通しないときはまず `nmcli connection up g1-link` を試す。

⚠️ **G1 の電源が入っていないと `.164` は応答しない**が、`.161` は応答することがある。
`.161` だけ応答する場合は「電源は入っているが PC2 が起動していない/落ちている」を疑う。

## 2. DDS で見えるトピック（ROS_DOMAIN_ID=0、CycloneDDS を `enp3s0` に束縛）

再現コマンド（Nav2 を足した Mapping のコンテナを使う。このホストに ROS 2 は無い）:

```bash
docker run --rm --network host --ipc host \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=0 \
  -e CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp3s0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>' \
  g1-mapping-visualization:local bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'
```

### 2.1 センサー生データ（**流れていることを実測確認**）

| トピック | 型 | 実測レート | 中身 |
|---|---|---|---|
| `/utlidar/cloud_livox_mid360` | `sensor_msgs/PointCloud2` | **9.998 Hz** | `frame_id: livox_frame`、1スキャン **20,064点**（≒20万点/秒。Planning.md の想定と一致） |
| `/utlidar/imu_livox_mid360` | `sensor_msgs/Imu` | **200.099 Hz** | 下記の注意点あり |
| `/utlidar/range_info` | — | 未計測 | |

**IMU の重要な注意点（2件）**

1. **`orientation` は全ゼロ（x=y=z=w=0）＝無効なクォータニオン。** Livox 内蔵 IMU は生6軸で
   姿勢融合をしない。姿勢は LIO 側で推定するか、ロボット本体の IMU（`/lowstate` の
   `imu_state`）から取る必要がある
2. **`linear_acceleration` の単位は g（m/s² ではない）。** 静止時の実測が
   `(0.476, -0.157, -0.863)` でノルム **0.998** だった。FAST-LIO 等の設定でスケールを
   取り違えると重力が 9.8 倍ずれる

なお静止時の重力方向が -z に揃っていないことから LiDAR は傾けて/裏返して搭載されている
と分かるが、**調査時のロボットの姿勢が不明（`fsm_id=0`＝脱力状態）なので、これから
取付角（U-09）は確定できない。** 直立させた状態で測り直すこと。
参考: Mapping 班の `nav_stack.sh` は実測値として
`--livox-rpy-deg 178.35 -8.41 -0.72` / `--livox-xyz -0.004 0.016 -0.037`（2026-09-05 実測）を
使っている（ロール約 178°＝ほぼ裏返し）。**U-09 はこの値の検算で足りる可能性がある。**

### 2.1.1 点群の幾何を実測（U-09 の一次確認、2026-09-09）

静止状態で 10 スキャン（200,544点）を取得し、センサー座標系のまま解析した
（取得スクリプト: 本文書 §5）。

**① 垂直視野は仕様どおり**

| 項目 | 実測 | 仕様 |
|---|---|---|
| 観測仰角の下限 | **-6.75°** | -7° |
| 観測仰角の上限 | **+50.50°** | +52° |

**② 足元の床が見えないこと（D-21 の前提）を実測で裏付けた**

水平距離ごとの最小 z を見ると、**下端が距離に比例して下がる直線**になっている:

| 水平距離 | 1m | 2m | 3m | 5m | 8m | 12m | 16m |
|---|---|---|---|---|---|---|---|
| 最小 z | -0.08 | -0.23 | -0.30 | -0.44 | -0.69 | -0.96 | -1.42 |
| 見かけの仰角 | -4.6° | -6.6° | -5.7° | -5.0° | -4.9° | -4.6° | -5.1° |

**この下端は床ではなく視野の下限そのもの**（一定の約 -5°）。床であれば一定の z で
平らになるはずだが、16m まで平らにならない。z≈-1 前後の点が現れ始めるのは**約11m以降**で、
「高さ1.3m から -7° の光線は約10.6m 先で床に当たる」という D-21 の記述と整合する。

⚠️ **ただしセンサー高さそのものはこのデータから確定できない**（近〜中距離に床の反射が無いため）。
U-09 の最終値（死角半径）は、ロボットを**直立させた状態**で開けた床に向けて測り直すこと。
調査時の姿勢は `fsm_id=0`（脱力）で不定。

⚠️ **当初「点群は地面基準・重力整列された座標系で配信されている」と推測したが誤りだった。**
z の薄切りに平面フィットすれば必ず法線が z 軸を向くという初歩的な誤りで、
z≈0 付近の点の集中を床面と見誤った。**点群はセンサー座標系（`livox_frame`）のままである。**

**③ 機体自身が至近に写ることを定量化した**

| 水平距離 | 点数 |
|---|---|
| 0.0〜0.2 m | **9,652 点**（z≈+0.06 に集中）＝**機体自身** |
| 0.2〜0.5 m | **8 点**（明確な空白） |
| 0.5〜1.0 m | 11,569 点（周囲の什器・壁） |

`Navigation/nav2/g1_nav2.yaml` の `obstacle_min_range: 0.30`（「至近は機体自身が写る」という
注記付き）に**実測の裏付けが取れた**。0.2〜0.5m がほぼ空白なので、0.3〜0.5m の
下限設定で自己観測を確実に除外できる。

### 2.2 ⚠️ 重要な発見: G1 に**内蔵の再定位（relocalization）サービス**がある

| トピック | 型 | 実測レート |
|---|---|---|
| `/unitree/slam_relocation/odom` | `nav_msgs/Odometry` | 0（サービス未起動） |
| `/unitree/slam_relocation/global_map` | `sensor_msgs/PointCloud2` | 0（同） |
| `/unitree/slam_relocation/points` | `sensor_msgs/PointCloud2` | 0（同） |
| `/unitree/slam_relocation/web_points` | `sensor_msgs/PointCloud2` | 0（同） |
| `/unitree/slam_mapping/odom` | `nav_msgs/Odometry` | 0（同） |
| `/unitree/slam_mapping/points` | `sensor_msgs/PointCloud2` | 0（同） |
| `/unitree_slam/waypoints` | — | 0（同） |
| `/planner_map` | — | 0（同） |
| **`/slam_info`** | `std_msgs/String` | **約5.5 Hz（流れている）** |

⚠️ **訂正（2026-09-09）**: 当初この表で `/slam_info` を「0（サービス未起動）」と書いたが、
**測定せずに他のトピックから推測した誤りだった。** 実測すると `ctrl_info`（約5Hz）と
`robot_data`（約0.5Hz）が流れている。**SLAM サービス自体は起動していて状態を配信しており、
「地図を読み込んでいない」だけである。** 詳細は §7。

**これは D-19 の前提を見直す材料になる。** D-19 は SDK2 の `SportModeState_`（`unitree_hg`）に
位置・速度フィールドが無いことから「SDK2 側に internal odometry のフォールバックは存在せず、
外部 LIO が唯一の位置情報源」と結論していた。しかし**ロボット自身の SLAM サブシステムは
DDS 上に `nav_msgs/Odometry` と点群地図を、標準 ROS 型で配信する口を持っている。**
SDK2 の高レベル API（`LocoClient`）とは別系統なので、D-19 の調査（SDK2 の IDL 調査）では
見えていなかった。

- 保存地図に対する自己位置推定が `/unitree/slam_relocation/odom` から得られるなら、
  **FAST-LIO（Phase 2a の主要作業）も AMCL も不要になる可能性がある**
- 現在はいずれも**発行元は居るがデータが流れていない**＝サービス未起動。起動には
  `slam_operate` API（純正方式トラック `Navigation/nav/` が使っている API-ID 群、
  例: 1804 = 保存地図の読込＋自己位置設定）を叩く必要があると思われる
- **未確認**: 出力される pose がどの座標系か、保存地図は誰のファイルシステム上か
  （`Navigation/nav3/README.md` は「`address` は PC1 のファイルシステムを指し、外部で
  作った地図を転送する手段が見つかっていない」と記録している）

### 2.3 障害物・安全系（Unitree 内蔵）

| トピック | 型 | 実測レート |
|---|---|---|
| `/safe_clouds` | `sensor_msgs/PointCloud2` | 0（未起動） |
| `/warning_clouds` / `/no_warning_clouds` | — | 0 |
| `/pre_collision_clouds` / `/pre_safe_clouds` | — | 0 |

純正の障害物検知レイヤと思われる。純正ナビを起動していないため流れていない。

### 2.4 ロボット状態（Unitree 独自型のため `unitree_ros2` の msg が必要）

| トピック | 型 | 用途 |
|---|---|---|
| `/lowstate` | `unitree_hg/msg/LowState` | 関節・IMU。**D-19 が言う姿勢の取得元** |
| `/lowstate_doubleimu` / `/secondary_imu` | — | 追加 IMU |
| `/odommodestate` | `unitree_go/msg/SportModeState` | |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | **純正リモコンの状態**。安全監視に使える可能性 |
| `/lowcmd` / `/user_lowcmd` | — | ⚠️ **指令系。絶対に publish しないこと** |

D-02 で `unitree_ros2` を使わない方針なので、これらの独自型トピックは ROS 側から直接は
読めない。必要なら SDK 側プロセス（`g1_sdk_bridge_cpp`）が取得して IPC で渡す（D-02 の設計どおり）。

## 3. 頭部カメラ（Intel RealSense D435i）

⚠️ **接続には人手が要る。** 調査開始時は `/dev/video*` も `lsusb` の Intel デバイスも
存在せず未接続だった。**利用者がケーブルを差した後**に認識された（`lsusb`: `8086:0b3a`、
`/dev/video0`〜`video5` の6ノード）。

なお D435i は DDS には出ておらず、**LeRobot の ZMQ 画像サーバ経由**で配信する構成
（Mapping 側の `camera_bridge` が ZMQ→ROS 2 に変換する）。調査時点でサーバ
（`192.168.123.164:5555`）は未起動だった。

### 3.1 ⚠️ 重大な発見: 「color」として扱われている画像は**実は赤外(IR)画像**

`/dev/video0`〜`video5` の各ノードから実際にフレームを取得して確認した結果:

| デバイス | 取得 | 内容 |
|---|---|---|
| `/dev/video0`, `video1`, `video4` | 開けない | |
| **`/dev/video2`** | OK（UYVY 640x480）| **左IRカメラ**。IRプロジェクタのドットパターンが写る。露出オーバー気味 |
| `/dev/video3` | OK（index指定時のみ）| **右IRカメラ**。オフィスの人・椅子・カーペット・**ロボット自身の足**が明瞭 |
| `/dev/video5` | OK（同）| **深度**と思われる（大半が無効値で暗い） |

**取得した3枚はいずれも彩度0＝モノクロだった。** カラーストリームはどのノードからも
取れていない。sysfs の `index` を見ると `video0`〜`video3` が深度/IRセンサー群、
`video4`/`video5` が RGB センサー群だが、**RGB は素の V4L2/OpenCV では開けなかった**
（RealSense のカラーは librealsense 側の設定が要る。`pyrealsense2` は PC2 に**未インストール**）。

**裏付け（記録済みデータ側からの確認）**: Mapping 実行時に保存された
`Mapping/real/runs/20260904_203726_room_a/derived/rgb_preview/*.jpg` を開くと
**すべてグレースケールで、暗い床タイル上に IR プロジェクタのドットパターンが見える。**
つまり記録時点から IR を撮っていた。

**影響**

- Mapping のパイプラインは `--device 2`(=左IR) を読み、`/g1_camera/color/image/compressed`
  というトピック名・`camera_color_optical_frame` というフレーム名で配信している。
  **名前と中身が食い違っている**（機能はしているが「カラー」ではない）
- カメラ内部パラメータ（`manifest.json` の `fx/fy/cx/cy` が 0.0 で `calibration_complete: false`）を
  今後校正する際、**IR カメラと RGB カメラでは内部パラメータが別物**なので、
  どちらを校正しているのかを取り違えないこと
- nav2_option の Phase 3（D435i による死角補完）は**深度**を使うので、いずれ
  librealsense（`pyrealsense2` か `realsense-ros`）の導入が必要になる。素の V4L2 では
  深度・カラーを正しく引き出せない
- **これは `Mapping/` 側の実装に関わる指摘なので、勝手に直さず申し送りとする**

### 3.2 副産物: 「追従者」の正体が画像で確認できた

`rgb_preview` の画像には**ロボットの真横を歩く人物の脚**が写っている。これは
A-7 の軌跡カーブで削った偽の障害物（`Navigation/nav3/README.md` が「閾値を上げても消えない
斜めの筋」と報告した動的物体）と同一人物の可能性が高い。地図側の推定と画像が一致した。

## 4. まとめ

| 対象 | 状態 |
|---|---|
| MID-360 点群 | ✅ 9.998 Hz / 20,064点/スキャンで正常 |
| MID-360 IMU | ✅ 200.1 Hz。ただし orientation は無効、単位は g |
| G1 内蔵 SLAM / 再定位 | ⏸ トピックは存在するがサービス未起動（`slam_operate` で起動要） |
| 純正の障害物検知点群 | ⏸ 同（純正ナビ未起動） |
| ロボット状態（LowState 等）| ✅ 発行元あり（型が独自のため ROS 側から直接は読まない方針） |
| 頭部カメラ D435i | ⚠️ 接続後は認識される（6ノード）。ただし**取得できるのはIRと深度のみでカラーは取れていない**。Mapping が「color」として記録していた画像も実はIRだった |
| SDK2 高レベル API | ✅ 別途確認済み（`g1_loco_client --get_fsm_id` で応答。`fsm_id=0`＝脱力状態）|

## 5. 再現方法（点群の取得と解析）

点群の取得（購読のみ。ロボットは動かない）:

```bash
docker run --rm --network host --ipc host \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=0 \
  -e CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp3s0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>' \
  -v /path/to/outdir:/work g1-mapping-visualization:local \
  bash -c 'source /opt/ros/humble/setup.bash && python3 /work/grab_live_cloud.py 10 /work/live_cloud.npy'
```

スクリプト: [../tools/grab_live_cloud.py](../tools/grab_live_cloud.py)

⚠️ `sensor_msgs_py.point_cloud2.read_points_numpy()` は使えない。この点群は
フィールドの型が混在（x/y/z=float32、intensity/tag/line=uint8 等）しており
`All fields need to have the same datatype` で失敗する。`read_points()`（構造化配列）で
受けて x/y/z を取り出すこと。

## 6. 既存地図（room_a）への自己位置合わせ試験 — **結論: いまの場所では合わない**

Phase 2a の成立性を、ロボットを**動かさずに**確かめた（2026-09-09、座位・静止）。

### 6.1 手順

1. 生 LiDAR を 60 スキャン（1,203,456点）と IMU を 1,194 サンプル取得
   （[../tools/grab_cloud_and_imu.py](../tools/grab_cloud_and_imu.py)）
2. **IMU の重力ベクトルで水平化**した。座位での重力は `(+0.0918, -0.0443, -0.9948)`、
   ノルム 0.9999、標準偏差 0.002〜0.006（完全に静止）。**-Z 軸からの傾き 5.85°**
   （参考: 脱力状態では約 30° 傾いていた。姿勢で大きく変わるので、
   Mapping 班の固定較正値ではなく**その場の IMU から求める方が確実**）
3. 水平化後の床は `z=-1.751m` ＝ **センサー高さ 1.75m**（座位・安全支持具に載った状態）
4. 床上 1.2〜2.6m の帯で切り出し、地図（`map_20260907.pcd`）と照合

### 6.2 結果

| | 一致率 | ピーク比（最良/中央）| 判定 |
|---|---|---|---|
| **実データ（いまの場所）** | **0.204** | **1.23 倍** | ❌ ピークが立たない＝曖昧 |
| 対照実験（地図内の既知位置から作った疑似 live）| 0.738 | 3.44 倍 | ✅ 誤差 2cm / 1° で復元 |

対照実験は、地図内の点 `(5.0, -5.0)` から見える部分だけを遮蔽込みで抜き出し、
既知の yaw 37° を与えて復元させたもの。**照合器は正しく動いており、
成功時は一致率 0.7 以上・ピーク比 3 倍以上という明確な signature が出る。**

### 6.3 ⚠️⚠️ 上記の結論は**誤りだった**（2026-09-09 に利用者の指摘で判明）

当初ここに「実データはこれと決定的に異なるため、**現在の場所は room_a ではない**」
「room_a の地図で検証するにはロボットを物理的に room_a へ移す必要がある」と書いたが、
**どちらも誤り。** 利用者によると **ロボットは map 座標の原点（青い星＝地図作成の開始地点）
付近に居る。** つまり場所は合っており、**照合の手法が悪かった。**

**なぜ失敗したのか（実測で判明した原因）**

MID-360 の視野は **-7°〜+52°** で、**構造的に上向きに偏っている**。頭部搭載で静止すると
見えるものの大半が「近い天井」と「自機」になる:

| | 点数 | 割合 | 水平距離の中央値 |
|---|---|---|---|
| センサーより上（天井側）| 477,050 | 61% | **1.65 m（近い）** |
| センサーより下（床側）| 21,085 | 2.7% | 16.52 m（遠い） |
| 至近の自機写り込み（r<0.3m）| 420,531 | — | — |

**天井には位置を特定できる水平構造がほとんど無い。** したがって
「静止した1視点の点群を間取り地図に合わせる」というアプローチ自体が原理的に弱い。
対照実験（§6.2）が成功したのは、地図の点をそのまま使ったため天井偏重にならず、
壁の構造がそのまま入っていたからで、**対照が現実より易しすぎた。**

センサー高さを未知として総当たり（0.6〜2.6m）で再探索しても、最良で
一致率 0.387 / ピーク比 1.58 倍にとどまり、成功の signature には届かなかった。

**したがって**: 静止しての自己位置合わせは諦め、**歩かせて点群を蓄積する**か、
**内蔵 SLAM の odometry に任せる**（§8）のが正しい。原点付近に居るのであれば
`map→odom` ≒ 恒等変換として Nav2 を動かす前提が成り立つ。

### 6.4 副産物: 天井から独立に求めたセンサー高さ

live の天井ピーク（センサー基準 +1.22m）と、地図側の床-天井（2.77m）から:

    センサー高さ = 2.77 - 1.22 = **1.55 m**

これは §2.1.1 で「このデータから確定できない」とした値を、別経路から求めたもの。
ただし**姿勢に依存する**ので U-09 の代わりにはならない（下記 6.5）。

### 6.5 ⚠️ 姿勢は勝手に変わる（自動校正が脆い理由）

同じ「座位」のまま、IMU で測った傾きが時間とともに変わった:

| 時点 | -Z 軸からの傾き |
|---|---|
| 座位に直した直後 | 5.9° |
| 地図照合に使った点群の取得時 | 約 4°（ほぼ水平） |
| その約30分後 | **33.16°**（頭部が上を向いた） |

照合に使った点群は「ほぼ水平」の状態のもの（重力を同時刻に測っており、取得中の
標準偏差は 0.001〜0.006 と安定）なので、**照合失敗の原因は姿勢ではない。**

しかし **起動時に1回だけ姿勢を測る自動校正（§9.2 の `--auto-level`）は脆い**ことが
これで分かる。**U-09 の取付角実測（剛体の固定変換）が必要**という結論は変わらない。

### 6.4 副産物: 既存ツール `align_to_map.py` の限界（申し送り）

`Mapping/real/quickstart/align_to_map.py` をそのまま使うと**失敗する**。2 つの原因を実測で特定した。

1. **床の推定が 5 パーセンタイル**（`FLOOR_PERCENTILE = 5.0`）。頭部 LiDAR は
   足元の床が死角なので床の点がほとんど無く、live 側の床を `-0.565m` と誤推定した
   （実際は `-1.75m`）。結果、高さ帯が約 1.2m ずれ、「床上 0.3〜1.8m」の帯に
   **全点の 93% が残る**（帯として機能していない）状態になった
2. **FPFH + RANSAC が 6 自由度で探索するため、上下反転（roll≈180°）の偽解に落ちる。**
   帯を直した後でも `roll=-176.8°`, `z=+3.8m` という物理的にありえない解を返した。
   **両方の点群が既に重力整列済みなら、探索は (x, y, yaw) の 3 自由度に限定すべき**

3 自由度に限定した照合器を [../tools/match_scan_to_map_2d.py](../tools/match_scan_to_map_2d.py)
に置いた（FFT 相関で全平行移動を一括評価し、yaw を走査する）。反転解が原理的に出ず、
**曖昧さを数値（ピーク比）で報告するので合否判定ができる**。

### 6.5 副産物: 構造帯の選び方は**センサー高さ**に依存する

センサーが床から 1.75m、下方視野が -7°（座位では約 5° 下向きなので実効 -12° 程度）だと、
**「床上 0.3〜1.8m」の帯は約 8m 以内では原理的に観測できない**（下を見られないため）。
実測でもこの帯は live 673,512 点中 77,223 点しか無かった。
床上 1.2〜2.6m にすると 353,004 点になり、地図側も壁の輪郭が明瞭に残る。

**Nav2 の costmap の高さ帯（D-21）や A-7 の変換ツールの既定値も、
この「センサー高さより下は近距離で見えない」性質を踏まえて決める必要がある。**

## 7. U-15: G1 内蔵の再定位サービスは使えるか — **結論: 我々の地図には使えない**

読み取りのみで確認した（2026-09-09）。**API は一切送信していない。**

### 7.1 slam_operate の API-ID（`Navigation/nav/protocol.py` より）

| API-ID | 定数 | 意味 | 機体は動くか |
|---|---|---|---|
| 1801 | `API_START_MAPPING` | 建図開始（**今いる場所を原点**に新規地図）| 動かない |
| 1802 | `API_END_MAPPING` | 建図終了・保存（**PC1 のファイルシステム**に書く）| 動かない |
| 1804 | `API_INIT_POSE` | 初期位姿（地図読込＋自己位置設定）| 動かない |
| **1102** | `API_NAVIGATE_POSE` | **位姿導航** | ⚠️ **動く。投げないこと** |
| 1201 / 1202 | `API_PAUSE` / `API_RESUME` | 一時停止 / 再開 | 1202 は停止中の移動を再開させうる |
| 1901 | `API_CLOSE_SLAM` | SLAM 終了 | 動かない |

送信経路は PC2 上の `unitree_sdk2py` の RPC クライアント（`Client("slam_operate")`）。

### 7.2 現在の状態（実測、読み取りのみ）

`/slam_info`（`std_msgs/String`、約5.5Hz）の `ctrl_info`:

```json
"stateMachine": { "ctrName": "not init", "state": "ready", "isPause": false, ... }
"info": "not init",  "errorCode": 0,  "total_distance": -1.0
```

`ctrName: "not init"` は `nav/protocol.py` の `NOT_INIT_MARKER`（「地図を読み込んでいない」の
実測上の目印）と一致する。つまり **SLAM サービスは起動していて状態を配信しているが、
地図が未読込なので `/unitree/slam_*` の odom / points にはデータが出ない**。

併せて `robot_data`（約0.5Hz）から PC1 の状態も取れる:
battery 83% / 52.5V、CPU 39% / 58℃、`sportMode=-1`、`gaitType=-1`。

### 7.3 ⚠️ 決定的な制約: 外部で作った地図は読ませられない

`Navigation/README.md` に**実機での試行結果が既に記録されていた**
（`nav/transport.py` のコメントは「未検証」と書いてあるが、README の方が新しい）:

- どんな `address` を渡しても `errorCode 507 "Load pcd failed."`。
  **PC2 上に置いた実在の PCD でも失敗する**（応答 0.01 秒＝定位に入る前に落ちている）
- SLAM は PC1（`192.168.123.161`）で動いており、`address` は **PC1 のファイルシステム**を指す
- PC1 へ地図を転送する手段が無い（SSH 不可・共有マウント無し・PC2 に SLAM バイナリ無し）
- 想定フローは **1802 が PC1 に保存 → 1804 が同じファイルを読む**（ロボット内で完結）

**したがって U-15 の当初の期待「内蔵再定位で FAST-LIO を置き換える」は、
我々の地図（`map_20260907.pcd`）については成立しない。** 動的物体を除去し、
床基準の高さ補正とレイトレーシングを施した地図は、内蔵再定位には**原理的に渡せない**。

### 7.4 それでも残る価値: 内蔵 SLAM の **odometry**

`/unitree/slam_mapping/odom` は `nav_msgs/Odometry`（標準 ROS 型）である。これが
1801 で流れ始めるなら、**`odom → base_link` の供給源として使え、FAST-LIO を
自前で構築・運用する必要が無くなる**。`map → odom` は我々の地図への ICP 一発合わせで
与える（姉妹トラック `Navigation/real/` が既に採っている方式）。

**未確認（1801 の送信が必要なため保留）**: 1801 で実際に odom が流れるか、
その配信レート・座標系・静止時のドリフト。

### 7.5 その他の実測メモ

- 全ての Unitree トピックは **RELIABLE + VOLATILE**、発行元は
  `_CREATED_BY_BARE_DDS_APP_`（ROS 2 ノードではなく生の CycloneDDS。D-02 の前提と整合）。
  **latch されていないので、DDS から過去の地図を回収することはできない**
  （`transient_local` で購読すると DURABILITY 非互換で 1 件も受け取れない）
- ⚠️ **`Navigation/README.md` の「PC1 のTCPポート: 主要18ポート全て閉」は不正確。
  実測で `9991` が開いていた**（他の 14 ポートは閉）。何のサービスかは未調査。
  運控PCの未知サービスを叩くのは読み取り調査の範囲を超えるため触っていない

## 8. U-15 続き: 1801 を送って内蔵 SLAM の odometry を実測（2026-09-09）

利用者の許可を得て、**機体を動かさない API だけ**を送って確認した。
送信スクリプト: [../tools/send_slam_api.py](../tools/send_slam_api.py)
（`unitree_sdk2py` の RPC クライアントは `_RegistApi()` した api-id しか送れない性質を使い、
**1801/1901 のみを登録して 1102(移動) を構造的に送れなくしてある**。
`1102` を引数に渡しても許可リストで弾かれることを実行して確認済み）

### 8.1 送受信の実測（**プロジェクト初の slam_operate 成功パス**）

```
[send] api-id=1801 (START_MAPPING) request={"data": {"slam_type": "indoor"}}
[recv] RPC code=0
[recv] {"succeed":true,"errorCode":0,"info":"Successfully started mapping.","data":{}}

[send] api-id=1901 (CLOSE_SLAM) request={"data": {}}
[recv] RPC code=0
[recv] {"succeed":true,"errorCode":0,"info":"Stop slam success.","data":{}}
```

`nav/transport.py` は「未検証。1802/1804/1102 の成功パスを一度も通していない」と
記録していたが、**1801 と 1901 は成功する**ことが分かった。
**座位のままで通った**（直立は不要）。

### 8.2 ⭐ `/unitree/slam_mapping/odom` の実測（70秒・静止）

| 項目 | 実測値 |
|---|---|
| 配信レート | **9.10 Hz** |
| `header.frame_id` | **`map`** |
| `child_frame_id` | **`base_link`** |
| 静止時のドリフト | **最大 0.9 cm / 70秒後 0.4 cm** |

`/unitree/slam_mapping/points` も同時に 9.10 Hz、1メッセージ約 1,198 点（`frame_id='map'`）。

**これは Phase 2a の作業量を大きく減らす。** 標準の `nav_msgs/Odometry` が
そのまま出るので、**FAST-LIO を自前で構築・運用する必要が無い**
（A-6 で vendoring した `FAST_LIO_LOCALIZATION_HUMANOID`、Open3D のバージョン固定、
フレーム名パッチ、`open3d_loc` の SIGSEGV 修正といった一連の作業が不要になる）。

⚠️ **ただしフレーム名の意味に注意。** 名乗っているのは `map → base_link` だが、
**原点は 1801 を送った時点のロボット位置**であり、意味的には odometry である。
Nav2 に載せるなら:

- これを **`odom → base_link` として扱う**（改名する）
- `map → odom` は我々の処理済み地図への ICP 一発合わせで与える
  （姉妹トラック `Navigation/real/` が既に採っている方式。§6 の照合器が使える）

⚠️ **TF は出ない。** `/tf` も `/tf_static` も存在しない（実測）。odom トピックから
自分で TF を配信する必要がある（Mapping 側の `odom_to_tf.py` がやっていること）。

### 8.3 併せて分かったこと

- **`ctrl_info` の `ctrName`/`info` は建図中も `"not init"` のままだった。**
  つまりこのマーカーは「**1804 でナビ用の地図を読み込んだか**」を示すもので、
  建図セッションの有無とは無関係。`nav/protocol.py` の `initialized` 判定を
  「SLAM が動いているか」の意味で使うと誤る
- `/safe_clouds` / `/planner_map` は建図中も流れない。これらは**ナビ(1102)側**の
  機能に属する
- **1901 で完全に元に戻る**: odom/points が 0 Hz になり、状態も `ready` / `not init` に復帰。
  残留状態は観測されなかった

### 8.4 U-15 の結論

| 問い | 答え |
|---|---|
| 内蔵**再定位**で我々の地図を使えるか | ⚠️ **未確定**（「不可」と断定できない。§8.5 参照） |
| 内蔵 SLAM の **odometry** は使えるか | ✅ **使える**。9.1Hz・静止ドリフト 1cm 未満・標準 ROS 型 |
| FAST-LIO は必要か | **odometry 目的では不要**。`map→odom` は ICP 合わせで与える |

**未確認（動かす必要がある）**: **歩行中**のドリフト・二足の上下動への耐性。
静止時のドリフトが小さいことは易しい方の条件であり、Phase 2a の本番評価は
歩行させてから行う。

### 8.5 ⚠️ 「我々の地図は使えない」は**未確定**である（自己訂正）

§7.3 で「外部で作った地図は読ませられない」と断定的に書いたが、**根拠が弱い。**
正確に切り分けると:

| 事実 | 出所 |
|---|---|
| 1801 / 1901 は成功する | **本調査の実測（2026-09-09）** |
| 1804 が全 address で `errorCode 507` を返す | `Navigation/README.md` の過去の記録。**nav2_option 側では 1804 を一度も送っていない** |
| PC1 へ地図を転送する手段が無い | 同上。**ただしその根拠の一部（「主要18ポート全て閉」）は本調査で不正確と判明（`9991` が開いている）** |

**未調査の経路**:

1. **PC1 の `9991` 番ポート**が何のサービスか（運控PCの未知サービスなので未着手）
2. Unitree 公式アプリに地図管理機能があれば、そこが転送経路になりうる
3. DDS に `/unitree_slam/waypoints` がある → 地図やウェイポイントを**送り込む**側の
   チャネルが存在する可能性
4. PC2 側のゲートウェイ的プロセス（`key_server` / `master_service` / `ota_pipe` /
   `video_hub`）が PC1 へのファイル操作を仲介していないか

### 8.6 転送できなくても成立するハイブリッド構成

仮に転送手段が本当に無くても、次の分担で成立する:

| 役割 | 使うもの | PC1 への転送 |
|---|---|---|
| 自己位置（`odom→base_link`）| 内蔵 SLAM の `/unitree/slam_mapping/odom`（§8.2）| 不要 |
| 自己位置の地図基準化（`map→odom`）| 処理済み地図への ICP 一発合わせ（§6 の照合器）| 不要 |
| Nav2 の global costmap | **処理済み地図 `room_a_map.yaml`**（A-7 で生成）| 不要 |

つまり **`1804` を使わない限り、地図を PC1 へ渡す必要はそもそも無い。**
`1804` を使いたくなるのは「純正の再定位でドリフトを補正したい」場合だけで、
その際はロボット自身に `1801`→`1802` で地図を作らせ、**処理済み地図との間の
変換を ICP で1回求める**という形で両立できる。

## 9. 実機データで Nav2 の配線を通した（足は繋がない、2026-09-09）

Planning.md の Phase 2c に相当する経路を、**実機のセンサーで**通した。
`RealMoveBackend` には繋いでいないので**機体は動かない**。

```
1801 → 内蔵SLAM odom(9.1Hz)
      → g1_slam_odom_tf(TF配線・自動校正)  → TF odom→base_link, /odom
      → map_server(room_a_map.yaml)         → global costmap
      + /utlidar/cloud_livox_mid360(実機)   → local costmap(voxel_layer)
      → planner_server → controller_server  → /cmd_vel
      → velocity_smoother                   → /cmd_vel_smoothed
```

### 9.1 結果

| 項目 | 結果 |
|---|---|
| 全ライフサイクルノードの activate | ✅ map_server / planner / controller / behavior / smoother / bt_navigator すべて `active [3]` |
| local costmap（**実機LiDAR由来**） | ✅ 1.677 Hz で配信 |
| `NavigateToPose` のゴール受理 | ✅ |
| **`/cmd_vel`** | ✅ **214 件（うち非ゼロ 210 件、`wz = 0.300 rad/s`、`vx = 0`）** |
| `/cmd_vel_smoothed` | ✅ 499 件 |

`vx=0` / `wz=0.3` で 40 秒間続いたのは、RegulatedPurePursuitController の
**rotate-to-heading**（まず目標方向へ向き直る）フェーズに留まっていたため。
足を繋いでいないのでロボットが実際に回らず、odom も変わらないので、
Nav2 は回転を出し続ける。**足未接続の試験としては期待どおりの挙動。**

📌 **副産物（D-14 / U-12 への示唆）**: Planning.md A-9 では、疑似データでの
rotate-to-heading が **0.02 rad/s** しか出ず D-14 のデッドバンド（`min_wz=0.03`）に
食われて動き出せない問題が起きた。**実機構成では 0.300 rad/s 出ている**ので、
その「にらみ合い」は起きにくい。ただし歩容が成立する下限速度（U-12）は
実際に歩かせて測るまで確定しない。

### 9.2 ⚠️ 内蔵 SLAM の姿勢は重力整列されていない（重要）

TF を配線する過程で判明した。**そのまま Nav2 に渡すと costmap が傾く。**

静止した座位で SLAM は **pitch = -7.469° ± 0.027°**（20秒・200サンプル、位置ドリフト
0.67cm）を安定して報告する。一方、同時刻の IMU が示すセンサー自身の傾きは約 3.9°。
つまり SLAM は**ロボット胴体の姿勢**を出しており、頭部センサーの傾きとは別物である。

重力ベクトルを TF で変換して測った、経路ごとの -Z 軸からのずれ:

| 経路 | 補正前 | **自動校正後** |
|---|---|---|
| 生の `livox_frame` | 3.9° | 3.9°（変わらない） |
| `map ← base_link` | 11.29° | 11.20° |
| **`map ← livox_frame`（costmap が使う経路）** | **6.10°（悪化）** | **0.06°** ✅ |

対処: [../tools/g1_slam_odom_tf.py](../tools/g1_slam_odom_tf.py) の `--auto-level`（既定 ON）が
起動時に IMU の重力と SLAM の姿勢を同時に採り、

    R(base_link←livox) = R(map←base_link)ᵀ · R_level

を計算して静的変換にする。合成後の残差は実測 **0.000°**。

⚠️ **この自動校正は「起動時の姿勢＝運用中の姿勢」でしか正しくない。** 本来
`base_link→livox_frame` は剛体の固定変換であり、歩行中は胴体姿勢が振動する。
**取付角の実測値（U-09）で置き換えるべき**で、現状は静止した配線試験のための近似。

### 9.3 この試験の限界

`map→odom` は恒等変換のまま置いた。**確認できたのは「配線が通ること」だけ**で、
経路の妥当性・到達判定は検証していない。

⚠️ **訂正（2026-09-09）**: 当初ここに「現在地は room_a ではないので global costmap と
local costmap は別の場所を表している」と書いたが、**誤り。** 利用者によると
**ロボットは map 座標の原点付近に居る**（§6.3 の訂正参照）。つまり
`map→odom` ≒ 恒等変換は**位置としてはおおむね妥当**であり、
やり直せば幾何的にも意味のある試験にできる。

**やり直す場合の条件**: 頭部の姿勢が水平に近いこと（§6.5 のとおり姿勢は変動する。
33° 上を向いた状態では local costmap が天井を拾って使えない）。
向き（yaw）が地図作成時と一致しているかは未確認なので、そこは別途合わせる必要がある。

### 9.4 実行方法

```bash
# 1801 で内蔵SLAMを起動してから
docker run -d --name g1-nav2-live --network host --ipc host \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=0 \
  -e CYCLONEDDS_URI='...enp3s0...' -v <scratch>:/work \
  g1-mapping-visualization:local bash -c 'bash /work/run_nav2_live.sh; sleep infinity'
```

設定: [../tools/nav2_live_wiring.yaml](../tools/nav2_live_wiring.yaml)
（**Humble 用の配線試験専用**。本番は Jazzy 向けの
`g1_ws/src/g1_navigation/config/nav2_params.yaml`）

⚠️ `set -u` を使うと ROS の `setup.bash` が
`AMENT_TRACE_SETUP_FILES: unbound variable` で落ちる（実際に踏んだ）。
