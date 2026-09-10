# README-nav2 — 事前地図の上で自己位置を出し、Nav2 で歩かせる

`README.md`（このフォルダのもう 1 本）は **建図**の話である
（LiDAR を録って地図にして GUI で見る）。**こちらは測位と Nav2 の話。**
スクリプトが 15 本あるのに入口が無く、使い方が `docs/plan/` の計画書 5 本と
`docs/作業ログ/` の HTML にしか無かったので分けた（2026-09-07）。

> **計画書を手順書として読まないこと。** あれは「その時点で何を考えていたか」の記録で、
> **通らないコマンドや後で撤回した前提が残してある**。手を動かすときはこの README を見る。

---

## 0. 30 秒で分かる現状

| | 状態 |
|---|---|
| 事前地図の上での自己位置 | **MOLA-LO で出る。**実機で ICP 品質 0.850・10 Hz・漂流 0.7 mm |
| 姿勢の精度 | 歩行中 中央値 **1.49°** / p95 3.97°（直す前は 10.53° / 17.82°） |
| 重畳 | 実機の**立脚静止で 89.2 %**（±10 cm で 95.4 %）。**歩行中は未測定** |
| 純正 SLAM（1801） | **要らない。**MOLA-LO 単体で `map → base_link` が出る |
| Nav2 で歩く | **歩いた（2026-09-09）。**1.79m のゴールに残り 0.283m まで（許容 0.30m）。転倒 0。
  ただし**「到達」判定は出ていない**（ゴールの向きの指定が誤っていた。§7 の落とし穴）|

**直っていない前提**: 内蔵 SLAM の odom は**歩行中に roll/pitch が中央値 10.5° 狂う**。
これは PC1 の中で作られていて手が届かない。**だから測位を外で作り直している。**

---

## 1. 何が何をするか

### 立てるもの（Mac 側）

| スクリプト | 役割 |
|---|---|
| `_common.sh` | **source するもの。**DDS の URI と IP/名前の**唯一の定義箇所**。`g1_exec` / `g1_tcp_ok` などのヘルパ |
| `check_link.sh` | **経路の確認。**Mac の有線 → VM → コンテナ → 実機 → DDS を**切れる順に**上から見る |
| `start_rviz_mac.sh` | コンテナを起こし、必要な apt を**冪等に**入れ、RViz2 を起動する |
| `nav_stack.sh` | 再生 or 実機 ＋ TF ＋ MOLA-LO ＋ OctoMap ＋ `map_server` ＋ Nav2 |
| `preflight.sh` | **歩かせる前の単一のゲート。**測位・base_link の向き・コストマップ・Nav2・足の接続を見る |
| `run_stage.sh` | 1 本走らせ、**走行中の bag を録って重畳まで出す**（順番を焼き込んである） |
| `stray_guard.py` | **逸脱と時間の見張り。**投げ方に関係なく効くので RViz のクリックも守れる |
| `diag_turn.py` | RPP の追従点と指令 vyaw の突き合わせ。`base_link` の向きの狂いを見つける |

### 地図と測位

| スクリプト | 役割 |
|---|---|
| `run_mola_lo.sh` | **記録から MOLA-LO で地図と軌跡を作る**（オフライン。ROS を立てない） |
| `eval_mola_traj.py` | 軌跡を真値と突き合わせて**合否を印字する** |
| `log_tf_pose.py` | ROS 上の `/tf` を**スキャン時刻で**引いて TUM 形式に落とす |
| `measure_overlay.py` | **重畳を数値にする**（ライブ点群が事前地図の占有セルに乗った割合） |
| `pcd_to_occupancy.py` | 点群 → `nav_map.pgm`（Nav2 の `static_layer` 用の 2D 占有格子） |
| `odom_to_tf.py` | 内蔵 SLAM の odom を `/tf` に流す。**MOLA-LO 構成では要らない**（§4） |

### もう使わないもの

| | なぜ |
|---|---|
| `align_to_map.py` ＋ `static_transform_publisher` | 位置合わせは**地図を作るときの初期姿勢**に 1 回入れるようにした（§3）。SLAM 再起動のたびのやり直しが消える |
| `capture_slam_cloud.py` | 上と同じ流れの一部 |
| `watch_mapping.py` | **純正 SLAM を起こさない**構成にしたので出番が無い。純正建図をするときだけ要る |

---

## 2. まず経路を確かめる（つまずきの 9 割はここ）

```bash
bash quickstart/check_link.sh            # 実機まで
bash quickstart/check_link.sh offline    # 実機を飛ばす
```

### ⚠️ `ping` を使ってはいけない

**`ping` は colima の VM にもコンテナにも入っていない。**
`ping ... && echo OK || echo NG` と書くと
**「到達しない」と「コマンドが無い」が同じ NG になって区別できない。**
2026-09-07 にこれで 2 回誤診した。`check_link.sh` は bash 組み込みの
`/dev/tcp` で TCP の接続可否を測る。

### ⚠️ `ros2 topic list` は嘘をつく

`ros2 daemon` は**先に起動したときの DDS 設定で作ったグラフをキャッシュ**している。
`CYCLONEDDS_URI` を変えても古い答えを返すので、`offline` なのに実機のトピックが
184 件見えた（2026-09-07）。**`--no-daemon` を付ける。**

### 切れる順番

```
Mac (en8 / 192.168.123.202)            ← USB アダプタ。抜けると OS から消える
  └─ colima VM (col0 / .201)           ← ケーブル抜き差しでここが切れる
       └─ コンテナ rviz (--network host)
            └─ G1 の内蔵スイッチ (L2)
                 ├─ PC1 .161  ポート全閉。LiDAR と SLAM はここが出す
                 └─ PC2 .164  ssh で入れる。loco_driver はここ
```

### ⚠️ ケーブルを抜き差しするとブリッジが切れる（**もう手作業では直さない**）

`col0` は socket_vmnet 経由で G1 の内蔵スイッチに L2 で載っている。
**ケーブルを抜き差しするとこの結びつきが外れる。** 紛らわしいのは

- **Mac → PC2 は通ったまま**（ホストの経路は別なので気づかない）
- `col0` には IPv4 が**付いたまま**
- `ros2 topic list` は daemon が握った**古い結果**を返すので「見えている」と錯覚する

の 3 つで、実データ（`ros2 topic echo --once`）が 1 件も来ないところまで行かないと
気づけない。

**`start_rviz_mac.sh live` が起動時に自分で検査して直す。**
コンテナ（無ければ VM）から PC2 の `22` へ TCP を張って測り、切れていたら
`colima stop` してからブリッジ付きで起動し直す。**どちらの経路で測ったかは標準出力に出る。**
張り直した後はコンテナの `ros2 daemon` も落とす（切れている間に握った古いグラフが
残るため。「2 件しか見えない」→ `daemon stop` で 139 件、という実測がある）。

⚠️ **`colima restart` 単体では直らないことがある**（`col0` に IPv4 が付かないまま
上がる型がある）。`start_rviz_mac.sh` は自分で `ip addr add` するので、
**手で restart せず `start_rviz_mac.sh live` を通すこと。**

### 環境変数 `G1_COLIMA_RESTART`（ブリッジの張り直し方）

| 値 | 挙動 |
|---|---|
| `auto`（**既定**） | colima が動いていればブリッジを検査し、**壊れている時だけ** `colima stop` する |
| `always` | 検査せず無条件に `colima stop` してから起動する |
| `never` | 検査も stop もしない（2026-09-09 までの挙動） |

不正な値は起動時に弾く（`die`）。`offline` では `col0` が要らないので、
**どの値でも検査も stop もしない。**

⚠️ **無条件の stop（`always`）は VM の再起動に約 2 分かかる**
（実測 08:02:26 stop → 08:04:49 起動完了）。当日は `auto` のまま使う。

---

## 3. 記録から地図を作る（実機不要）

```bash
docker exec rviz bash /work/G1_Hackason/Mapping/real/quickstart/run_mola_lo.sh <SESSION>
```

出力は `runs/<SESSION>/mola/` に出る。

### ⚠️ どの `.mm` が測位に使えるか

| ファイル | 作り方 | 層 | 測位 |
|---|---|---|---|
| `map.mm` | `MOLA_SAVE_MM` | `localmap`（`HashedVoxelPointCloud`） | **使える**（実機で品質 0.85） |
| `map_full.mm` | `sm2mm` | `raw`（`CGenericPointsMap`） | **使えない**（ICP 品質 0.00 に張り付く） |

パイプラインの ICP は `localmap` 層を見るが、`sm2mm` の既定生成器は `raw` しか作らない。
**名前に騙されないこと**（`map_full` の方が良さそうに見えるが使えない）。

`.mm` を `mm-info` で覗くときは **`--load-plugins libmola_metric_maps.so` が要る**。
無いと中身があっても「empty」と出る。

### 地図を Nav2 の座標系に載せる

MOLA の `.mm` は既定では**MOLA 自前の座標系**（最初のスキャンの LiDAR 姿勢）になる。
この機体は **LiDAR が逆さま取付なので roll が 178°** で、`nav_map.pgm` と噛み合わない。
**地図を作るときに初期姿勢を渡す。**

```bash
# 初期姿勢 = 真値（benchmark_s5/poses.txt）の 1 行目 = SLAM 地図座標系での LiDAR 姿勢
docker exec \
  -e MOLA_OUT_NAME=mola_aligned \
  -e MOLA_INITIAL_X=-0.0168 -e MOLA_INITIAL_Y=0.0264 -e MOLA_INITIAL_Z=-0.0437 \
  -e MOLA_INITIAL_YAW=0.239 -e MOLA_INITIAL_PITCH=3.322 -e MOLA_INITIAL_ROLL=177.927 \
  -e MOLA_SAVE_MM=/work/.../runs/<SESSION>/mola_aligned/map.mm \
  rviz bash /work/G1_Hackason/Mapping/real/quickstart/run_mola_lo.sh <SESSION>
```

うまく載ったかは `eval_mola_traj.py` の「真値座標系への合わせ」が**恒等に近いか**で分かる
（載っていれば rpy 0.35 / −0.05 / −0.10°・0.7〜2 cm 程度）。

### 合否を出す

```bash
Navigation/.venv/bin/python quickstart/eval_mola_traj.py \
    runs/<SESSION>/mola/traj.txt runs/<SESSION>/benchmark_s5/poses.txt
```

基準（**測る前に決めたもの。後から動かさない**）:
姿勢 中央値 ≤2° / p95 ≤5° / 位置 ≤5 cm / ≥5 Hz / 5 m 先 ≤0.5 m。
**対抗馬は「odom の yaw ＋ IMU」の 1.90° / 4.33° / 0.377 m。これに負けたら使う意味が無い。**

---

## 4. 実機で測位を回す

**この引数の組み合わせで実機で動いた**（2026-09-07・立脚静止）。

```bash
bash quickstart/check_link.sh                                  # 先に経路
G1_RVIZ_CFG=$PWD/quickstart/rviz/g1_nav.rviz \
  bash quickstart/start_rviz_mac.sh live

RUN=/work/G1_Hackason/Mapping/real/runs/<SESSION>
docker exec -d -u ubuntu \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI="$(bash -c '. quickstart/_common.sh; g1_dds_uri live')" \
  -e ROS_DOMAIN_ID=0 rviz bash -c "
source /opt/ros/humble/setup.bash
ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
  mola_lo_pipeline:=/work/G1_Hackason/Mapping/real/quickstart/mola/g1_lidar3d_icp_imu.yaml \
  lidar_topic_name:=/utlidar/cloud_livox_mid360 \
  imu_topic_name:=/utlidar/imu_livox_mid360 \
  use_imu_for_lio:=True min_nearby_poses_occupied:=2 \
  ignore_lidar_pose_from_tf:=true ignore_imu_pose_from_tf:=true \
  start_mapping_enabled:=False \
  mola_initial_map_mm_file:=$RUN/mola_aligned/map.mm \
  initial_localization_method:=InitLocalization::FixedPose \
  initial_pose:=\"[-0.0168, 0.0264, -0.0437, 0.239, 3.322, 177.927]\" \
  publish_localization_following_rep105:=False \
  use_mola_gui:=False use_rviz:=False use_sim_time:=false > /home/ubuntu/mola_loc.log 2>&1"
```

**確認**: `/lidar_odometry/pose_quality` が **0.8 前後**、`/tf` が **10 Hz**。
品質が 0.00 なら地図の層が違う（§3 の表）。

### ⚠️ 踏んではいけない地雷

| やってはいけないこと | 何が起きるか |
|---|---|
| `mola_tf_base_link:=livox_frame` を渡す | **SIGSEGV（exit -11）**。既定の `base_link` ＋ `ignore_*_pose_from_tf:=true` にする |
| `map_full.mm` を読ませる | ICP 品質 0.00 で張り付く |
| `initial_pose` を省く | roll が 178° 違うので収束しない |
| GICP パイプライン（`lidar3d-default.yaml`）を使う | `RKNN search requires nanoflann>=1.5.1` で最初のスキャンで落ちる。**arm64 の `mp2p_icp` は nanoflann 1.4.2 でビルドされており、ヘッダの `#if` で焼き込まれているので新しい nanoflann を入れても直らない**。`mola/g1_lidar3d_icp_imu.yaml` を使う |
| `run_mola_lo.sh` に `set -u` を足す | ROS の `setup.bash` が `AMENT_TRACE_SETUP_FILES` を触るので即死する |

### 純正 SLAM は起こさなくてよい

MOLA-LO 単体で `map → base_link` が出る（実機で確認）。
純正 SLAM は**立たせる操作で黙って止まる**ので `watch_mapping.py` の常駐が要ったが、
**その必要が消えた。**

---

## 5. 重畳を測る（目で見ない）

RViz2 のスクリーンショットは**カメラ角度と拡大率で印象が変わる**。数えて数字にする。

```bash
# 1. 録る（歩かせながら録るのが本番。静止中の値は甘い）
docker exec -u ubuntu rviz bash -c \
  'source /opt/ros/humble/setup.bash && cd /work/G1_Hackason/Mapping/real/runs &&
   timeout 30 ros2 bag record -o walk_bag /utlidar/cloud_livox_mid360 /tf /tf_static'

# 2. 測る（コンテナに numpy が無いので Mac 側の venv で）
Navigation/.venv/bin/python quickstart/measure_overlay.py \
    runs/walk_bag runs/<SESSION>/map/nav_map.yaml
```

**判定**: 占有セルに乗った割合。立脚静止で **89.2 %**（±10 cm で 95.4 %）。
**歩行中もこの水準を保てば合格。**

> ⚠️ **静止中の値では判定にならない。** ずれは歩行中にしか出ない
> （静止中の姿勢ばらつきは 0.39°、歩行中は直す前で 10.5°）。

---

## 6. Nav2 と繋ぐ（2026-09-09 に通した）

```bash
G1_USE_MOLA=1 bash quickstart/nav_stack.sh live
```

`live` かつ `G1_USE_MOLA=1` なら、下の 3 つは `nav_stack.sh` が自動で選ぶ。
**手で組み直さないこと**（3 つは対で動く）。

### (1) `odom` は恒等の静的変換で置く

MOLA-LO を `publish_localization_following_rep105:=False` で回すと
`map → base_link` を直接出すので `odom` が無い。`odom_to_tf.py --static-only` が
`map → odom` を恒等の静的変換として置き、TF が `odom → map → base_link` を辿る。

⚠️ **`rep105:=True` は使えない。** MOLA は rep105 の値に関わらず
`odom → base_link` と `map → odom` も出し続けるが、**それは MOLA 内部の時計**
（起動からの秒数）で、`map → base_link` だけが ROS の時計である。
だから REP-105 ペアは Nav2 からは使えない。

⚠️ さらに、既定のままだと **`odom` を MOLA（動的）と我々（静的）の 2 つが名乗る。**
tf2 はフレームを最初に受けた種類で確定するので、**どちらが先に届くかで動く時と動かない時がある。**
動的が先に立つと壁時計での参照が全部
`transformPoseInTargetFrame: Extrapolation Error` になり、
**controller が経路を変換できず `/cmd_vel` が 1 度も出ない。**
コストマップの中身は静的レイヤで埋まるので**「動いているように見える」。**
→ `local_costmap.global_frame` を **`map`** にして `odom` を引かせない（`g1_nav2.yaml`）。

### (2) `base_link` は「水平・床面」で定義し直す

⚠️⚠️ **`LIVOX_RPY_DEG` の pitch -8.41° は内蔵 odom の傾いた胴体系を基準に測った値。**
内蔵 odom を使わない live の構成ではその傾きを誰も補正しないので、
`map → base_link` が **pitch 12.2° / z 1.27m**（＝ LiDAR の位置）になる。
経路の点は map の z=0 に在るので、RPP が 3D で `base_link` 系へ変換すると
**全点が一様に約 +0.27m 前・約 -0.033m 右にずれ、機体は右へ逸れ続ける**
（RPP は変換の**後**に z を 0 にするので歪みが残る）。

live では `rpy 177.93 3.32 0 / xyz 0 0 1.228` を使う。地図 `mola_floor0` の datum から
合成が厳密に閉じる（検算 4.4e-16 m）。初期姿勢も対で
`[-0.0168, 0.0264, 0.0, 0.24, 0.0, 0.0]` になる。
**確認**: `tf2_echo map base_link` の z が 0、pitch が 0 付近。

### (3) `map_server` は自分で起こす

Nav2 の `global_costmap.static_layer` は `/map` を読むが、それを publish する
ノードが 2026-09-06 の構成に無く、静的レイヤが空のままだった。
`nav_stack.sh` に追加済み。ライフサイクルノードなので `lifecycle_bringup` も要る。

---

## 7. 脚を繋ぐ

**PC2 側の 2 プロセス構成。**`rclpy` と `unitree_sdk2py` が同居できないため
localhost の UDP（47600）で繋いでいる。**安全機構は「止められる側」＝ SDK 側にある。**

```
[Nav2] --/cmd_vel--> [cmd_vel_bridge.py (pixi 3.11)] --UDP--> [loco_driver.py (system 3.8)] --> 足
```

### 配る

```bash
bash Navigation/real/deploy_to_pc2.sh          # md5 で検証して配る（冪等）
bash Navigation/real/deploy_to_pc2.sh check    # 配らずに差分だけ
bash Navigation/real/deploy_to_pc2.sh status   # PC2 で何が動いているか
```

### 起こす（この順で）

```bash
# 1. ROS 側。⚠️ pixi run python では DDS の環境変数が入らず /cmd_vel が見えない
ssh g1 'bash ~/mapping_tools/start_cmd_vel_bridge.sh'

# 2. 素振り。⚠️ --dry-run 単体では何も表示されない（--arm の判定が先にあるため）
ssh g1 'python3 ~/nav_tools/loco_driver.py --dry-run --arm --max-vy 0'

# 3. 本番。**ここから足が動く。**前景で走らせて Ctrl+C を停止手段にする
ssh -t g1 'python3 ~/nav_tools/loco_driver.py --network-interface eth0 --max-vy 0 --arm'
```

既定のクランプ: `vx 0.30` / `vy 0.20` / `vyaw 0.50` / **無指令 0.5 秒で停止**。

⚠️ **`pkill -f loco_driver.py` を ssh の 1 行に書かないこと。**
パターンが**リモートコマンド行自身にマッチして自分のシェルを殺す**（ssh が exit 255 で落ちる）。
`loco_drive[r].py` と括り、**kill と起動は別の ssh 呼び出しに分ける**
（同じ行に `loco_driver.py` という literal が別の用途で入っていても当たる）。

### ⚠️⚠️ ゴールの向きを固定値で投げてはいけない（2026-09-09 に実機で踏んだ）

`check_navigation.py` は以前ゴール姿勢を `orientation.w = 1.0`（yaw 0）で投げていた。
**機体の向きともゴールへの方位とも無関係な値**なので、

1. 位置は達成しても `yaw_goal_tolerance`（0.35 rad）を永久に満たせない
   （実測: ゴールまで残り 0.283m < 許容 0.30m まで詰めたのに「到達」にならない）
2. その場で向きを直そうとする → **`SimpleProgressChecker` は並進しか数えない**ので
   「15 秒で 0.5m 動いていない」＝失敗と判定
3. 失敗 → 復帰動作の **`Spin` 90°** が走る

結果、**1.79m のゴールに対して累積 5.59m 動き 174° 回った。**
転倒もエラーも出ないので「なぜか着かない」としか見えない。

**最初の 1 本はこれで通す**（旋回が一度も要らないので、どこへ動くかが自明）:

```bash
python3 check_navigation.py --ahead 1.0 --tries 1 --no-sim-time \
    --planner-id Smac2D --max-stray 0.5 --timeout 40 --record /tmp/rec
```

- `--ahead D` … **今の向きに真っ直ぐ D m 先**。ゴールの向きも今の向きなので旋回ゼロ
- `--max-stray M` … 逸脱ガード。開始点から「直線 + M」を超えたら即キャンセル。
  **実機では必ず付ける**（復帰動作で想定外の方へ行ったとき timeout を待たずに止まる）
- 部屋の座標（`--waypoints`）を使う場合、向きは**機体からゴールへの方位**が入る

### 安全の決め事

1. **人が支える。** 最初の 1 本は必ず二人（操作と支え）で
2. **停止手段を手に持つ。** 一次はリモコンの緊急停止、二次は `loco_driver` の端末の Ctrl+C
3. **`--max-vy 0`** で前進と旋回だけに絞る（横移動を混ぜない）
4. **短距離から。** `--ahead 1.0` → 1.5 → 5 → 15
5. **バッテリーを見る。** `rt/lf/bmsstate` の `soc`（PC2 の SDK でしか読めない。
   コンテナには `unitree_hg` の型が無い）。38% で純正 SLAM が黙って止まった記録がある

---

## 7.5 当日の手順（1 本道。**判断は現地でしない**）

次の実機セッションはこの順で回す。**preflight が通るまで足を繋がない。**

```bash
cd G1_Hackason/Mapping/real

# ── 0. 立てる（実機に触らない）
bash quickstart/check_link.sh                              # 経路（見るだけ。直さない）
G1_RVIZ_CFG=$PWD/quickstart/rviz/g1_nav.rviz \
  bash quickstart/start_rviz_mac.sh live                   # ← ブリッジは**これが直す**
G1_USE_MOLA=1 bash quickstart/nav_stack.sh live

# ── 0.5. コストマップの消え残りを落とす。**preflight の前に 1 回**
bash quickstart/clear_costmaps.sh
#   立ち上げてから時間が経っているほど効く。減った分が消え残り、
#   残った分は本物（壁・人・家具）。**走行のたびに打つ必要は無い**

# ── 1. ゲート。**全部 OK でなければ先に進まない**
bash quickstart/preflight.sh

# ── 2. 逸脱ガードを常駐させる（RViz からクリックするなら必須）
docker exec -d -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI="$(bash -c '. quickstart/_common.sh; g1_dds_uri live')" \
  -e ROS_DOMAIN_ID=0 rviz bash -c \
  'source /opt/ros/humble/setup.bash && python3 \
   /work/G1_Hackason/Mapping/real/quickstart/stray_guard.py \
   --margin 0.5 --max-abs 3.0 --max-seconds 60 --no-sim-time \
   > /home/ubuntu/guard.log 2>&1'

# ── 3. 足を繋ぐ。**人が支え、リモコンを手に持ってから**
bash Navigation/real/deploy_to_pc2.sh                      # md5 で検証して配る
ssh g1 'bash ~/mapping_tools/start_cmd_vel_bridge.sh'      # ROS 側（まだ動かない）
ssh g1 'python3 ~/nav_tools/loco_driver.py --dry-run --arm --max-vy 0'   # 素振り
ssh -t g1 'python3 ~/nav_tools/loco_driver.py --network-interface eth0 --max-vy 0 --arm'
#   ↑ 前景で走らせる。**この端末の Ctrl+C が停止手段**

# ── 4. 1 本目: 真っ直ぐ 1m（旋回ゼロ）＋ 走行中の重畳
bash quickstart/run_stage.sh ahead 1.0
bash quickstart/run_stage.sh ahead 1.0 --repeat 3   # 続けて 3 本（1 本ずつ別フォルダ）
#   ⚠️ **1.0 未満にしない。** 0.80 m 以下は警告が出る（止めはしない。理由は §8）

# ── 5. 2 本目: 地図上の絶対座標（歩く前に ComputePathToPose で検算される）
bash quickstart/run_stage.sh goal 1.15 1.02      # ← 第一候補（下の表）

# ── 6. 3 本目: RViz2 の「2D Goal Pose」でクリック
#   ⚠️ **ドラッグの向きがそのままゴールの yaw になる。**進行方向に沿ってドラッグする。
#      横や逆を向けると位置は着いてもその場で回り続け、Spin 復帰を呼ぶ
#   ガードのログ: docker exec rviz tail -f /home/ubuntu/guard.log
```

⚠️ **手順 0 で colima に手を出さない。** `check_link.sh` が「コンテナ → PC2 に届かない」
と言っても、**現地で `colima restart` を打つか迷わなくてよい**。ブリッジの検査と
張り直しは `start_rviz_mac.sh live` が自分でやる（§2 の `G1_COLIMA_RESTART`）。
張り直しに入ると VM の再起動で 2 分ほど黙るが、何をしているかは標準出力に出る。

⚠️ **距離を縮めて安全を買おうとしないこと。** 人が近くにいるときに縮めたくなるが、
`ahead 0.5` は**歩かずに「到達 1/1」と出る**（§8）。安全は距離ではなく
**逸脱ガードの上限**で買う。`stray_guard.py --margin 0.3 --max-abs 1.2 --max-seconds 20`
のように締めれば、1.0 m の走行でも暴走は 1.2 m で止まる。

### 実験段階の方針（2026-09-10 に決めた）

**商用ではないので「歩き出す前の関門」は置かない。** `run_stage.sh` も
`check_planning.py` も、気になることは**警告で出してそのまま走る**。
実験を速く回すことを優先し、**機体を止める役目は走行中の逸脱ガードだけ**が持つ。

⚠️ ただし**走った後の裏取りは落とさない**（`verify_arrival`）。止めないぶん、
「動かずに成功」のような嘘の結果を後から必ず拾えるようにしておく必要がある。

速さのつまみ（既定は実測から決めてある。括弧内は 2026-09-10 の実測値）:

| 環境変数 | 既定 | 実際に必要な時間 |
|---|---|---|
| `G1_BAG_WARMUP_S` | 1.0 | bag の購読完了まで **0.12 s** |
| `G1_BAG_FLUSH_S` | 1.0 | bag の書き出し完了まで **0.015 s** |
| `G1_NAV_WARMUP_S` | 2.0 | `check_navigation` の TF 溜め（旧 5 s 固定） |
| `G1_PREFLIGHT_RATE_SCALE` | 1.0 | `ros2 topic hz` の測定窓の倍率。**`hz` は必ず窓いっぱい待つ** |

`preflight.sh` の所要は 1 本あたりの窓を実レートに合わせて **36 s → 16 s** に、
`tf2_echo` を 14 s → 6 s に詰めてある。local costmap の中身も
**同じメッセージを 2 回取りに行っていた**のを 1 回にした。

### 合否（測る前に決めてある）

| # | 合格 | 測り方 |
|---|---|---|
| 4 | 1 回で到達（xy 0.30m / yaw 0.35rad 以内）・転倒 0・ガード不作動 | `run_stage.sh` の出力 |
| 4 | **歩行中の重畳 ≥ 85%**（立脚静止の実測は 89.2%） | `run_stage.sh` が自動で出す |
| 5 | 2〜3m の絶対座標に 3 回中 2 回 | `run_stage.sh goal` を 3 回 |
| 6 | クリック 1 回で到達し、ガードが待機に戻る | 目視 ＋ `guard.log` |

⚠️ **4 が落ちたら 5・6 はやらない。** 変数を増やさない。

### 絶対座標のゴール候補（**当日に決めないで済むよう先に出してある**）

今日の実機の立ち位置 `(-0.35, 0.05) yaw 25°` から 1.5〜3.5m にある waypoint:

| 距離 | 機体正面からの方位 | X | Y | clearance | |
|---|---|---|---|---|---|
| 1.79 m | +8° | **1.15** | **1.02** | 2.12 m | **第一候補**（方位が小さく旋回が少ない）|
| 3.42 m | +39° | 1.15 | 3.12 | 2.16 m | 予備 |

⚠️ **当日の立ち位置がずれたら読み直す。** `tf2_echo map base_link` で現在地を見て、
`runs/<SESSION>/measure_20260908/waypoints.json` から 2〜3m・方位が小さいものを選ぶ。
**方位が大きいゴールは出発時に大きく回るので、最初の 1 本には向かない。**

### 止める

```bash
ssh g1 'bash ~/mapping_tools/start_cmd_vel_bridge.sh stop'
# loco_driver は前景の端末で Ctrl+C
bash quickstart/nav_stack.sh stop
```

---

## 8. 雑多だが毎回引っかかること

### 歩かずに「到達 1/1」と出る（2026-09-10 に実機で踏んだ）

**いちばん危ない型。落ちない・転倒しない・エラーコードも出ない。**
`ahead 0.5` を投げたら `到達 1/1 / error_code=None / recoveries=0` と出たが、
bag の `map -> base_link` 119 サンプル 11.83 秒で**開始点からの最大距離は 0.049 m**
（＝測位のゆらぎ）だった。**1 歩も歩いていない。**

機構は 3 つが噛み合ったもの:

1. **コストマップの消え残り。** 障害物層（voxel_layer）の印は、そのセルを貫くレイが
   後から来ないと消えない。30 分ほど立ち上げたままにしたら、機体まわり ±3 m の
   LETHAL 1,305 セルのうち **384 セルが「事前地図にも無く、その時 LiDAR が見てもいない」**
   ものになっていた。**そのうちの 1 つがちょうどゴールのセルだった**
2. **Smac2D は `tolerance`（0.50 m）の中で一番近い到達可能点を返す。**
   ゴールが 0.50 m 先だと、**機体の現在地そのもの**がその点になり得る。経路は 1 点だけ
3. **コントローラは経路の終端と機体を比べる。** 終端＝現在地なので
   **1.6 ms で「Reached the goal!」**、bt_navigator は `Goal succeeded`

対策は入れてある。3 つとも要る:

| 打ち手 | どこ |
|---|---|
| 消え残りを先に落とす | `clear_costmaps.sh`（§7.5 の手順 0.5）。実測で全体 LETHAL 20,046 → 19,525、ゴールのセルは 100 → 0 |
| 短すぎるゴールを**警告する** | `run_stage.sh` が `ahead D` で `D ≤ xy_goal_tolerance + planner tolerance`（= 0.80 m）なら警告を出す。値は `g1_nav2.yaml` から読むので焼き込みではない。⚠️ **実験段階なので止めない**（2026-09-10 の方針）。嘘の成功は下の裏取りが拾う |
| 成功を幾何で裏取りする | `check_navigation.py` の `verify_arrival()`。アクションが成功と言っても**終端からゴールまでが `--arrive-tol`（既定 0.35 m）を超えていたら失敗として数える**。`navigation.json` に `checks`（実移動量・ゴールまでの距離）も残す |

⚠️ **`results` だけを見ないこと。** `navigation.json` の `checks` に実移動量が入っている。
`== 到達 n/m ==` の後に「Nav2 が成功と言ったが着いていない回」があれば必ず出る。



### 古いスタックが残る

`nav_stack.sh stop` は `lifecycle_manager` を落とさない。
二重の publisher が `/tf` と `/map` に居ると原因不明のずれになる。

```bash
bash quickstart/nav_stack.sh stop
docker exec rviz bash -lc '
pkill -9 -f "octomap_server_nod[e]"; pkill -9 -f "nav2_"; pkill -9 -f "lifecycle_manage[r]"
pkill -9 -f "map_serve[r]"; pkill -9 -f "odom_to_t[f]"; pkill -9 -f "bag pla[y]"; pkill -9 -f mola'
```

### RViz2 を Mac から見る

`http://192.168.123.201/`（VNC パスワード `ubuntu`）。ブリッジが生きているときだけ届く。

### RViz2 のスクリーンショットを取る

```bash
docker exec -u ubuntu -e DISPLAY=:1 -e XAUTHORITY=/home/ubuntu/.Xauthority rviz \
  bash -c 'import -window root /tmp/x.png'
docker cp rviz:/tmp/x.png ./x.png
```

**root で `import` すると X の cookie が引けない**（`Authorization required`）。
`-u ubuntu` と `XAUTHORITY` の両方が要る。

### OctoMap の 3D voxel を見る

`g1_nav.rviz` の `OctoMap voxels`（`/occupied_cells_vis_array`・`MarkerArray`）。
**2026-09-07 まで `Enabled: false` で一度も見えていなかった。**
`octomap-rviz-plugins` は要らない（`octomap_server` が MarkerArray を自分で出す）。
**fps は 31 → 11 に落ちる**ので、判定のときだけ入れて済んだら切る。

### 点群・軌跡の解析は Mac 側でやる

**コンテナには numpy も open3d も無い。**
`Navigation/.venv/bin/python` を使う（numpy 2.2.6 / open3d 0.19.0 / scipy / pyyaml）。

`numpy 2.2.6` ＋ Apple Accelerate は**正常な入力でも `matmul` で警告を投げる**
（乱数でも再現する空振り）。黙らせてよいが、**非有限値の混入は別途 `assert` で見ること**。

### 記録に異物が混ざっている

`20260906T135940_UiS_room_v3` の `/utlidar/cloud_livox_mid360` には、
**SLAM 点群のメッセージが 229/5,900 件書かれている**
（`frame_id=map` / `point_step=48` / 8 フィールド。生 LiDAR は `livox_frame` / 22 / 6）。

**`frame_id` を見ずに `point_step=22` を仮定して読むと、229 回だけ別座標系の点を掴む。**
`measure_overlay.py` は**両方**で弾く。**他の記録は未調査。**

### コンテナを起こした直後は X（`:1`）がまだ無い

`docker start` の直後に RViz2 を起動すると
`qt.qpa.xcb: could not connect to display :1` で即死する（2026-09-10 に踏んだ）。
コンテナは `supervisord` → `vnc_run.sh` → `vncserver :1` → `Xtigervnc :1` の順に上がる。
`start_rviz_mac.sh` は **`xdpyinfo` が通るまで 2 秒おきに待つ**（上限 90 秒。超えたら落ちる）。

⚠️ **`xdpyinfo` は `-u ubuntu` ＋ `XAUTHORITY` で打つ。** root で打つと X が完全に
上がっていても `Authorization required` で必ず失敗する。

```bash
# ❌ X が上がっていても失敗する
docker exec rviz bash -c 'DISPLAY=:1 xdpyinfo'
# ✅
docker exec -u ubuntu -e DISPLAY=:1 -e XAUTHORITY=/home/ubuntu/.Xauthority \
  rviz bash -c 'xdpyinfo | head -3'
```

⚠️ **ポート 80（noVNC の websockify）は X より先に開く。**
`nc -z <VM_IP> 80` を「X の準備完了」の判定に使ってはいけない。
`/tmp/.X11-unix/X1` の有無も cookie が読めるかとは別なので単体では足りない。

---

## 9. もっと詳しく

| 文書 | 中身 |
|---|---|
| `docs/plan/2026-09-07_2-mola-lo-map-alignment.md` §8 | 実行の記録。合否・対照実験・詰まり 5 件・残件 |
| `docs/作業ログ/2026-09-07_2_MOLA-LO_地図の重畳を直す.html` | 上の画像つき版 |
| `docs/作業ログ/2026-09-06_3_Nav2_地図の重畳ずれ.html` | **原因を突き止めた回**（追記 1〜6）。なぜ MOLA-LO なのかの根拠 |
| `Navigation/real/README.md` | 純正方式と Nav2 方式の違い。`loco_driver.py` の安全機構 |
| `README.md`（このフォルダ） | 建図の手順 |
