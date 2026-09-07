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
| Nav2 で歩く | **未達。**`odom` フレームが無い問題が残っている（§6） |

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

**ケーブルを抜き差ししたら `colima restart` が要る。**
Mac 側は生きたままなので気づきにくい。再起動後は `col0` の IPv4 が消えるので
`start_rviz_mac.sh live` に付け直させる。

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

## 6. Nav2 と繋ぐ（未達。ここが残件）

### `odom` フレームが存在しない

Nav2 の `local_costmap` は `global_frame: odom` を要求する（`g1_nav2.yaml`）が、
MOLA-LO を `publish_localization_following_rep105:=False` で回すと
**`map → base_link` を直接出すので `odom` が無い。**

対処案（**未検証**）:

```bash
# map -> odom を恒等の静的変換で置くと、TF が odom -> map -> base_link を辿って
# odom -> base_link を合成できる（odom ≡ map になる）
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

許容できる理由: `odom` を使う狙いは「局所コストマップを滑らかな座標系に置く」ことで、
この測位は**漂流 0.7 mm / ばらつき 0.39°** なので `map` でも滑らかである。

### `map_server` は自分で起こす

Nav2 の `global_costmap.static_layer` は `/map` を読むが、
**それを publish するノードが 2026-09-06 の構成に無く、静的レイヤが空のままだった。**
`nav_stack.sh` に追加済み。ライフサイクルノードなので `lifecycle_bringup` も要る。

### ⚠️ `nav_stack.sh --mola` は §4 の組み合わせと食い違っている

`G1_USE_MOLA=1` の経路は**まだ一度も通していない**。
`map_full.mm` を読む・`initial_pose` を渡さない・`ignore_*_pose_from_tf` が違う・
REP-105 が `True` など、**§4 で実機が動いた組み合わせと 6 箇所違う。**
直す前に §4 を手で流して確かめること。

---

## 7. 脚を繋ぐ

**PC2 側の 2 プロセス構成。**`rclpy` と `unitree_sdk2py` が同居できないため
localhost の UDP（47600）で繋いでいる。**安全機構は「止められる側」＝ SDK 側にある。**

```
[Nav2] --/cmd_vel--> [cmd_vel_bridge.py (pixi 3.11)] --UDP--> [loco_driver.py (system 3.8)] --> 足
```

```bash
# 素振り。⚠️ --dry-run 単体では何も表示されない（--arm の判定が先にあるため）
ssh g1 'python3 ~/nav_tools/loco_driver.py --dry-run --arm'

# 本番。前方のみに限るなら --max-vy 0（後退は既定で禁止）
ssh -t g1 'python3 ~/nav_tools/loco_driver.py --network-interface eth0 --max-vy 0 --arm'
```

既定のクランプ: `vx 0.30` / `vy 0.20` / `vyaw 0.50` / **無指令 0.5 秒で停止**。

---

## 8. 雑多だが毎回引っかかること

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

---

## 9. もっと詳しく

| 文書 | 中身 |
|---|---|
| `docs/plan/2026-09-07_2-mola-lo-map-alignment.md` §8 | 実行の記録。合否・対照実験・詰まり 5 件・残件 |
| `docs/作業ログ/2026-09-07_2_MOLA-LO_地図の重畳を直す.html` | 上の画像つき版 |
| `docs/作業ログ/2026-09-06_3_Nav2_地図の重畳ずれ.html` | **原因を突き止めた回**（追記 1〜6）。なぜ MOLA-LO なのかの根拠 |
| `Navigation/real/README.md` | 純正方式と Nav2 方式の違い。`loco_driver.py` の安全機構 |
| `README.md`（このフォルダ） | 建図の手順 |
