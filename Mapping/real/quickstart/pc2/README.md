# PC2（機体の Orin NX）で測位と Nav2 を動かす

2026-09-14 に **測位（MOLA-LO）と Nav2 の両方が PC2 で動くところまで**通した。
ここはその再現手順と、踏んだ罠の記録である。

## なぜ PC2 でやるのか

無線で歩かせると **Mac のコンテナからは DDS が届かない**。発見は multicast で
行われ、multicast は L2 を越えない。AP（Ubuntu ゲーミング PC）は NAT しているので
unicast peer を並べても participant の locator が壊れる。
PC2 は LiDAR と同じ機体内部の網に居るので、この問題が丸ごと消える。

---

## 1. いま PC2 に在るもの

| 置き場 | 中身 |
|---|---|
| `~/jammy_ros/` | jammy の ROS 2 Humble を focal 上で動かす prefix（2.3 GB）。`env.sh` を source して使う |
| `~/g1_cfg/mola/g1_lidar3d_icp_imu.yaml` | MOLA のパイプライン（**既定のものは使えない**。罠 6） |
| `~/g1_cfg/map/map.mm` | MOLA の 3D 地図（1.8 MB） |
| `~/g1_cfg/map/nav_map_run.{yaml,pgm}` | Nav2 の 2D 地図 |
| `~/g1_cfg/nav2/g1_nav2.yaml` | Nav2 の params。⚠️ BT のパスをコンテナ用から書き換えてある |
| `~/g1_cfg/nav2/navigate_g1.xml` | BT（既定 Smac2D） |
| `~/g1_cfg/nav2/amcl_live.yaml` | AMCL の params（「6.」） |
| `~/g1_cfg/cloud_to_scan.py` | 3D 点群 → `/scan`（「6.」） |
| `~/g1_cfg/dog_odom_to_tf.py` | `/dog_odom` → TF `odom -> base_link`（「6.」） |
| `~/g1_cfg/run_*.sh` | 起動スクリプト（このディレクトリの複製） |

---

## 2. 起動

```bash
# 機体の電源が入っていること。PC2 に SSH で入る
ssh unitree@192.168.123.164        # 有線
ssh unitree@10.42.0.76             # 無線（下の「3. ネットワーク」が要る）

# 測位 ＋ Nav2 をまとめて
bash ~/g1_cfg/run_stack_live.sh
bash ~/g1_cfg/run_stack_live.sh --init "[x, y, 0.0, yaw_deg, 0.0, 0.0]"
G1_PC2_SECONDS=150 bash ~/g1_cfg/run_stack_live.sh   # 秒数を切る

# 止める
bash ~/g1_cfg/run_stack_live.sh --stop
```

ログ: `/tmp/pc2_mola.log` `/tmp/pc2_nav2.log` `/tmp/pc2_mapserver.log` `/tmp/pc2_stf.log`

### 脚 odom を事前情報に渡す（2026-09-15 追加）

`run_mola_live.sh` は既定で純正の脚 odometry **`/dog_odom`** を MOLA の
launch 引数 `odom_topic_name` に渡す（`ros2-lidar-odometry.launch.py` 384-388 行目）。
BridgeROS2 が `CObservationRobotPose` に変換し、`StateEstimationSimple` の
`fuse_odometry_3d_pose()` が**増分**として last_pose に足し込む
（`StateEstimationSimple.h` 186-192 行目）。そこから ICP の prior になる。

```bash
G1_ODOM_TOPIC="" bash ~/g1_cfg/run_mola_live.sh ...   # 渡さない（従来動作）
G1_ODOM_TOPIC=/other_odom bash ~/g1_cfg/run_mola_live.sh ...
```

⚠️ `forward_ros_tf_odom_to_mola:=True` とは**排他**。両方立てると launch の
`validate_odometry_sources()` が RuntimeError で止まる。

### 通ったときの見た目（2026-09-14 実測）

```
MOLA   Map loaded successfully / MRPT exception 0 / lookupSensorPose 0
Nav2   bt_navigator planner_server controller_server behavior_server
       smoother_server global_costmap local_costmap map_server
       lifecycle_manager  ← すべて activated
TF     Nav2 の "Timed out waiting for transform" が 0 回
```

### ⛔ 歩かせる前に必ず

**初期姿勢は地図の datum を焼いたままである。** 機体がそこに居なければ MOLA は
**偽の極大に落ちる**。09-13 の実機では真値から **2.26 m・16.4°** 外したまま
1 時間以上走り、`preflight.sh` は全項目 OK と答えた
（⚠️ `pose_quality` は**誤った姿勢の方が高い**: 0.855 対 0.818）。

Mac 側の `bootstrap_localization.sh`（大域探索＋二重ゲート・実測 3 秒）が出した姿勢を
`--init` で渡すこと。**PC2 への移植は未了**（段 4）。

---

## 3. ネットワーク

機体は二重接続（`eth0` 192.168.123.164 有線／`wlan0` 10.42.0.76 無線）。
**両方同時に使える**。宛先アドレスが別なので競合しない。

| 宛先 | 経路 | 要るもの |
|---|---|---|
| `192.168.123.164` | Mac `en8` → スイッチ → 機体 `eth0` の**直結** | ケーブルだけ |
| `10.42.0.76` | Mac → 静的経路 → AP が転送＋NAT → 機体 `wlan0` | 下の 3 つ |

### 無線を通すのに要る 3 つ（2026-09-14 に設定・恒久化済み）

```bash
# (1) Mac の静的経路（networksetup なので再起動後も残る）
sudo networksetup -setadditionalroutes AX88179B 10.42.0.0 255.255.255.0 192.168.123.200
networksetup -getadditionalroutes AX88179B          # 確認

# (2)(3) AP 側の filter ＋ NAT。dispatcher が AP を上げるたび入れ直す
ssh ubuntu@192.168.123.200 'sudo install -m 755 ~/50-g1teleop-forward \
    /etc/NetworkManager/dispatcher.d/'
```

⚠️ **(2)(3) の設置は 2026-09-14 時点で未確認のまま終わった**（機材が落ちて検証できず）。
次回いちばん先に確かめること:

```bash
ssh ubuntu@192.168.123.200 'grep -c "nat POSTROUTING" \
    /etc/NetworkManager/dispatcher.d/50-g1teleop-forward'   # 1 なら NAT 版が入っている
ping 10.42.0.76                                              # これが通れば全部 OK
```

### なぜ filter と NAT の 2 本が要るのか（実測）

1. **行きが reject される。** NetworkManager の `shared` モードは AP を上げるたび
   `nm-sh-fw-<dev>` を作り直し、**LAN 側からの新規接続を reject** する
   （通すのはクライアントの外向きと `ct related,established` だけ）。
   実測: ping 10 発がそのまま reject のカウンタに吸い込まれた（1 → 11）。
2. **戻りが機体の内部スイッチへ消える。** PC2 の `eth0` は**機体内部のスイッチ**に
   繋がったままで（PC1 `.161` / LiDAR `.120`）、その内部網も **192.168.123.0/24**。
   Mac は `192.168.123.202` なので機体は
   `ip route get 192.168.123.202 → dev eth0` と判断し、返事を内部へ投げる。
   ⛔ **機体側の経路を触ってはいけない**（PC1 と LiDAR への通信が壊れる）。
   ⇒ AP 側で送信元を `10.42.0.1` に書き換える。
   確認: 機体で `echo $SSH_CLIENT` が `10.42.0.1` になっていれば効いている。

⚠️ **PC2 にインターネットは無い。** `eth0` の既定経路 `192.168.123.1` は機体内部の
スイッチ上に存在せず、AP 側にも出口が無い。パッケージを足すときは下の「5.」を使う。

---

## 4. 踏んだ罠（全部ここで潰してある）

| # | 症状 | 原因と手当て |
|---|---|---|
| 1 | `mola-cli` が SIGSEGV・出力 0 行 | `jrun` が `LD_LIBRARY_PATH` を子に渡し、ラッパの **focal `/bin/sh` が落ちていた**。`--library-path` だけで足りるので export をやめた（**罠 5**）|
| 2 | スキャンは届くのに 1 枚も処理されない | `base_link→livox_frame` の静的 TF が無い（`odom_to_tf.py` は PC2 に無い）。`run_mola_live.sh` が `static_transform_publisher` を起こす。⚠️ `ignore_lidar_pose_from_tf:=true` は代用にならない（取付角を渡せない）|
| 3 | 最初の 1 スキャンで致命停止 | 既定パイプラインが `RKNN search requires nanoflann>=1.5.1`。arm64 Humble の mp2p_icp は nanoflann 1.4.2。`mola/g1_lidar3d_icp_imu.yaml` は回避済み |
| 4 | Nav2 が `librcl.so` で落ちる | ELF の PT_INTERP が focal の ld.so（罠 3）。`write_jammy_env.sh` を「`$ROS/bin` と `$ROS/lib` の ELF を**全部**ラップ」に直した（**71 個**）|
| 5 | `static_transform_publisher` が起動しない | 罠 4 の副作用でラッパ（`#!/bin/sh`）になったため、`jrun` に渡すと落ちる。`ros2 run` 経由に変えた |
| 6 | `timeout jros2 ...` が `No such file or directory` | **`jros2` はシェル関数**。`timeout` は実行ファイルしか起動できない |
| 7 | `ros2 run` が無い | MOLA の閉包に `ros2run` は入らない。CLI 一式を別途入れた |

⚠️ **止めるときは SIGINT。** SIGTERM だと内側の `ros2 launch` が子を孤児にする。

---

## 5. インターネット無しでパッケージを足す

PC2 は外に出られないが、**Mac のコンテナが jammy/arm64** なのでそこで deb を取れる。
`setup_jammy_ros.sh` と同じ「空の `dpkg status` で閉包を全部引く」やり方が使える。

```bash
# 1) コンテナ側で閉包を解決して URL を出す（PKGS は足したいもの）
docker exec rviz bash -c 'R=/tmp/jammy_nav2; OPTS=$(cat $R/aptopts); \
  apt-get $OPTS install --no-install-recommends --download-only -y --print-uris <PKGS>' ...

# 2) PC2 に既に在る deb 名と突き合わせ、不足だけ wget

# 3) Mac のディスクを経由せず tar でパイプする（空きが少ないため）
docker exec rviz tar cf - -C /tmp/nav2debs . | ssh g1 'tar xf - -C ~/jammy_ros/nav2debs'

# 4) PC2 で展開して環境を作り直す
ssh g1 'for d in ~/jammy_ros/nav2debs/*.deb; do dpkg -x "$d" ~/jammy_ros/rootfs; done
        cp -n ~/jammy_ros/nav2debs/*.deb ~/jammy_ros/var/cache/apt/archives/
        bash <write_jammy_env.sh> ~/jammy_ros'
```

2026-09-14 の実測: Nav2 の閉包は **1006 個 456.8 MB**、PC2 に無いのは **500 個 187 MB**
だけだった。転送は有線で **2 秒**。
⚠️ **`var/cache/apt/archives/` に寄せておくこと。** 次回の差分計算がこれで効く。

---

## 6. AMCL で測位する（2026-09-15 追加・MOLA-LO の代わり）

MOLA-LO は **機体が立ったまま静止していても 8.4 秒で 0.527 m 滑った**。
そのとき PC2 の load average は 8 コアで **8.01**（飽和）で、MOLA 単体が **253% CPU**。
`sigma-is-not-the-cause-cpu-contention-is` の通り、再生では壊れず負荷で再現する。
⇒ **Nav2 標準の AMCL（2D パーティクルフィルタ）に置き換えた。**

```bash
bash ~/g1_cfg/run_amcl_live.sh --init "0.703 12.966 -57.5"   # x[m] y[m] yaw[deg]
G1_PC2_SECONDS=120 bash ~/g1_cfg/run_amcl_live.sh --init "..."
bash ~/g1_cfg/run_amcl_live.sh --stop
```

### 構成（TF を出すのは 2 つだけ）

| 出すもの | 誰が |
|---|---|
| `map -> odom` | **amcl** |
| `odom -> base_link` | `dog_odom_to_tf.py`（`/dog_odom` を 2D に潰して詰め替える） |
| `base_link -> livox_frame` | 同上（静的・`xyz 0 0 1.228` / `rpy 177.93 3.32 0` deg） |
| `/scan` | `cloud_to_scan.py`（3D 点群 → base_link 系の帯 1.30〜1.80 m） |

⚠️ **MOLA は起こさない。** `map -> base_link` と衝突して base_link の親が 2 つになる。
⚠️ **controller も `loco_driver.py` も起こさない。** これは測位だけ。`/cmd_vel` は出さない。

### 3D → 2D は deb ではなく自作にした

`pointcloud_to_laserscan` の deb を入れる道もあるが、PC2 には**インターネットが無い**ので
「5.」の手順（Mac のコンテナで閉包を解決 → tar → **`write_jammy_env.sh` で 71 個の
ELF ラッパを作り直す**）が要る。いま動いている MOLA と Nav2 がその env.sh に乗っている
ので、**動いている環境を作り直す危険**を 100 行の numpy で置き換えられるなら置き換える
方が安い。加えて自作なら**出力の座標系を base_link にできる**ので、AMCL 側の laser pose
が恒等になり、**取付角（ロール 178°）の掛け忘れという静かな失敗が構造的に起きない**。

### 実測（2026-09-15・機体は立位で静止・PC2 は 8 コア）

| 項目 | 値 |
|---|---|
| `/scan` | 9.65 Hz・帯の点 **1291 点/枚**・埋まったビン **342/720** |
| `/tf` | 107 Hz |
| `map -> base_link` | (0.703, 12.966) / -57.48 deg ＝ **`--init` と一致** |
| CPU | amcl **7.2%** ／ cloud_to_scan 13.8% ／ dog_odom_to_tf 21.5% ／ map_server 4.5% |
| 合計 | **1 コアの約 47%**（8 コアの 5.9%）。load average 1.53 |

MOLA の 253% に対し **amcl 単体は 48 分の 1**。

### ⚠️ 帯 1.30〜1.80 m が成立する理由（と壊れ方）

MID-360 の縦 FOV は -7°〜+52° で、**上下逆さま**に付いているので
**水平より上はおよそ 7° しか見えない**。センサ高 1.228 m なので、帯 1.30〜1.80 m は
「水平よりわずかに上」の細い円錐でしか拾えない。それでも実測 1291 点/枚あり、
720 ビン中 342 が埋まる。点が足りなくなったら `G1_SCAN_MIN_HEIGHT` を下げる
（地図は床上 0.23〜1.80 m の帯で焼いてあるので 1.80 を超えない限り矛盾しない）。

`laser_model_type` は **`likelihood_field` 固定**。地図（0.23〜1.80 m）の方が
スキャン（1.30〜1.80 m）より濃いという非対称があり、`beam` だと自由空間の食い違いを
罰してしまう。

### 踏んだ罠（「4.」の続き）

| # | 症状 | 原因と手当て |
|---|---|---|
| 8 | `G1_PC2_SECONDS=45` のつもりが **199 秒動いた** | **背景（`&`）に置いた子は SIGINT を無視する**（非対話シェルの規則）。`kill -INT $CHILD` が無視される。⇒ 自動停止は**自分自身に SIGTERM**、後始末は INT → TERM → KILL と上げ、孫はパターンで落とす |
| 9 | `pkill -f run_amcl_live` を打った ssh が**無言で死ぬ** | `bash -c "...pkill -f X..."` の**自分のコマンドラインに X が入っている**。`[r]un_amcl_live` と書いて外す |
| 10 | `dog_odom_to_tf.py` が 1 コアの 66% | `/dog_odom` は 1008 Hz。コールバックで間引いても **take は消えない**。(1) `raw=True` で CDR のまま受ける (2) `spin_once` を 1/rate 秒に 1 回だけ呼ぶ。⇒ 21.5%。取りこぼしは `KEEP_LAST(1)` が捨てるので常に最新が残る |
| 11 | AMCL が起動直後に 1 件だけ scan を捨てる | `Message Filter dropping message ... earlier than all the data in the transform cache`。TF キャッシュが空の間に来た最初の 1 枚。**起動時の 1 行だけなら正常** |

---

## 7. まだ出来ていないこと

- **段 4**: `bootstrap_localization.sh` を PC2 へ移すか、Mac で出した姿勢を `--init` で渡し、
  **地図の 3 箇所で重畳 ≥ 70 %** を確認する。これを飛ばして歩かせてはいけない
- `/cmd_vel` を足に渡す（`~/nav_tools/loco_driver.py` は PC2 に在る）
- ゴールをクリックする画面（Mac の RViz2 か PC2 の `foxglove_bridge`。後者は 0.06 コア）
- `ros2 topic` は入れたので、**次は記録（`ros2 bag record`）も PC2 で回せる**はず（未検証）

---

## 8. GLIM の事前地図測位（2026-09-15 追加・MOLA-LO の代わり その 2）

MOLA-LO は立位・静止で震え **1.869 m/s**、推定が 4.8 m 彷徨った。CPU 競合でも脚 odom の
配線でもなく MOLA 側の問題だと 09-15 に確定している。同じ Livox MID-360・同じ屋内の
第三者ベンチで **GLIM(CPU) の APE RMSE は 0.025 m**（FAST-LIO2 0.060 / KISS-ICP 0.124）。

⚠️ **AIST の GLIL（純正の事前地図測位）は商用クローズドで使えない。**代わりに MIT の
フォーク `se7oluti0n/glim_localization`（GLIM v1.0.4 の `localization` ブランチ。
`global_mapping` を改造して保存地図を読み、照合する）を使った。

### 8.1 何を建てたか（全部 **CPU 版**・arm64 / Ubuntu 22.04 / ROS 2 Humble）

| 物 | 出どころ | 版 |
|---|---|---|
| GTSAM | **`ros-humble-gtsam`（packages.ros.org の jammy/arm64 deb）** | 4.2a9 |
| gtsam_points | `se7oluti0n/gtsam_points` ブランチ `localization` | 1.0.4 |
| glim（本体） | `se7oluti0n/glim_localization` ブランチ `localization` | 1.0.4 |
| glim_ros | `se7oluti0n/glim_localization_ros2` | 1.0.4 |

⚠️ **CUDA は使わない。** PC2 は Orin NX だが JetPack 5 系（Ubuntu 20.04 / CUDA 11.4）で、
GLIM が想定する CUDA 12.2+ / JetPack 6.1 と合わない。そして上のベンチで最良だったのは
CPU 版である。`-DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF`（Iridescence を避ける）。

⚠️ **GTSAM を自前で建てる必要は無かった。**koide3 の PPA にあるのは 4.3.0 で、
1.0.4 系のフォーク（`boost::shared_ptr` を使う）とは合わない。ところが
**`packages.ros.org` の `ros-humble-gtsam` が中身 4.2a9**（`gtsam/config.h` の
`GTSAM_VERSION_STRING`）で、gtsam_points 1.0.4 が想定するまさにその版だった。

ソース: `third_party/glim_src/`（4 リポジトリ）。ビルドは**コンテナ `rviz` の中の
`/root/glim_b`（ビルド）と `/opt/glim_loc`（install）**で行った。`/work` の既存物には触っていない。

### 8.2 フォークに当てたパッチ 4 つ（**全部これが無いと動かない**）

| # | 場所 | 症状 | 原因 |
|---|---|---|---|
| 1 | `glim/src/glim/mapping/localization.cpp:77` | ビルドが `'tbb_task_arena' was not declared in this scope` で落ちる | **自由関数**から `Localization` のメンバを参照している。フォークの作者は TBB 無しの GTSAM で建てていたので露見しなかった。`ros-humble-gtsam` は TBB 付きで `GTSAM_USE_TBB` が立つ。⇒ ローカルに `tbb::task_arena(1)` を作る |
| 2 | `glim_ros2/src/glim_rosbag.cpp` | **建図が終わった瞬間に落ち、地図が保存されない** | `dump_path` は `GlimROS` のコンストラクタで既に declare 済み。二度目で `ParameterAlreadyDeclaredException` |
| 3 | `glim/include/glim/odometry/odometry_estimation_ct.hpp` | IMU 無しの構成で `/initialpose` を何度投げても `latest frame is null. Abort relocalize` | `get_latest_frame()` が `OdometryEstimationIMU` にしか実装されておらず、基底は**常に nullptr** |
| 4 | `glim_ros2/src/glim_ros/glim_ros.cpp` | （静かに壊れる）| G1 の LiDAR トピックに混ざる異物。下の 8.3 |

さらに `glim_ros2` から **`glim_octomap` ターゲットと pcl/octomap の依存を外した**。
我々は自前の 2D 地図を持っているので不要で、残すと `pcl_ros` 経由で
GDAL / spatialite / poppler / mysqlclient まで閉包に入る。

⚠️ **`Localization::load_ply()` は `localization.hpp:69` に宣言だけあって実体が無い。**
「PCD を GLIM の地図として読む」道は**存在しない**（8.4）。

### 8.3 G1 固有の入口（間違えると静かに外れる）

| 項目 | 値 | 根拠 |
|---|---|---|
| `acc_scale` | **9.80665** | G1 の IMU は **g 単位**（実測 \|acc\| = 1.0021）。1.0 のままだと重力が 1/9.8 になる |
| per-point time | **ナノ秒・フレーム先頭からの相対** | field `time` は float32 で 0 〜 100,320,000（= 0.1 s）。`autoconf` に任せず明示する |
| `T_lidar_imu` | 単位 | ドライバは LiDAR も IMU も `livox_frame` で出す。`fastlio_g1.yaml` の extrinsic も単位 |
| `points_frame_id` / `points_point_step` | `livox_frame` / `22` | **我々が足したパラメータ**（パッチ 4）|

⚠️ **異物の混入。** `/utlidar/cloud_livox_mid360` には `frame_id='map'` / `point_step=48` の
純正 SLAM 地図点群が混ざる（建図記録で 5900 件中 229 件 ＝ 3.9%）。**これも x/y/z を持つので
GLIM の `extract_raw_points` は通してしまい**、`header.stamp=0` のまま TimeKeeper に入る。
`frame_id` と `point_step` の**両方**で弾く（片方だけでは通り抜ける）。

⚠️ **Livox の IMU quaternion は (0,0,0,0) で来る**（実測 100%）。MRPT ではこれで例外＝致命停止
したが、**GLIM は orientation を読まない**（加速度と角速度だけ）ので影響しない。

### 8.4 ⚠️ 事前地図の座標系 — **GLIM 地図は既存の map 系に乗らない。変換で吸収した**

既存の地図（`map_octomap_r4_s5_floor0.pcd` / `nav_map_*.yaml`）は **floor0 規約**。
GLIM の事前地図は `GlobalMapping::save()` が吐く**サブマップの因子グラフ**
（`graph.txt` ＋ `%06d/data.txt` ＋ 点群のバイナリ）で、`Localization::load()` は
`SubMap::load()` でそれを読む。**既存の PCD を入れる口は無い**（`load_ply` は実体が無い）。

⇒ **採った道: 同じ記録から GLIM で地図を作り直し、GLIM 地図 → 既存 map 系の剛体変換を
求めて静的 TF で吸収する。地図そのものは 1 バイトも触らない。**

同じ記録なので両者の軌跡は同じ時刻に同じ物理点を指している。時刻で突き合わせ
（⚠️ **GLIM の方が中央値 0.0499 s 遅い**＝ちょうど半周期。最近傍だと歩行中の変位が
残差に乗るので **MOLA 側を補間する**）、Umeyama で解く。

```bash
Navigation/.venv/bin/python quickstart/pc2/align_glim_to_map.py \
    runs/20260906T135940_UiS_room_v3/glim/map \
    runs/20260906T135940_UiS_room_v3/mola_floor0/traj.txt \
    --out runs/20260906T135940_UiS_room_v3/glim/map_frame.txt
```

実測（2026-09-15）:

| 量 | 値 |
|---|---|
| 対応 | **5442 点**（GLIM 5660 / MOLA 5670） |
| 位置残差 | RMSE **0.0560 m** / 中央値 0.0375 / 最大 0.289 |
| 回転残差 | 中央値 **0.607 deg** / 最大 5.114 |
| `T_map_glimmap` | xyz `-0.003879 0.007641 1.243624` / rpy `-0.167 0.460 -111.254` deg |

z ≒ **1.2436** はセンサ高 1.228 とほぼ一致する（floor0 は床が z=0、GLIM の原点はセンサ）。
roll/pitch ≒ 0 は両方が重力整列している証拠。**592 s 歩き回って両者が 5.6 cm しか
食い違わない**ので、GLIM の建図も MOLA の建図も互いに裏を取れている。

⇒ `config_ros.json` の `map_frame_id` は **`"map"` ではなく `"glim_map"`**。鎖は

```
map --(静的 TF)--> glim_map --(GLIM localization)--> odom --(GLIM odometry)--> base_link
```

`still_report.py` は BFS で鎖を辿るので、`map -> base_link` が間接でもそのまま通る。

### 8.5 建図のやり方（Mac のコンテナで・約 100 秒）

```bash
docker exec rviz bash -c '
  set +u; source /opt/ros/humble/setup.bash; set -u
  export LD_LIBRARY_PATH=/opt/glim_loc/lib:$LD_LIBRARY_PATH
  export ROS_DOMAIN_ID=42           # ⚠️ ROS_LOCALHOST_ONLY は付けない（8.7 の罠 3）
  R=/work/G1_Hackason/Mapping/real/runs/20260906T135940_UiS_room_v3
  /opt/glim_loc/lib/glim_ros/glim_rosbag "$R/raw/rosbag2" --ros-args \
    -p config_path:=/work/G1_Hackason/Mapping/real/quickstart/pc2/glim_loc \
    -p auto_quit:=true -p dump_path:="$R/glim/map"'
```

実測: 2.8 GB / 592 s の記録を **8〜10 倍速**で処理して 100 秒。
出来た地図は **サブマップ 12 個・23 MB**（`runs/.../glim/map/`）。

### 8.6 再生での検証（`replay_glim_loc.sh`）

```bash
docker exec rviz bash /work/G1_Hackason/Mapping/real/quickstart/pc2/replay_glim_loc.sh \
    --bag <bag> --init "X Y YAW_DEG" --out <記録先> [--ct] [--seconds N]

Navigation/.venv/bin/python quickstart/pc2/still_report.py <記録先> \
    --ref runs/20260906T135940_UiS_room_v3/map/nav_map_ref.yaml --init X Y YAW_DEG
```

実測（`click3` / `click4` の**静止区間**・再定位の後だけを記録）:

| 量 | 合否 | click3 | click4 | 参考: MOLA(09-15) |
|---|---|---|---|---|
| 滑り（端から端） | < 0.05 m | **0.006** ✅ | **0.006** ✅ | 0.527 ❌ |
| 震え（0.5 s 窓） | < 0.10 m/s | **0.029** ✅ | **0.022** ✅ | 1.869 ❌ |
| 上限 0.361 m/s 超 | 0 % | **0.0** ✅ | **0.0** ✅ | 81 % ❌ |
| 初期姿勢からのずれ | < 0.30 m | **0.012** ✅ | **0.048** ✅ | — |
| 重畳 壁の帯 許容±1 | > 80 % | **90.5** ✅ | **84.9** ✅ | — |
| 再定位の overlap | — | 0.984 → 0.843 | 0.989 → 0.852 | — |

⚠️ **記録は再定位の後から始めること。** 再定位の前は `map -> odom` が単位のままなので
base_link は見当違いの場所に居る。そこから記録すると「滑り」に**再定位の跳び（実測 10.7 m）**が
そのまま入り、測位の良し悪しと無関係な数字になる（09-15 に 1 度読み違えた）。

⚠️ **`runs/_meas/B_amcl` `runs/_boot/bag` `runs/_boot/still` には IMU が入っていない。**
録ったトピックが `/utlidar/cloud_livox_mid360` と `/tf` 系だけである。GLIM の既定 odometry は
IMU 必須なので、これらは `--ct`（CT-ICP ＋ passthrough submapping）でしか回らない。
そして **CT の構成は事前地図とサブマップ原点の規約が食い違う**
（`sub_mapping.cpp:617` は回転を捨てるが passthrough は捨てない）ので、
実測で overlap が 0.4226 → 0.11463 まで落ち、yaw が 113° 外れた。
⇒ **判定に使ってはいけない。**live の機体は IMU を 200 Hz で出しているので本番はこの道を通らない。
検証は **IMU が入っている `click3` / `click4` の静止区間**（変位 0.008 m / 30 s）で行った。

### 8.7 PC2 へ流し込む

```bash
# 1 行。tar は third_party/ws_glim/glim_pc2.tar.gz（76.5 MB）
ssh g1 'tar xzf - -C ~' < third_party/ws_glim/glim_pc2.tar.gz

# 動かす
ssh g1 'bash ~/g1_cfg/run_glim_loc_live.sh --init "0.703 12.966 -57.5"'
ssh g1 'G1_PC2_SECONDS=120 bash ~/g1_cfg/run_glim_loc_live.sh --init "..."'
ssh g1 'bash ~/g1_cfg/run_glim_loc_live.sh --stop'
```

tar が置く物（`$HOME` からの相対）:

| 置き場 | 中身 |
|---|---|
| `~/jammy_ros/ws_glim/bin/` | `glim_rosnode` `glim_rosbag` |
| `~/jammy_ros/ws_glim/lib/` | 依存の閉包 **191 個 171 MB**。⚠️ **core（libc / libstdc++ / ld.so）は入れていない**。`--library-path` は `$JAMMY_LIBS` が先なので、PC2 に在る物はそちらが勝ち、無い物だけここから解決される |
| `~/g1_cfg/glim_loc/` | 設定一式（`ct/` を含む） |
| `~/g1_cfg/map/glim_map/` | GLIM の事前地図（サブマップ 12 個・23 MB） |
| `~/g1_cfg/map/glim_map_frame.txt` | `map -> glim_map` の変換（8.4） |
| `~/g1_cfg/run_glim_loc_live.sh` | 起動スクリプト |

**要る前提**

- `~/jammy_ros/env.sh` が在ること（「1.」の構成。`jrun` / `jros2` / `$LOADER` / `$JAMMY_LIBS`）
- ROS 2 の CLI に **`ros2 service` と `ros2 topic`** が在ること。
  地図の読み込み（`~/load_map` サービス）と再定位（`/initialpose`）に使う。
  `ros2 topic` は「6.」で入れた。**`ros2 service` は未確認**。無ければ「5.」の手順で
  `ros2-humble-ros2service` を足す
- `/tmp/pc2_lock` を取ってから動かすこと（DDS domain 0 ＋ NIC eth0 に出る）
- ⚠️ **MOLA / AMCL と同時に起こさない。** `map -> odom` / `map -> base_link` と衝突して
  TF の親が 2 つになる

**未解決ライブラリの確認**（実測 **0**。PC2 と同じく `--library-path` だけで解いた）

```bash
for f in ~/jammy_ros/ws_glim/bin/* ~/jammy_ros/ws_glim/lib/*.so; do
  "$LOADER" --library-path "$JAMMY_LIBS:$HOME/jammy_ros/ws_glim/lib" --list "$f" | grep "not found"
done
```

### 8.8 踏んだ罠（「4.」「6.」の続き）

| # | 症状 | 原因と手当て |
|---|---|---|
| 12 | `glim_rosbag` が `rmw_create_node: failed to create domain` で即死 | コンテナは既に `CYCLONEDDS_URI` を `lo` に固定している。そこに `ROS_LOCALHOST_ONLY=1` を足すと **`lo` が二度選ばれて** `the same interface may not be selected twice`。⇒ domain だけ分ける |
| 13 | `set -u` の下で `source /opt/ros/humble/setup.bash` が `AMENT_TRACE_SETUP_FILES: unbound variable` で落ちる | ROS の setup.bash は `set -u` を想定していない。`set +u; source; set -u` で挟む |
| 14 | 再定位が **overlap 0.0 の偽の一致で上書きされる** | `/initialpose` を投げっぱなしで 2 回送っていた。**受理をログで確認してから次を投げる**（確認してから送る順に直した） |
| 15 | 終了時に `terminate called without an active exception` で core を吐く | `Localization` の再定位スレッドが join されないまま落ちる。**測位そのものには影響しない**（仕事は全部終わってから落ちる）。未修正 |
| 16 | `libsub_mapping.so` を `enable_imu: false` で使うと毎フレーム `An inference algorithm was called with inconsistent arguments` | キーフレームを繋ぐ因子が IMU 因子しか無いので因子グラフが分断される。IMU 無しでは `libsub_mapping_passthrough.so`（最適化しない）に替える |
