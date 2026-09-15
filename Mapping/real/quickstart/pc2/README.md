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
