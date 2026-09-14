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

## 6. まだ出来ていないこと

- **段 4**: `bootstrap_localization.sh` を PC2 へ移すか、Mac で出した姿勢を `--init` で渡し、
  **地図の 3 箇所で重畳 ≥ 70 %** を確認する。これを飛ばして歩かせてはいけない
- `/cmd_vel` を足に渡す（`~/nav_tools/loco_driver.py` は PC2 に在る）
- ゴールをクリックする画面（Mac の RViz2 か PC2 の `foxglove_bridge`。後者は 0.06 コア）
- `ros2 topic` は入れたので、**次は記録（`ros2 bag record`）も PC2 で回せる**はず（未検証）
