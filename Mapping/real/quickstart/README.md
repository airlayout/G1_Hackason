# quickstart — Docker無しでLiDARを取り、地図にして、GUIで見る

`Mapping/real`本体はUbuntu PC + Docker + ROS 2 Humbleを前提にしている。
ここはその前提が揃わない環境（macOSの操作PCなど）でも**LiDAR取得から地図生成・検証・
GUI確認まで通す**ための最小経路。

**Docker・ROS 2・sudo・XQuartzのいずれも要らない。**

2026-09-02に実機で端から端まで実証した。

## 通しの例

初回配布（`## 配置`）を済ませてあれば、これで一周する。`Mapping/real`で実行する。

```bash
# 1. LiDARとSLAMが動いているか見る（ロボットには何も指令しない）
ssh g1 'python3 ~/mapping_tools/probe_dds_topics.py 5'

# 2. 60秒記録する。G1は静止させておく
ssh g1 'python3 ~/mapping_tools/record_dds_to_bag.py --duration 60 --name room_a'

# 3. 回収して地図にし、検証する（<id>は手順2が出力したsession）
rsync -a -e ssh g1:/home/unitree/mapping_runs/<id> runs/
./mapctl rebuild  <id>
./mapctl validate <id>

# 4. GUIで見る
../../Navigation/.venv/bin/python quickstart/view_pcd_gui.py runs/<id>/map/map_raw.pcd
```

## ブラウザで見ながら歩いて地図を作る（通しの手順）

2026-09-03に可視化まで実機で通した経路。`Mapping/real`で実行する。
詳しい背景は第6節。

### 事前（初回だけ）

```bash
# PC2 に配布物と Humble 環境を入れる
ssh g1 'mkdir -p ~/mapping_tools'
scp quickstart/*.py quickstart/*.sh g1:~/mapping_tools/
scp -r quickstart/humble_env g1:~/
ssh g1 'chmod +x ~/mapping_tools/*.py ~/mapping_tools/*.sh; bash ~/humble_env/setup_pc2.sh'
```

Mac 側には **Lichtblick** を入れる（`app.foxglove.dev`はアカウント必須。第6節参照）。

### 毎回の手順

```bash
# --- 1. 有線をつなぎ、疎通を確認する ---
#     Mac の en8 が 192.168.123.200 であること。WiFi は AP 分離で使えない
ping -c 3 192.168.123.164
ssh g1 'hostname'

# --- 2. LiDAR と SLAM が生きているか（ロボットには何も指令しない） ---
ssh g1 'python3 ~/mapping_tools/probe_dds_topics.py 5'

# --- 3. 可視化の橋を立てる ---
ssh g1 'bash ~/mapping_tools/start_foxglove_bridge.sh'
ssh g1 'bash ~/mapping_tools/start_odom_tf.sh'      # ロボットの現在位置を出したいなら

# --- 4. Lichtblick で接続する ---
#     Open connection -> **Foxglove WebSocket**（Rosbridge ではない）
#     -> ws://192.168.123.164:8765
#     （デスクトップ版なのでSSHトンネルは不要。ブラウザ版を使うなら
#       bash quickstart/tunnel_foxglove.sh を実行して ws://localhost:8765）

# --- 5. 3Dパネルを設定する（第6節の表のとおり） ---
#     Display frame = map / Decay time = 300 / Point size = 2〜5
#     /unitree/slam_mapping/points を有効化
#     /utlidar/cloud_livox_mid360 は**無効のまま**にする

# --- 6. G1 を立たせ、歩く直前に記録つきで建図を開始する ---
#     ※ 建図は放置すると自動停止する（後述）。必ず歩く直前に投げる
#     ※ -t が無いと Ctrl-C が届かない
ssh -t g1 'python3 ~/mapping_tools/record_dds_to_bag.py --with-mapping --name room_x'
#     これが 1801 を投げ、/unitree/slam_mapping/points と /odom を db3 に記録する

# --- 7. 純正リモコンで低速に外周を回り、開始地点付近へ戻る ---
#     ブラウザで地図の育ち方を見ながら、薄い場所を埋める
#     急旋回・足踏み・長時間の静止を避ける

# --- 8. Ctrl-C で停止する（1802 が自動で飛ぶ） ---

# --- 9. 回収して地図にする ---
rsync -a -e ssh g1:/home/unitree/mapping_runs/<id> runs/
./mapctl rebuild  <id>
./mapctl validate <id>
../../Navigation/.venv/bin/python quickstart/view_pcd_gui.py runs/<id>/map/map_raw.pcd

# --- 10. 片付け ---
ssh g1 'bash ~/mapping_tools/start_foxglove_bridge.sh stop 2>/dev/null; \
        bash ~/mapping_tools/start_odom_tf.sh stop'
```

### 落とし穴（実際に踏んだもの）

**ケーブルを抜かないこと。** WiFi は AP 分離で Mac↔PC2 が通らない（第6節）。抜くと
ブラウザ表示が止まり、**1802 を投げられなくなる**。2026-08-26 に建図を2回とも壊した
のと同じ状況になる。

**建図は放置すると自動停止する。** 2026-09-03に2回発生（約16分と20〜40分）。停止時の
ロボットは`sportMode=-1` / `gaitType=-1`＝アクティブな動作モードに入っていない状態
だった。原因は未特定。**歩く直前に手順6を投げるのが確実。**
`probe_dds_topics.py`で`slam_mapping/points`が止まっていたら建図が終わっている。

**1802 が保存する PCD は回収できない。** PC1(`192.168.123.161`)はpingに0.5msで応答するが
22/21/23/80/445/873/2049/8080/8000/9000のいずれも閉じている（2026-09-03に
MacとPC2の両方から確認）。**持ち帰れる地図は手順6で記録したdb3だけ**なので、
記録を省略しないこと。

**蓄積ビューは後からの補正を反映しない。** `slam_mapping/points`は増分スキャンなので、
G1のLIOがループ閉じ込みで過去を修正しても表示は動かない。進捗確認には十分だが、
最終地図の正確なプレビューではない。

## 何を流用し、何を足したか

| 役割 | 実装 | 状態 |
|---|---|---|
| 地図の再構成 | `mapctl rebuild` | **無改造で動く** |
| 成果物の検証 | `mapctl validate` | **無改造で動く** |
| セッションの成果物レイアウト | `python/g1_mapping/session.py`に準拠 | そのまま |
| 点群の記録 | `record_dds_to_bag.py` | **ここで追加** |
| サービスの死活確認 | `probe_dds_topics.py` | **ここで追加**（`mapctl doctor`の代わり） |
| GUI表示 | `view_pcd_gui.py`（Open3D） | **ここで追加**（RViz2の代わり） |

### なぜ記録だけ作り直したのか

`Mapping/real`の記録は`docker/record-topics.sh` = `ros2 bag record`である。これは
G1と同じL2に居るROS 2 Humble環境を要求する。

- **macOSのDocker（colima等）はVM内のNATネットワーク**にいるため`192.168.123.0/24`へ
  届かず、DDSが通らない
- **PC2（`.164`）はG1のL2に直結しているが、ROS 2 foxyのCLIからUnitreeのトピックが
  見えない。** `rmw_cyclonedds_cpp`を指定しても`ros2 topic list`は空になる
  （`Navigation/README.md`の実測記録）

一方`unitree_sdk2py`の`ChannelSubscriber`は型を明示すれば購読できる。そこで
**PC2で直接DDS購読し、受けたCDRをそのままrosbag2のdb3へ書く**ことにした。

cyclonedds由来のIDL型の`serialize()`が返すCDRは先頭が`00 01 00 00`で、
`rebuild.py`の`_CdrReader`が期待する形式と完全に一致する。したがって記録形式さえ
db3に合わせれば、**下流（rebuild / validate）は一行も変えずに使える**。

## 配置

| ファイル | 実行場所 |
|---|---|
| `probe_dds_topics.py` | **PC2** |
| `record_dds_to_bag.py` | **PC2** |
| `mapping_ctl.py` | **PC2**（1801/1802/1901。`record_dds_to_bag.py`が読み込む） |
| `diagnose_ros2.sh` | **PC2**（ROS 2 がUnitreeのトピックを掴めるかの切り分け） |
| `humble_env/` | **PC2**（pixiでROS 2 Humbleを入れる。`setup_pc2.sh` / `pixi.toml` / `pixi.lock`） |
| `start_foxglove_bridge.sh` | **PC2**（ブラウザ可視化の橋を立てる） |
| `odom_to_tf.py` / `start_odom_tf.sh` | **PC2**（G1のodomをTFに変換。これが無いと3Dパネルに位置が出ない） |
| `restamp_points.py` / `start_restamp.sh` | **PC2**（地図点群の時刻0を打ち直す。これが無いと蓄積表示ができない） |
| `view_pcd_gui.py` | **操作PC**（GUIはローカルに出す） |
| `tunnel_foxglove.sh` | **操作PC**（PC2の橋をlocalhostへ引き込む） |
| `check_foxglove_stream.py` | **操作PC**（橋にデータが流れているかの切り分け） |

PC2側は初回だけ配る。`~/mapping_tools/`に置けば再起動しても消えない。

**以降のコマンドはすべて`Mapping/real`をカレントディレクトリとして書いてある。**

```bash
cd Mapping/real

ssh g1 'mkdir -p ~/mapping_tools'
scp quickstart/probe_dds_topics.py quickstart/record_dds_to_bag.py \
    quickstart/mapping_ctl.py quickstart/diagnose_ros2.sh \
    quickstart/start_foxglove_bridge.sh \
    quickstart/odom_to_tf.py quickstart/start_odom_tf.sh \
    quickstart/restamp_points.py quickstart/start_restamp.sh g1:~/mapping_tools/
ssh g1 'chmod +x ~/mapping_tools/*.py ~/mapping_tools/*.sh'

# ブラウザ可視化を使うなら、あわせてHumble環境も配る（第6節）
scp -r quickstart/humble_env g1:~/
```

`g1`は`~/.ssh/config`のホスト別名（`HostName 192.168.123.164` / `User unitree`）。
設定は`SETUP.md`を参照。

## 1. LiDAR / SLAM が動いているか確かめる

```bash
ssh g1 'python3 ~/mapping_tools/probe_dds_topics.py 5'
```

引数は購読秒数。**購読するだけで、ロボットには何も指令しない。**

```text
topic                                   count      Hz  detail
rt/utlidar/cloud_livox_mid360              50    9.96  frame_id=livox_frame points=20064 ...
rt/unitree/slam_mapping/points              0       -  受信なし
rt/unitree/slam_relocation/points           0       -  受信なし
rt/slam_info                               28    5.63  type=ctrl_info state=ready ctrName=not init
```

| 行 | 正常 | 意味 |
|---|---|---|
| `rt/utlidar/cloud_livox_mid360` | 約10Hz | LiDARドライバ稼働中。**SLAMとは独立に常時動く** |
| `rt/slam_info` | 約5Hz + `state=ready` | SLAMサービス稼働中・建図待ち |
| `rt/unitree/slam_mapping/points` | 建図中のみ | `0`は1801未実行というだけで異常ではない |

`ctrName=not init`は「地図未読込（1804前）」の目印。

> ⚠️ **`ros2 topic list`で死活確認をしないこと。** 2026-09-02にPC2で試したときは空に見え、
> 動いているサービスを「落ちている」と誤診した。**ただし当時の検証には穴がある**
> （壊れた`ros2 daemon`に問い合わせていた可能性。`## 6`を参照）。原因が確定するまでは、
> 死活確認は必ず上の`probe_dds_topics.py`で行うこと。

## 2. 記録する（PC2で実行）

```bash
# 60秒で自動停止
ssh g1 'python3 ~/mapping_tools/record_dds_to_bag.py --duration 60 --name room_a'

# Ctrl-Cで止めるまで記録する。-t が無いとCtrl-Cが届かない
ssh -t g1 'python3 ~/mapping_tools/record_dds_to_bag.py --name room_a'
```

出力の`session : 20260902T…_room_a`を以降で使う。

歩いて部屋全体の地図を作る場合は`--with-mapping`を付ける（`## 5. 歩いて地図を作る`を参照）。

### 既定で生データを並行記録する

トピックを指定しなければ**LiDARとIMUの両方**を記録する。

| モード | 既定トピック |
|---|---|
| 通常 | `/utlidar/cloud_livox_mid360` ＋ `/utlidar/imu_livox_mid360` |
| `--with-mapping` | 上記2つ **＋** `/unitree/slam_mapping/points` ＋ `/unitree/slam_mapping/odom`（計4本） |

> ⚠️ **2026-09-04に修正。** それ以前の`--with-mapping`は`if/else`の排他で、
> **生LiDARとIMUを1バイトも記録していなかった**。2026-09-03の`UiS_room_v1`は
> このためFAST-LIO2で検証し直せず、11.2mのずれの原因を切り分けられなかった。
> 内蔵SLAMの点群は姿勢が座標に焼き込み済みなので、**生データが無いと再処理の余地が無い**。

**IMUを取っておかないと、後からFAST-LIO2に掛け直せない。** 記録しなかったデータは
取り返しがつかないので既定に入れてある。odomは軌跡になり、`mapctl validate`の
`trajectory: 0 poses` のWARNも解消する。IMUは200Hzだが1メッセージが小さく、
帯域への影響は無視できる。

SDKに型が無いトピックは**警告を出して外し、残りで記録を続ける**（`--topic`で明示
指定した場合だけエラーで止まる）。`Imu_`/`Odometry_`がPC2のSDKに実在するかは
**未確認**なので、この作りにしてある。

主なオプション:

| オプション | 既定 | 内容 |
|---|---|---|
| `--topic` | 上の表 | ROS名で指定。複数可 |
| `--duration` | `0`（Ctrl-Cまで） | 秒 |
| `--backend` | `raw` | `mapctl rebuild`の既定トピック選択に効く |
| `--runs-dir` | `~/mapping_runs` | PC2側の保存先 |
| `--iface` | `eth0` | PC2のG1側NIC |

## 3. 回収して地図にする（操作PCで実行）

```bash
rsync -a -e ssh g1:/home/unitree/mapping_runs/<session_id> runs/
./mapctl rebuild  <session_id>      # --voxel 0.02 で解像度を上げられる
./mapctl validate <session_id>
```

`rebuild`と`validate`は`Mapping/real`本体のもので、**このディレクトリのために
変更していない**。

## 4. GUIで見る（操作PCで実行）

```bash
../../Navigation/.venv/bin/python quickstart/view_pcd_gui.py \
    runs/<session_id>/map/map_raw.pcd
```

左ドラッグ=回転 / 右ドラッグ=平行移動 / スクロール=ズーム / `R`=視点リセット /
`Q`・`ESC`=閉じる。色は高さZ、原点の座標軸は赤=X 緑=Y 青=Z。

Open3Dは`Navigation/.venv`に既に入っている（`pyproject.toml`の`pcd`グループ）。
無ければ`cd Navigation && uv sync --group pcd`。

## 5. 歩いて地図を作る（建図・**実機未検証**）

静止スキャンでは一箇所から見える範囲しか取れない。部屋全体の地図を作るには、G1内蔵の
SLAMに建図させ、**地図座標系**の点群`/unitree/slam_mapping/points`を記録する。

```bash
ssh -t g1 'python3 ~/mapping_tools/record_dds_to_bag.py --with-mapping --name room_a'
```

`--with-mapping`は3つを自動でやる。

1. 記録前に**1801（建図開始）**を投げる。失敗したら記録せず終了コード1で止まる
2. 既定トピックを`/unitree/slam_mapping/points`、backendを`onboard`に切り替える
3. 記録が終わったら**1802（建図終了・保存）**を投げる

**1802はCtrl-Cでも例外でも必ず投げる。** `finally`節に入れてある。2026-08-26の実機試験は
2回とも停止時に通信が切れて`kEndMapping`が届かず、PCDが書かれないまま
セッションがfailedになった。同じ失敗を繰り返さないための作りである。

1802が失敗しても記録は残るので、`./mapctl rebuild`で地図は作り直せる。結果は
`state.json`の`end_mapping`に残る。

### 通信が切れても1802を飛ばすために

レコーダは**SIGINT / SIGTERM に加えてSIGHUPも捕まえる**。SSHが切れるとsshdはSIGHUPを
送るが、既定の動作はプロセス即死で`finally`が走らない＝1802が飛ばない。WiFi運用では
現実的に起こるので捕まえている。

さらに**tmuxで走らせてSSHセッションと寿命を切り離す**のが安全。

```bash
ssh g1 'tmux new -d -s mapping "python3 ~/mapping_tools/record_dds_to_bag.py \
        --with-mapping --name room_a 2>&1 | tee ~/mapping.log"'

ssh g1 'tmux capture-pane -pt mapping | tail -5'   # 途中経過を見る
ssh g1 'tmux send-keys -t mapping C-c'             # 停止（1802が飛ぶ）
```

手順:

```bash
# 1. G1を立たせ、記録を開始する（1801が自動で飛ぶ）
ssh -t g1 'python3 ~/mapping_tools/record_dds_to_bag.py --with-mapping --name room_a'

# 2. 純正リモコンで低速に部屋の外周を回り、開始地点付近へ戻る
#    急旋回・足踏み・長時間の静止を避ける

# 3. Ctrl-C で停止する（1802が自動で飛ぶ）

# 4. 以降は静止スキャンと同じ
rsync -a -e ssh g1:/home/unitree/mapping_runs/<id> runs/
./mapctl rebuild  <id>       # backend=onboard なので自動で slam_mapping/points を使う
./mapctl validate <id>
../../Navigation/.venv/bin/python quickstart/view_pcd_gui.py runs/<id>/map/map_raw.pcd
```

### 個別に叩く場合

```bash
ssh g1 'python3 ~/mapping_tools/mapping_ctl.py status'   # サービスの状態を見る
ssh g1 'python3 ~/mapping_tools/mapping_ctl.py start'    # 1801
ssh g1 'python3 ~/mapping_tools/mapping_ctl.py stop --map /home/unitree/test1.pcd'  # 1802
ssh g1 'python3 ~/mapping_tools/mapping_ctl.py close'    # 1901 SLAM終了
```

### ⚠️ この経路は実機で一度も通していない

`mapping_ctl.py`は`backends/onboard_unitree/src/g1_onboard_lio.cpp`（**1801の成功実績が
ある実装**）のPython移植で、api-id・サービス名・バージョン・登録集合・リクエストJSONを
C++に合わせてある。偽SDKを注入した通し試験で配線（1801→記録→1802、失敗経路、
終了コード）も確認した。

⚠️ **C++との一致は人手で合わせただけで、自動では守られていない。**
`g1_onboard_lio.cpp`を変更したら`mapping_ctl.py`の定数も手で追随させること。
特にapi-idがずれると`RPC_ERR_CLIENT_API_NOT_REG(3103)`で機体まで届かない。

#### 2026-09-03に実機で確認できた分

`status` / `probe` / `start`(1801)は**実機で成功した**。

```text
[1801] 建図を開始します（slam_type=indoor）
[RPC] api_id=1801 rpc_code=0 response={"succeed":true,"errorCode":0,
      "info":"Successfully started mapping.","data":{}}
```

直後の観測で、未知だった2点も解消した:

- `rt/unitree/slam_mapping/points`が**10.00Hzで流れ出す**
  （`frame_id=map`、888点、`point_step=48`、
  `fields=[x y z intensity normal_x normal_y normal_z curvature]`）
- `rt/slam_info`に**`type=mapping_info`が混ざり始め**、レートが5.5Hz→15.4Hzに上がる。
  `ctrl_info`も並行して流れ続けるので「切り替わる」のではない

#### 1802も成功した（2026-09-03）

**2026-08-26に2回とも失敗して以来、誰も通していなかった経路が通った。**

```text
[1802] 建図を終了し保存します: /home/unitree/test1.pcd
[RPC] api_id=1802 rpc_code=0 response={"succeed":true,"errorCode":0,
      "info":"Save pcd successfully.","data":{}}
```

投げた直後に`slam_mapping/points`が止まり、`slam_info`が`ctrl_info` / `state=ready`へ
戻ることも確認した。

ただし**保存先PCDは回収できていない**。PC1(`192.168.123.161`)はpingには
0.5msで応答するが、22/21/23/80/445/873/2049/8080/8000/9000のいずれも閉じている
（MacからとPC2からの両方で確認）。

#### まだ通していないもの

- `--with-mapping`の通し実行（1801→記録→1802を1コマンドで）
- 歩行しながらの記録と、その結果の`rebuild`

`stop`(1802)を投げるまでG1は建図モードのまま。放置せず、
`ssh g1 'python3 ~/mapping_tools/mapping_ctl.py stop'`で閉じること。

## 6. ブラウザで見ながら歩く（pixi + Humble + foxglove_bridge・**実機で確認済み**）

`--with-mapping`は記録するだけで、**歩いている最中に地図がどう育っているかが見えない**。
終わって`rebuild`するまで、どこが抜けたか分からない。ここを埋める。

### ⚠️ PC2のfoxyはUnitreeのDDSに触れると必ずクラッシュする

**先に知っておくこと。`/opt/ros/foxy`のROS 2は、この機体では使えない。**

domain 0のeth0（＝Unitreeのdiscoveryが飛んでくる場所）に参加した瞬間、
`ros2 topic list`も`rviz2`も`rosbridge`も、例外なくSIGSEGVで落ちる。
2026-09-03にgdbで位置を特定した。

```text
Thread 3 "dq.builtins" received signal SIGSEGV
#2  ddsi_plist_init_frommsg ()   ← /opt/ros/foxy/.../libddsc.so.0 (0.7.0)
#4  builtins_dqueue_handler ()
```

`dq.builtins`はdiscoveryメッセージ専用の処理スレッド、`ddsi_plist_init_frommsg`は
受信した自己紹介パケットのパラメータ列を解釈する関数。つまり**foxy同梱の
cyclonedds 0.7.0が、Unitreeの0.10.2が撒いたパケットを読んでいる最中に落ちている**。

4条件の切り分けで、原因がdiscoveryの受信であることを確定させた:

| 条件 | Unitreeの自己紹介が届くか | 結果 |
|---|---|---|
| domain 0 / eth0 | **届く** | **SIGSEGV** |
| domain 77 / eth0 | 届かない（チャンネル違い） | 正常 |
| domain 0 / localhostのみ | 届かない | 正常 |
| domain 0 / wlan0 | 届かない（Unitreeはeth0側） | 正常 |

ノード生成そのものはどこでも成功する。**相手の自己紹介を受け取った瞬間だけ死ぬ。**

RTPSは「知らないパラメータは長さ分だけ読み飛ばす」ことを規格で要求しているので、
これは設計上の非互換ではなく**0.7.0側の実装バグ**。同じ機体で`unitree_sdk2py`
（cyclonedds 0.10.2を直接使う）が9.98Hzで受信し続けていることが、
0.10世代なら読めることの実測になっている。

foxyのもう一方のRMWであるFastDDS 2.0でも`bad_alloc`で落ちる。別ベンダの
独立した2実装が同じデータで揃って転ぶので、2020年頃の古いパーサに共通の甘さが
あった可能性が高い（断定はしていない）。

**したがって「rosbridgeを入れれば見える」は誤り。** rosbridgeもROS 2の購読に乗る以上、
同じ死に方をする。0.10世代のcyclonedds、つまりHumble以降を持ち込むしかない。

### なぜapt/Dockerではなくpixiなのか

Humbleは22.04専用で、20.04のPC2にaptでは入らない。取りうる道は3つ:

| 方法 | 可否 | 備考 |
|---|---|---|
| apt | **不可** | focal向けHumbleパッケージは存在しない |
| Docker | 可 | 公式イメージ。`network_mode: host`なら遅延ゼロ。sudo必要 |
| **pixi + RoboStack** | **採用** | root不要。プロジェクトローカルに閉じる |

pixiを選んだ理由は、`/opt/ros/foxy`にも既存のminiforgeにも一切触らずに済むこと。
`~/g1_humble/`の中で完結し、撤退はディレクトリ削除だけ。`pixi.lock`で版が固定される。

RoboStackはコミュニティによる再ビルドで、OSRF公式バイナリではない。ただし版番号は
upstreamと一致する（`rmw_cyclonedds_cpp 1.3.4`はHumbleの版そのもの）。

### 導入（PC2で実行・root不要）

```bash
# 操作PCから配布
scp -r quickstart/humble_env g1:~/
scp quickstart/start_foxglove_bridge.sh g1:~/mapping_tools/

# PC2で構築（約1.2GB。pixi本体はsha256を検証してから展開する）
ssh g1 'bash ~/humble_env/setup_pc2.sh'

# 動作確認（foxyがSIGSEGVした条件そのもの）
ssh g1 'cd ~/g1_humble && ~/.pixi/bin/pixi run ros2 topic list'
```

`/utlidar/cloud_livox_mid360`を含む130以上のトピックが並べば成功。

### 構成

```text
PC2: foxglove_bridge (ws://0.0.0.0:8765)   ~/g1_humble の pixi 環境で動く
              ↓ 有線（SSHトンネル。DDSは通さない）
Mac: ssh -N -L 8765:localhost:8765 g1
     ブラウザ → app.foxglove.dev → Open connection
              → **Foxglove WebSocket** → ws://localhost:8765
```

```bash
ssh g1 'bash ~/mapping_tools/start_foxglove_bridge.sh'   # PC2で橋を立てる
bash quickstart/tunnel_foxglove.sh                        # Mac側でトンネル
python3 quickstart/check_foxglove_stream.py /unitree/slam_mapping/points 5
```

**SSHトンネルを挟む理由**: `app.foxglove.dev`はHTTPSなので、素の`ws://`で
リモートへ繋ぐと混在コンテンツとして遮断される。`localhost`宛だけは例外なので通る。
Foxgloveのデスクトップ版を使うならトンネルは不要で、`ws://192.168.123.164:8765`に
直接繋げる。

**rosbridgeではなくfoxglove_bridgeを使う理由**: 生LiDARは441KB/フレームを10Hz。
rosbridgeはこれをJSON化するが、foxglove_bridgeは**CDRのまま**流す
（`encoding=cdr`を実測で確認）。

**foxglove_bridgeは0.8系に固定すること。** 3.x系は同梱の`libfoxglove.so`が
GLIBC 2.32以上を要求し、Ubuntu 20.04（2.31）では起動しない。0.8系はwebsocketpp
ベースで`libfoxglove.so`を持たないため動く。

起動時に`Failed to add channel`が185件ほど出るが**想定内**。`unitree_api` /
`unitree_go` / `unitree_hg`の独自メッセージ定義が環境に無いためで、点群・IMU・odomは
標準型なので影響しない。

### 何が流れているか（2026-09-03実測）

建図中は**G1内蔵SLAMが「今こう理解している」点群と自己位置がそのまま外へ出ている**。

| トピック | frame_id | 中身 | レート | 帯域 |
|---|---|---|---|---|
| `/utlidar/cloud_livox_mid360` | `livox_frame` | 生LiDAR 20,064点 / `point_step=22` | 10.13Hz | 4.5MB/s |
| `/unitree/slam_mapping/points` | **`map`** | 増分スキャン 888点 / `point_step=48` | 9.97Hz | 0.4MB/s |
| `/unitree/slam_mapping/odom` | — | G1の自己位置 | 約10Hz | — |
| `/utlidar/imu_livox_mid360` | — | LiDAR内蔵IMU | — | — |

地図側は`fields=[x y z intensity normal_x normal_y normal_z curvature]`で、
法線と曲率まで付いた48バイト/点。生LiDARとは別物なので混同しないこと。
帯域は生LiDARの1/10で、ブラウザに載せても軽い。

1801を投げると`rt/slam_info`に**`type=mapping_info`のメッセージが混ざり始め**、
レートが**5.5Hz → 15.4Hz**に上がる。`ctrl_info`も引き続き流れるので、
「`type`が入れ替わる」わけではない（観測するタイミングでどちらも見える）。
建図中かどうかの判定には、レートか`mapping_info`の出現有無を使うのが確実。

なお`mapping_ctl.py status`は`ctrl_info`の書式しか解釈しないので、`mapping_info`を
拾ったときは`state`が`None`と表示される（表示だけの問題）。

### G1はTFを出していない。odomから作る必要がある

**G1には`/tf`も`/tf_static`も無い**（2026-09-03に確認）。TFはROS 2で座標系どうしの
位置関係を時刻つきで配る仕組みで、これが無いと3Dパネルは:

- `map`座標系の`slam_mapping/points`は描けるが
- `livox_frame`座標系の生LiDARと重ねられない
- **G1が地図上のどこに居るのかも描けない**

Lichtblick/Foxglove の3Dパネルで **Display frame が空欄のまま赤いエラー**になるのは
これが原因。トピックを1つ有効にすると、そのメッセージの`frame_id`が
Display frameの候補に現れる。

一方`/unitree/slam_mapping/odom`（`nav_msgs/Odometry`）は自己位置を出しており、
中身はTFのtransformと同じ:

```text
frame_id: map            child_frame_id: base_link
position:    x=0.170 y=0.072 z=0.055
orientation: quaternion
```

`odom_to_tf.py`がこれを`/tf`に詰め替える。実測10.17Hz。

```bash
ssh g1 'bash ~/mapping_tools/start_odom_tf.sh'        # 開始
ssh g1 'bash ~/mapping_tools/start_odom_tf.sh stop'   # 停止
python3 quickstart/check_foxglove_stream.py /tf 5     # 確認
```

**`base_link -> livox_frame`は流していない。** LiDARの取付オフセットの実測値が
リポジトリにもPC2にも無いことを確認したため（`/unitree`配下にURDFもextrinsicも無い）。
推測値を入れると点群が静かにずれた場所に描かれ、誤りに気づけない。実測が得られたら
`--livox-xyz X Y Z`で与えれば生LiDARも重ねられるようになる。
なお`SimEnv3D`の`LIDAR_OFFSET_Z`は**シミュレータ用の仮定値**なので流用しないこと。

なお`nav_msgs/Odometry`は3Dパネルのトピック一覧には**出てこない**（描画可能な型だけが
並ぶため）。出ていないのではなく、一覧に載らないだけ。

### 地図点群のheader.stampが0。打ち直さないと蓄積できない

**`/unitree/slam_mapping/points`は`header.stamp`が0で配信されている**（2026-09-03に
CDR直読と`ros2 topic echo`の両方で確認）。

```text
/unitree/slam_mapping/points   stamp: sec=0            <- これ
/unitree/slam_mapping/odom     stamp: sec=1788406335   正常（同じSLAMが出している）
/utlidar/cloud_livox_mid360    stamp: sec=1788406334   正常
PC2の現在エポック                     1788406335
```

同じSLAMが出すodomには正しい時刻が入っているので、点群だけの不具合とみられる。

#### 影響範囲（Lichtblickのソースで確認・当初の説明は誤っていた）

`lichtblick/packages/suite-base/src/panels/ThreeDeeRender/renderables/pointExtensionUtils.ts`
を読むと、時刻の使い分けが2箇所で異なる。

```ts
// 期限切れ判定（decay）— receiveTime を使う
while (pointsHistory.length > 1 && pointsHistory[0]!.receiveTime < expireTime) { ... }

// 姿勢の更新（TF参照）— messageTime（headerの時刻）を使う
const srcTime = entry.messageTime;
const updated = updatePose(..., currentTime, srcTime);
if (!updated) { /* MISSING_TRANSFORM エラー */ }
```

つまり:

| 機能 | 使う時刻 | `stamp=0`の影響 |
|---|---|---|
| Decay（蓄積） | **receiveTime** | **影響なし。時刻0でも蓄積できる** |
| TF参照（姿勢） | messageTime | 変換が必要な場合に**失敗** |

したがって影響が出るのは**表示座標系が点群の座標系と異なるときだけ**。

| Display frame | 結果 |
|---|---|
| `base_link` | `map -> base_link`を時刻0で引いて**失敗**。描画されない |
| **`map`** | 点群と同じ座標系＝恒等変換。**時刻0でも問題ない見込み** |

**⚠️ 当初このREADMEには「時刻0ではDecayが効かない」と書いていたが、これは誤り。**
`receiveTime`で判定しているため蓄積は正常に働く。

#### restamp_points.py は不要かもしれない

上記より、**Display frameを`map`にするだけで足りた可能性が高い**。
2026-09-03の実機では`points_stamped`への切替・Display frameの変更・Decay time設定を
**同時に行った**ため、どれが効いたのか観測上は切り分けられていない。

**次回の確認手順（1分）**: 素の`/unitree/slam_mapping/points`を、
Display frame=`map` / Decay time=`300`で有効にする。蓄積されれば
`restamp_points.py`は撤去してよい。

```bash
ssh g1 'bash ~/mapping_tools/start_restamp.sh'        # 開始
ssh g1 'bash ~/mapping_tools/start_restamp.sh stop'   # 停止
# -> /unitree/slam_mapping/points_stamped
```

打ち直しが必要になった場合の注意: **受信時刻での代用であって真の観測時刻ではない。**
この点群は既に`map`座標系へ変換済みで表示時にTFを掛けないため位置には影響しないが、
**他センサとの同期や後処理でのLIO再計算には使わないこと**。その用途には各点に`time`を
持つ生LiDARを使う。

なお既製品としては`ros-humble-topic-tools 1.1.1`がRoboStackにあるが、
`relay`は無変換中継、`transform`はPython式で出力メッセージを組み立てる方式なので、
巨大な`data`配列を持つ`PointCloud2`の再構築には実用的でない。

### Lichtblick / Foxglove の設定（この通りにすれば映る）

2026-09-03に実機で地図が描画されるまでに必要だった設定。**4つ全部要る。**

| 場所 | 設定 | 値 | なぜ |
|---|---|---|---|
| Frame | **Display frame** | **`map`** | `base_link`はG1に固定された座標系。Follow mode=Poseだとカメラがロボットと一緒に動き、地図が溜まって見えない |
| Topics | `/unitree/slam_mapping/points_stamped` | 有効 | **ただし素の`points`でも足りる可能性が高い**（上記参照・要確認） |
| 同トピックを展開 | **Decay time** | `300` | 既定0では最新1フレームだけ。この点群は1回約900点の**増分**なので、溜めないと地図にならない |
| 同 | **Point size** | `2`〜`5` | 既定2でも見えるが、点が疎なので上げると見やすい |

補助的に効く設定:

- **Color mode** `Color map` / **Color by** `intensity` / **Color map** `Turbo`
  — 反射強度で色分けされ、構造が読み取りやすくなる
- **Follow mode** は`map`固定なら`Pose`のままでよい

**`/utlidar/cloud_livox_mid360`（生LiDAR）は無効にしておくこと。** `livox_frame`から
`map`への変換が無いため赤いエラーになり、Alertsが溜まり続ける。

接続は **Open connection → Foxglove WebSocket → `ws://localhost:8765`**。
Rosbridgeではない。デスクトップ版なら`ws://192.168.123.164:8765`に直接でもよい。

#### ビューアの入手

`app.foxglove.dev`（Web版）は**アカウント必須**。Foxgloveは2024年にStudioを
クローズドソース化し、リポジトリはアーカイブされた。

ログイン不要で使えるのが **Lichtblick**（コミュニティfork、活発に開発中）。
macOS版のdmgがあり、`Foxglove WebSocket`接続にそのまま対応する。
2026-09-03に v1.28.1 で動作確認。

**adhoc署名でApple公証を受けていない**（`spctl`は`rejected: no usable signature`）。
`curl`で取得すると検疫属性が付かずGatekeeperの警告が出ないので、その点は承知して使うこと。
sha512はリリースの`latest-mac.yml`と照合できる。

### ⚠️ ケーブルは抜けない（2026-09-03に前提が崩れた）

**当初「WiFiに移ればケーブルが不要になる」と書いていたが、これは成立しない。**

`Fujitsu_free_Wi-Fi`は**APのクライアント分離（プライバシーセパレータ）が有効**で、
Mac↔PC2が双方向で完全に遮断される。両端ともゲートウェイとインターネットには
出られるのに、クライアント同士だけが通らない。

| 経路 | 結果 |
|---|---|
| Mac → ゲートウェイ`.211.254` | OK 4.7ms |
| PC2 → ゲートウェイ`.211.254` | OK 2.7ms |
| PC2 → インターネット`8.8.8.8` | OK 8.3ms |
| Mac → PC2 `.211.16`（ping / TCP:22） | **不達** |
| PC2 → Mac `.210.1` | **不達** |
| Mac → 同一APの他クライアント6台 | **6/6全滅** |

無関係な他端末も全滅しているのでPC2固有の問題ではなく、こちら側では解除できない。
ゲスト用フリーWiFiとしては妥当な設定。

したがって**2026-08-26の「歩行中にケーブルが抜けて`kEndMapping`が届かない」失敗モードは
まだ消えていない**。回避するなら、AP分離の無いネットワーク（スマホのテザリング、
持ち込みルータ、あるいはPC2自身を`nmcli device wifi hotspot`でAP化する）が要る。
PC2のwlan0（`rtl8852bu`）はAPモードに対応しており、`dnsmasq-base`も導入済みなので
最後の手は使える見込み（未検証）。

### 限界: 蓄積ビューは後からの補正を反映しない

`slam_mapping/points`は**増分スキャン**なので、G1のLIOがループ閉じ込みや姿勢グラフ
最適化で過去の位置を修正しても、**こちらが既に描いた点は動かない**。長く歩いて一周
戻ったとき、G1が最終的に保存する地図と表示に差が出る可能性がある。

**「進捗の確認」には十分だが、「最終地図の正確なプレビュー」ではない。**
UnitreeのLIOが再最適化を配信するのかは未確認。

## 実測値（2026-09-02・G1を立たせて静止）

| 項目 | 実測 |
|---|---|
| `rt/utlidar/cloud_livox_mid360` | 9.98Hz / 20,160点 / `point_step=22` / `frame_id=livox_frame` |
| fields | `x y z intensity ring time` — **各点時刻あり** |
| 帯域 | 約4.4MB/s（60秒で265MB） |
| 60秒の記録 | 599スキャン、取りこぼし0件 |
| 回収 | 265MBが約2.5秒（有線） |
| `rebuild` | 1,200万点 → voxel 0.05m → 102,666点、約6秒 |
| `validate` | 4項目PASS（`trajectory: 0 poses`のWARNのみ） |
| 地図の範囲 | 21.2m × 20.3m × 7.7m |

## 落とし穴

### 生LiDARはセンサ座標系。歩きながら溜めても重ならない

`/utlidar/cloud_livox_mid360`の`frame_id`は`livox_frame`で、姿勢で変換しない限り
スキャン同士が重ならない。`Mapping/real/README.md`も同じ警告をしている。

- **静止して撮る** → そのまま有効な一枚になる
- **歩いて部屋の地図** → `/unitree/slam_mapping/points`（地図座標系）に切り替える。
  こちらは建図中しか流れないのでAPI 1801が要る

### LiDARは足元の床が見えない

実測した仰角は**-6.8°〜+52.1°**で、Livox Mid360の公称垂直FOV（-7°〜+52°）と一致する。
水平から7°下までしか届かないため、**センサ高hの約8.1×h手前までの床は死角**になる。

走査が上向きの扇形になるのはこの仕様によるもので、機体の姿勢のせいではない
（立たせる前後で変わらないことを実測で確認した）。避障をこの点群だけに頼る設計は
足元の障害物を取りこぼす。

### 1802が書くPCDはPC1にあり、回収できない

API 1802（建図終了・保存）が書き出す先は**PC1（`192.168.123.161`）のファイルシステム**。
PC1はSSHも主要ポートも閉じているため、ファイルを取り出す手段が無い
（`Navigation/README.md`の調査）。ナビゲーション自体は1804がPC1上の同じファイルを
読むので困らないが、**地図を持ち帰るにはこの記録経路しかない。**

### PC2のネットワーク

WiFi（`wlan0`）経由の外部DNSが断続的で、Docker Hubにも届かないことがある。
PC2でimageをpullしたりapt installしたりする前提の手順は組まないほうがよい。

## 検証の状況

| 対象 | 状況 |
|---|---|
| 生LiDARの記録 → `rebuild` → `validate` → GUI | **実機で実証済み**（2026-09-02） |
| `probe_dds_topics.py` | **実機で実証済み** |
| `mapping_ctl.py`の`status`/`probe`/`start`(1801) | **実機で実証済み**（2026-09-03） |
| 1801後に`slam_mapping/points`が流れること | **実機で実証済み**（10.00Hz、`frame_id=map`） |
| `mapping_ctl.py`の`stop`(1802) | **実機で実証済み**（2026-09-03。`Save pcd successfully.`） |
| `mapping_ctl.py`の`close`(1901) | **実機未検証** |
| `--with-mapping`の通し実行 | **実機未検証** |
| IMU/odomの記録 | **実機未検証。** 偽SDKで型解決・db3・metadata・rebuildのトピック選択は確認 |
| `Imu_`/`Odometry_`がPC2のSDKにあるか | **未確認。** ただしROS 2側では`/utlidar/imu_livox_mid360`と`/unitree/slam_mapping/odom`の実在を確認 |
| `diagnose_ros2.sh` | **実行済み。** foxyがSIGSEGVすることを特定（第6節） |
| foxyでのROS 2利用 | **不可と確定。** `ddsi_plist_init_frommsg`でSIGSEGV（gdbで確認） |
| pixi + Humble + foxglove_bridge | **実機で実証済み**（2026-09-03。生LiDAR 10.13Hz/4.5MB/s、地図 9.97Hz/0.4MB/s） |
| WiFi越しのMac→PC2疎通 | **不可と確定。** AP分離で双方向遮断（第6節） |

### オフラインで確認したこと（2026-09-03・手動）

偽の`unitree_sdk2py`を注入し、実機と同じCDRの点群を流して通し試験をした。

- 記録 → db3 → `mapctl rebuild` → PCD まで通ること
- `--with-mapping`で 1801 → 記録 → 1802 の順に投げること
- 点群とIMU/odomを**別のIDL型で同時に**記録し、db3のtopicsとmetadata.yamlに
  正しい型名が入ること。`rebuild`がIMUではなく点群トピックを選ぶこと
- `Imu_`が無いSDKでは警告して外し、点群だけで記録を続けること
  （`--topic`で明示指定した場合はエラーで停止）
- 投げるJSONと登録集合がC++と同じであること
  （`{"data":{"slam_type":"indoor"}}` / `{"data":{"address":...}}` /
  registered=`[1801,1802,1901]` / version=`1.0.0.1`）
- 1801失敗なら記録せず終了コード1で止まり、1802を呼ばないこと
- 1802失敗でも db3 と`metadata.yaml`が無事で、`state.json`に失敗が残ること

いずれも**自動テストとしては残していない**ので、変更したら手で確かめ直すこと。

### 実機で確認したこと（2026-09-03）

- **foxyはこの機体では使えない。** `ddsi_plist_init_frommsg`でSIGSEGV（gdbで位置特定）。
  4条件の切り分けでdiscoveryの受信が原因と確定
- **Humbleなら動く。** pixi + RoboStackで`~/g1_humble`に導入し、
  foxyが落ちたのと同一条件（domain 0 / eth0）で`ros2 topic list`が130以上を列挙
- **ブラウザ可視化が全段開通。** Unitree DDS → Humble → foxglove_bridge →
  SSHトンネル → Macのlocalhost。地図トピックで9.97Hz / 0.4MB/s、`encoding=cdr`
- **1801が成功し、`slam_mapping/points`が10.00Hzで流れ出す**
- **`rt/slam_info`に`mapping_info`が混ざり始める**（5.5Hz→15.4Hz。`ctrl_info`も継続）
- **1802が成功し、`Save pcd successfully.`が返る。** 2026-08-26以来の未踏経路。
  ただし保存先はPC1で、全ポートが閉じており回収できない
- **地図点群の`header.stamp`が0。** 打ち直さないと蓄積表示ができない
- **`Fujitsu_free_Wi-Fi`はAP分離でMac↔PC2が通らない。** ケーブルレス化は不成立
- **建図は放置すると自動停止する。** 2026-09-03に2回発生（約16分と20〜40分）。
  停止時のロボットは`sportMode=-1` / `gaitType=-1`＝**アクティブな動作モードに
  入っていない**状態だった（バッテリ38%、モータ異常なし）。原因は未特定だが、
  **歩かせる直前に`start`を投げ直すのが確実**
- **PC2にtmuxが入っていない。** 計画書のtmux前提の手順はそのままでは使えない
  （`nohup`で代替可能）

### 残っている未検証

- **1802の成功パスは実機で一度も通っていない**（2026-08-26は2回とも通信断）
- `--with-mapping`の通し実行（1801→記録→1802）
- 歩行しながらの記録と、その結果の`rebuild`
- SIGHUPを捕まえる実装は入れたが、実際にSSHを切って試してはいない
- `trajectory`は常に0 poses。odometryを記録していないため`validate`がWARNを出す
- PC2自身をAP化する退避策（wlan0のAPモード対応と`dnsmasq-base`の存在は確認済み）
