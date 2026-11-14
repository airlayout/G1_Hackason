# G1 Mapping 現場手順

## 1. 接続前

1. G1の周囲に転倒・衝突の危険がないことを確認する。
2. 無線計測ではPCとG1を、端末間通信が可能な同じネットワークへ接続する（5GHz Wi-Fi推奨）。
   `.env`の`MAPPING_RUNTIME=native`、`MAPPING_TRANSPORT=ssh`を使う。
   初回構築・接続設定は[`real/README.md`](real/README.md)を参照。有線接続の場合はEthernetを使う。
3. 有線の場合は**ケーブルの取り回しを固定する。** 歩行に引かれて抜けると計測が壊れる。
   2026-08-26の試験では2回続けて途中で抜け、いずれも停止処理が失敗した（第6節）。
4. G1の移動には純正リモコンを使い、Mappingシステムから歩行指令を送らない。
5. LiDARを覆う物、鏡面物、動き続ける大きな物体を可能な範囲で避ける。

## 2. PC準備

リポジトリのルートから実行する。以降のPC側コマンドは`Mapping/real/`で実行する。

```bash
cd Mapping/real
test -f .env || cp .env.example .env
```

`.env`の`G1_IFACE`を現場PCのNIC名に変更する。確認には`ip -br address`を使う。
無線SSH方式ではWi-Fi NICとG1のWi-Fi IPを設定する。次のstatic IP設定は
有線方式向けなので、無線の場合は省略する。
G1の接続先は`G1_HOST`、SSHの接続先は`G1_SSH_USER`・`G1_SSH_HOST`、
カメラ配信先は`CAMERA_HOST`に指定する。
手動SSH例の`g1`は[`real/README.md`](real/README.md)で説明するSSHエイリアス。
パスワードや鍵はリポジトリへ保存しない。

現場PCのstatic IPは`192.168.123.0/24`のうちG1側が使っていないアドレスにする。
G1のリンク上には本体以外の機器も作業者もいるため、`.200`のような切りのよいアドレスは
既に埋まっていることがある（2026-08-26の試験では`.120` `.161` `.164` `.200`の4台が
応答し、`.200`を要求したNM接続はIP設定に失敗した）。

空きの選定はスクリプトに任せる。`arping`のDADで確認してから設定する。

```bash
bash ../../Common/network/setup_ethernet_for_g1.sh          # 空きを自動で選ぶ
bash ../../Common/network/setup_ethernet_for_g1.sh 222      # 明示するとき
```

自分で見るなら:

```bash
ip neigh show dev <NIC>                              # 応答のあった機器を一覧する
sudo arping -D -c2 -w2 -I <NIC> 192.168.123.<候補>   # 戻り値0なら空き
```

実行環境を`.env`の`MAPPING_RUNTIME`で選ぶ。Docker方式は`docker`、
既存のUbuntu 24.04＋ROS 2 Jazzyを使う場合は`native`。
nativeの前提・構築内容は[`real/README.md`](real/README.md)を参照。
接続設定を済ませ、初回と関連コードの更新後に以下を実行する。

```bash
./mapctl build
```

カメラ配信を準備する。次はG1上のLeRobot環境を使うImageServerの配置例。
Innomakerカメラを使う場合は[`real/README.md`](real/README.md)の専用server手順を使う。
カメラ専用serverは歩行modeや`slam_operate`を変更しない。

```bash
ssh g1 'mkdir -p ~/mapping_tools'
scp quickstart/start_camera_only.sh quickstart/camera_only_server.py \
  g1:~/mapping_tools/
ssh g1 'chmod +x ~/mapping_tools/start_camera_only.sh'
```

オフラインfield kitを受け取った場合は、同梱の`install-field-kit.sh`を先に実行する。

## 3. 非破壊診断

カメラserverが既に配信中ならそのまま利用する。未起動の場合は、
先に別端末で使用するカメラserverを起動し、計測終了まで開いておく。
以下はLeRobot ImageServerの例（G1上に`lerobot` conda環境が必要）。

```bash
ssh -t g1 'bash ~/mapping_tools/start_camera_only.sh'
```

`run_g1_server.py --camera`は低レベル制御bridgeも同時に起動してmotion modeを解放するため、
内蔵LIO＋純正リモコンでのMappingには使用しない。

```bash
./mapctl doctor --backend raw
```

確認する内容:

- 指定したG1用NICと接続経路（有線DDSの場合は`192.168.123.0/24`のIPv4）
- DockerとCompose、またはnativeのROS 2 overlay
- DDSでの内蔵LIO応答（`onboard`を使う場合）
- 生LiDAR・IMUの型、受信周波数、点群fields
- 各点時刻の有無とLiDAR–IMU時刻差
- 保存先の空き容量
- LeRobot ImageServerのJPEG、撮影timestamp、解像度、受信周波数

`FAIL`を無視して開始しない。`WARN`は内容をセッションメモへ残す。

## 4. Mapping開始

FAST-LIO2＋OctoMapを使う場合は`raw`を明示する。
`auto`は内蔵LIOを優先するため、この手法を使う計測では指定しない。

```bash
./mapctl start --backend raw --name room_a
```

開始後、G1を5〜10秒静止させてIMU初期化を待つ。その後、低速で部屋の外周を回り、
最後に開始地点付近へ戻る。急旋回、足踏み、同じ場所での長時間静止を避ける。

状態確認:

```bash
./mapctl status
```

別のGUI端末からライブ地図を確認:

```bash
./mapctl view --live
```

`Density Map`は、同じ5cmボクセルを何スキャンから観測できたかを青→緑→黄→赤で示す。
青い領域が広く残る場合は、急旋回せず別角度からもう一度観測する。色の上限は
`.env`の`DENSITY_TARGET_SCANS`（既定10スキャン）で調整できる。

RVizを閉じてもMapping処理は継続する。反対に、Mapping停止後もRVizは自動終了しないため、
確認が終わったらRVizウィンドウを閉じる。

## 5. 停止・保存

```bash
./mapctl stop
./mapctl validate
```

native＋SSH＋rawでは、`stop`がG1原本の回収と最終地図の再生成まで実行する。
その他の方式では地図保存を要求してからrosbagを停止する。
途中で端末を閉じたりコンテナを強制終了したりしない。

`validate`が成功し、`map/map_raw.pcd`と`raw/rosbag2/`が存在し、LiDAR・IMU・odometry・
登録点群・RGB・CameraInfo・camera metadataの件数がすべて1以上であることを確認する。
native＋SSH＋rawでは、回収した原本`raw/g1/rosbag2/`と最終地図の
`map/map_raw.bt`・`map/map_raw.octomap.json`も確認する。

保存済み地図の再確認:

```bash
./mapctl view
```

## 6. 通信が切れて停止に失敗した場合

### native＋SSH＋rawの場合

接続を戻して次を実行する。G1原本の回収と最終地図の再生成を再開できる。

```bash
./mapctl recover
./mapctl validate
./mapctl view
```

### その他の方式の場合

以下のコンテナ終了操作はDocker方式向け。native方式では`mapctl stop`が
ROSプロセスとrecorderを順に正常終了させ、保存に失敗した場合は記録からの
OctoMap再生成を試みる。停止結果と`logs/`を確認し、必要なら同じ`mapctl rebuild`を使う。

`stop`の途中でEthernetが切れると、G1へ「Mapping終了」が届かず次の状態になる。
2026-08-26の初回試験で2回続けて発生した。

- G1側にPCDが書き出されない（`/home/unitree/maps/`に何も残らない）
- rosbagが閉じられず`metadata.yaml`が生成されない
- セッションの状態が`failed`になる

**この状態でもデータは失われていない。** 次の順で復旧する。

まずコンテナを**正常終了**させる。強制終了(`kill -9`)してはいけない。SIGTERMを受けると
rosbag2が自分で`metadata.yaml`を書き出す。

```bash
docker stop -t 30 \
  g1-mapping-session_recorder-1 \
  g1-mapping-session_trajectory-1 \
  g1-mapping-onboard_pipeline-1 \
  g1-mapping-camera_bridge-1
```

次にrosbagから地図を作り直し、検証する。

```bash
./mapctl rebuild <session_id>
./mapctl validate <session_id>
```

`db3`さえ残っていれば`metadata.yaml`がなくても`rebuild`は動く。

復旧後、次の計測を始める前にG1側のSLAMが待機状態へ戻っているかを確認する。
終了コマンドが届いていないため、Mapping中のまま残っている可能性がある。

```bash
./mapctl doctor --backend auto     # slam_infoのstateがreadyであること
```

## 7. backendを切り替える場合

内蔵LIOが失敗した場合:

```bash
./mapctl stop --allow-partial
./mapctl start --backend raw --name room_a_raw
```

同一セッション中には切り替えない。座標系とLIO内部状態を混ぜないためである。

## 8. 持ち帰るもの

`[OUTPUT]`に表示されたセッションディレクトリを丸ごと持ち帰る。
保存先は`MAPPING_RUNS_DIR`で指定し、既定は`runs/<session>/`。特に次を欠落させない。

- `manifest.json`
- `raw/rosbag2/`
- `raw/g1/rosbag2/`（native＋SSH＋rawで回収した原本）
- `map/map_raw.pcd`（生成できた場合）
- `logs/`
- `report/quality.json`
