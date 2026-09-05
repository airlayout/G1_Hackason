# G1 Mapping 現場手順

## 1. 接続前

1. G1の周囲に転倒・衝突の危険がないことを確認する。
2. 現場PCとG1をEthernet接続する。
3. **ケーブルの取り回しを固定する。** 歩行に引かれて抜けると計測が壊れる。
   2026-08-26の試験では2回続けて途中で抜け、いずれも停止処理が失敗した（第6節）。
4. G1の移動には純正リモコンを使い、Mappingシステムから歩行指令を送らない。
5. LiDARを覆う物、鏡面物、動き続ける大きな物体を可能な範囲で避ける。

## 2. PC準備

```bash
cd Mapping/real
cp .env.example .env
```

`.env`の`G1_IFACE`を現場PCのNIC名に変更する。確認には`ip -br address`を使う。
カメラserverの配置と起動にはPC2へのSSHログインを使用する。パスワードや鍵は
リポジトリへ保存しない。

現場PCのstatic IPは`192.168.123.0/24`のうちG1側が使っていないアドレスにする。
G1のリンク上には本体以外の機器もいるため、`192.168.123.200`のような切りのよい
アドレスは既に埋まっていることがある（2026-08-26の試験では`.120` `.161` `.164` `.200`
の4台が応答し、`.200`を要求したNM接続はIP設定に失敗した）。空きの確認方法:

```bash
ip neigh show dev <NIC>          # 応答のあった機器を一覧する
ping -c1 -W1 192.168.123.<候補>  # 応答が無ければ空き
```

初回だけ以下を実行する。

```bash
./mapctl build
```

カメラ専用serverスクリプトをPC2へ配置する。これはカメラだけを起動し、歩行modeや
`slam_operate`を変更しない。

```bash
ssh unitree@192.168.123.164 'mkdir -p ~/mapping_tools'
scp quickstart/start_camera_only.sh quickstart/camera_only_server.py \
  unitree@192.168.123.164:~/mapping_tools/
ssh unitree@192.168.123.164 'chmod +x ~/mapping_tools/start_camera_only.sh'
```

オフラインfield kitを受け取った場合は、同梱の`install-field-kit.sh`を先に実行する。

## 3. 非破壊診断

先に別端末でカメラserverを起動し、計測終了まで開いておく。

```bash
ssh -t unitree@192.168.123.164 'bash ~/mapping_tools/start_camera_only.sh'
```

`run_g1_server.py --camera`は低レベル制御bridgeも同時に起動してmotion modeを解放するため、
内蔵LIO＋純正リモコンでのMappingには使用しない。

```bash
./mapctl doctor --backend auto
```

確認する内容:

- G1用NICと`192.168.123.0/24`のIPv4
- DockerとCompose
- DDSでの内蔵LIO応答
- 生LiDAR・IMUの型、受信周波数、点群fields
- 各点時刻の有無とLiDAR–IMU時刻差
- 保存先の空き容量
- LeRobot ImageServerのJPEG、撮影timestamp、解像度、受信周波数

`FAIL`を無視して開始しない。`WARN`は内容をセッションメモへ残す。

## 4. Mapping開始

最初は自動選択を使う。

```bash
./mapctl start --backend auto --name room_a
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

`stop`は地図保存を要求してからrosbagを停止する。途中で端末を閉じたりコンテナを強制終了
したりしない。

`validate`が成功し、`map/map_raw.pcd`と`raw/rosbag2/`が存在し、LiDAR・IMU・odometry・
登録点群・RGB・CameraInfo・camera metadataの件数がすべて1以上であることを確認する。

保存済み地図の再確認:

```bash
./mapctl view
```

## 6. 通信が切れて停止に失敗した場合

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

`runs/<session>/`をディレクトリごと持ち帰る。特に次を欠落させない。

- `manifest.json`
- `raw/rosbag2/`
- `map/map_raw.pcd`（生成できた場合）
- `logs/`
- `report/quality.json`
