# G1 Mapping 現場手順

## 1. 接続前

1. G1の周囲に転倒・衝突の危険がないことを確認する。
2. 現場PCとG1をEthernet接続する。
3. G1の移動には純正リモコンを使い、Mappingシステムから歩行指令を送らない。
4. LiDARを覆う物、鏡面物、動き続ける大きな物体を可能な範囲で避ける。

## 2. PC準備

```bash
cd Mapping/real
cp .env.example .env
```

`.env`の`G1_IFACE`を現場PCのNIC名に変更する。確認には`ip -br address`を使う。
SSH鍵を使う場合は`G1_SSH_KEY`も設定する。

初回だけ以下を実行する。

```bash
./mapctl build
```

オフラインfield kitを受け取った場合は、同梱の`install-field-kit.sh`を先に実行する。

## 3. 非破壊診断

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

RVizを閉じてもMapping処理は継続する。反対に、Mapping停止後もRVizは自動終了しないため、
確認が終わったらRVizウィンドウを閉じる。

## 5. 停止・保存

```bash
./mapctl stop
./mapctl validate
```

`stop`は地図保存を要求してからrosbagを停止する。途中で端末を閉じたりコンテナを強制終了
したりしない。

`validate`が成功し、`map/map_raw.pcd`と`raw/rosbag2/`が存在することを確認する。

保存済み地図の再確認:

```bash
./mapctl view
```

## 6. backendを切り替える場合

内蔵LIOが失敗した場合:

```bash
./mapctl stop --allow-partial
./mapctl start --backend raw --name room_a_raw
```

同一セッション中には切り替えない。座標系とLIO内部状態を混ぜないためである。

## 7. 持ち帰るもの

`runs/<session>/`をディレクトリごと持ち帰る。特に次を欠落させない。

- `manifest.json`
- `raw/rosbag2/`
- `map/map_raw.pcd`（生成できた場合）
- `logs/`
- `report/quality.json`
