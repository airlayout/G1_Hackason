# G1 Mapping システムアーキテクチャ

## 結論

操作・記録・成果物検証を共通化し、LIOの実行部分だけをbackendとして交換する。
Mapping中にbackendを切り替えることはせず、失敗時は新しいセッションとして再開する。

```text
                         onboard backend ─ Unitree slam_operate ─ G1内蔵LIO
                       /
mapctl ─ session manager
                       \
                         raw backend ─ PointCloud2 adapter ─ FAST-LIO2

           ├─ rosbag2 recorder
           ├─ trajectory writer
           └─ PCD/session validator

別プロセス: visualization ─ RViz2
                         └─ 保存PCD publisher
```

Mappingプロセスは歩行指令を送らない。最初の現場試験ではUnitree純正リモコンで移動し、
地図作成と移動制御を分離する。

## 共通操作契約

`real/mapctl`が唯一の運用エントリーポイントになる。

```bash
./mapctl doctor --backend auto
./mapctl start --backend auto --name room_a
./mapctl status
./mapctl stop
./mapctl validate
```

`auto`は内蔵LIOを先にprobeし、利用できなければraw backendをprobeする。明示的に
`--backend onboard`または`--backend raw`を指定することもできる。

現場で地図を保存できなかったセッションは、持ち帰ってから作り直す。

```bash
./mapctl rebuild [session_id]
```

これはrosbagに記録済みの地図点群を蓄積して`map/map_raw.pcd`を書き出す**オフライン
経路**であり、G1にもDockerにもROS 2にも依存しない。`db3`だけを読むため、異常終了で
`metadata.yaml`が欠けたセッションからも復旧できる。「生データを並行記録しておけば
現場で地図が完成しなくても持ち帰れる」という前提を、実際に成立させるための実装。

## 可視化境界

可視化は`g1-mapping-visualization` imageへ分離し、Mapping backendのコンテナへGUI、
X11ソケット、RViz依存を持ち込まない。

- ライブ: backendが配信する点群をRViz2が直接購読する。
- 保存済み地図: `pcl_ros/pcd_to_pointcloud`がPCDを`/g1_mapping/map`へ再配信する。
- raw backendの既定Fixed Frameは`camera_init`、保存PCDは`map`とする。
- onboardの実トピックはROS remappingで共通表示名へ接続する。
- 保存PCD表示ではCycloneDDSをloopbackへ限定し、G1用NICがない開発PCでも動かす。

操作は`mapctl view --live`と`mapctl view [session_id]`へ集約する。RVizを終了しても
Mappingプロセスには影響せず、Mappingが停止しても可視化プロセスを強制終了しない。

## backend契約

### onboard

- Unitree SDK2のDDS RPCで`slam_operate`へ接続する。
- Mapping開始はAPI ID `1801`、終了・PCD保存は`1802`を使う。
- PCDはまずG1上へ保存し、停止処理でSSH/SCPにより現場PCへ回収する。
- `slam_info`または`slam_key_info`を受信できることを、非破壊probeの成功条件にする。

### raw

- G1が配信する`PointCloud2`と`Imu`を購読する。
- `PointCloud2`をLivox `CustomMsg`へ変換し、各点時刻を保持してFAST-LIO2へ渡す。
- 各点時刻が無い場合は、点の並び順から1スキャン内の時刻を推定する縮退モードを持つ。
  縮退モードは診断レポートで明示し、正常扱いにはしない。
- FAST-LIO2の`map_save`サービスで、セッション配下へPCDを保存する。

## 標準トピック

| 用途 | トピック | 型 |
|---|---|---|
| 正規化済みLiDAR | `/g1_mapping/livox` | `livox_ros_driver2/msg/CustomMsg` |
| 正規化済みIMU | `/g1_mapping/imu` | `sensor_msgs/msg/Imu` |
| LIO odometry | `/g1_mapping/odom` | `nav_msgs/msg/Odometry` |
| 登録済み点群 | `/g1_mapping/cloud_registered` | `sensor_msgs/msg/PointCloud2` |
| 地図点群 | `/g1_mapping/map` | `sensor_msgs/msg/PointCloud2` |

実機入力・内蔵LIO出力のトピック名は`.env`で変更できる。ファームウェア差をソースコードに
埋め込まない。

## セッション成果物契約

```text
runs/<timestamp>_<name>/
├── manifest.json
├── state.json
├── raw/rosbag2/
├── map/map_raw.pcd
├── trajectory/trajectory.tum
├── logs/
└── report/quality.json
```

`manifest.json`にはbackend、Git revision、入力トピック、G1接続先、開始時刻を保存する。
`state.json`は`created/running/stopping/completed/failed`の状態遷移を記録する。

## 依存関係と再現性

- ホスト: Docker Engine、Docker Compose、SSH client、Python 3.10以上
- コンテナ: Ubuntu 22.04、ROS 2 Humble、CycloneDDS
- 外部ソース: `real/vendor/mapping.repos`とDocker build argsでcommitを固定
- DDS: host network、domain 0、`.env`で指定したNICだけを使用

現場にインターネットがない場合は`make-field-kit.sh`でDocker imagesを含む配備物を作る。

## 初回現場試験で確定する値

推測値をコードへ固定せず、診断結果とセッションmanifestに残してから設定を更新する。

2026-08-26の初回試験で確定した分:

- **トピック名と型・受信周波数** — `/utlidar/cloud_livox_mid360`(PointCloud2) 9.97Hz、
  `/utlidar/imu_livox_mid360`(Imu) 200.06Hz、`/unitree/slam_mapping/points`(PointCloud2)
  および`/unitree/slam_mapping/odom`(Odometry) 各9.96Hz。`.env.example`の既定値と一致した。
- **内蔵LIO出力のfields** — `x y z intensity normal_x normal_y normal_z curvature`、
  `point_step=48`、`frame_id=map`。地図全体ではなく、地図座標系へ変換済みの増分スキャンで
  1メッセージ672〜1090点。

未確定のまま残っている分:

- PointCloud2の各点時刻の単位（`raw` backendを実走させるまで確定しない）
- LiDARとIMUのheader時刻差
- `T_imu_lidar`と`T_base_lidar`
- **G1上で内蔵LIOがPCDを書き込めるディレクトリ** — 試験では停止時に通信が切れ、
  `kEndMapping`が届かなかったため`/home/unitree/maps/`は作られなかった。書き込み先も
  権限も未確認のまま。
