# Mapping（G1による部屋空間の計測・作成）

G1の3D LiDAR・IMU・頭部RGBを使い、部屋の点群地図・軌跡・再処理可能な同期データを作る機能。
実機用は次の2方式を、同じ操作と成果物形式で切り替えられる。

- `onboard` — G1内蔵のUnitree LIO/SLAMサービスを利用
- `raw` — G1の生LiDAR・IMUをROS 2版FAST-LIO2で処理

どちらの方式でも、生LiDAR・IMU・RGB・CameraInfoと正規化済みLIO出力を同じrosbag2へ
記録する。現場ではライブ地図・自己位置・RGBを確認し、3DGSは持ち帰ってから処理する。

RViz2は可視化専用コンテナへ分離している。Mapping中のライブ点群と、保存後の
`map_raw.pcd`を同じ表示設定で確認できる。

## 構成

- [`real/`](real/README.md) — 実機用の共通CLI、2つのbackend、Docker配備、テスト
- [`sim/`](sim/README.md) — Isaac Simから実機と同じtopic契約でFAST-LIO2へ入れる経路
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — システム境界と成果物契約
- [`FIELD_RUNBOOK.md`](FIELD_RUNBOOK.md) — 現場で上から順に実行する手順
- [`FAILURES.md`](FAILURES.md) — 実測で判明した失敗と再発防止策

MappingのROS 2環境はDocker内へ隔離する。LeRobotによる歩行環境
`G1_HuggingFace/venv/`とは依存関係を共有しない。

## 現在の状態

**2026-08-26に実機で初回試験を実施した。** `mapctl doctor`が両backendともREADYになり、
G1内蔵LIOサービスの応答とセンサートピックを確認できた。計測・記録も動作した。

確定した実機の事実:

| 項目 | 実測値 |
|---|---|
| 生LiDAR | `/utlidar/cloud_livox_mid360` @ 9.97Hz |
| 生IMU | `/utlidar/imu_livox_mid360` @ 200.06Hz |
| 内蔵LIOの地図点群 | `/unitree/slam_mapping/points` @ 9.96Hz、`frame_id=map` |
| 内蔵LIOのodometry | `/unitree/slam_mapping/odom` @ 9.96Hz |
| 内蔵LIOサービス | 応答あり（`rt/slam_info`、待機時は`state: ready`） |

`.env.example`のトピック名は**既定値のまま実機と一致した**（ファーム差の吸収は不要だった）。

内蔵LIOの地図点群は「地図座標系へ変換済みの増分スキャン」で、1メッセージあたり
672〜1090点、`point_step=48`、fieldsは`x y z intensity normal_x normal_y normal_z curvature`。
地図全体が毎回来るわけではないので、蓄積してはじめて地図になる。

**まだ検証できていないこと:**

- **`stop`の正常完了とG1側へのPCD書き出し**。試験では2回とも停止時に通信が切れており、
  G1へ`kEndMapping`が届かなかった。そのため`/home/unitree/maps/`にPCDは生成されていない
  （実機を全探索して確認済み）。この経路は次回の試験で確かめる。
- **`raw` backendの実走**。`doctor`はPASSするが、FAST-LIO2で実際に地図を作ってはいない。
  LiDAR–IMU外部パラメータと時刻同期の詰めはこの段階で必要になる。

Isaac Simでは`sim` backendとして、Mid-360形式点群、200Hz IMU、RGB、CameraInfoを
同一simulation clockで配信し、実機rawと同じFAST-LIO2・recorderへ接続できる。

なお、通信断で地図が保存できなくてもrosbagから作り直せる（`mapctl rebuild`）。
実際に324,112点の地図を再構成して`mapctl validate`が全項目PASSすることを確認した。
詳細は[`FAILURES.md`](FAILURES.md)を参照。
