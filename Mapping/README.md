# Mapping（G1による部屋空間の計測・作成）

G1の3D LiDARとIMUを使い、部屋の点群地図・軌跡・再処理可能な生データを作る機能。
実機用は次の2方式を、同じ操作と成果物形式で切り替えられる。

- `onboard` — G1内蔵のUnitree LIO/SLAMサービスを利用
- `raw` — G1の生LiDAR・IMUをROS 2版FAST-LIO2で処理

どちらの方式でも、取得できる生データはrosbag2へ並行記録する。現場でオンライン地図が
完成しなくても、記録を持ち帰って再処理できることを優先している。

RViz2は可視化専用コンテナへ分離している。Mapping中のライブ点群と、保存後の
`map_raw.pcd`を同じ表示設定で確認できる。

## 構成

- [`real/`](real/README.md) — 実機用の共通CLI、2つのbackend、Docker配備、テスト
- [`sim/`](sim/README.md) — シミュレーション側の将来の入力アダプタ
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — システム境界と成果物契約
- [`FIELD_RUNBOOK.md`](FIELD_RUNBOOK.md) — 現場で上から順に実行する手順
- [`FAILURES.md`](FAILURES.md) — 実測で判明した失敗と再発防止策

MappingのROS 2環境はDocker内へ隔離する。LeRobotによる歩行環境
`G1_HuggingFace/venv/`とは依存関係を共有しない。

## 現在の状態

実機非接続で実装・mock検証済み。実機固有のトピック名、PointCloud2 fields、時刻同期、
LiDAR–IMU外部パラメータは、最初の現場試験で`mapctl doctor`の結果を基に確定する。
