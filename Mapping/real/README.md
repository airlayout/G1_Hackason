# Mapping / real

実機 G1 の LiDAR で計測を行う。**動作未検証**（実機で試すのはこれから）。

## 想定している運用

**G1 は手動で歩かせる**（リモコン等）。操作 PC は Ethernet ケーブルで繋いで
**点群を受け取るだけ**で、ロボットには一切コマンドを送らない。

この結果、実機側でやることは「LiDAR の点群を `scans.npz` にする」ことだけになる。
歩行制御・`LocoClient`・elastic band といった話は一切出てこない。
自己位置推定と地図構築は `Mapping/run_slam.py` が sim / real 共通で処理する。

```
G1（手動で歩行）── Ethernet ──> 操作PC
                                  │  record_scans_real.py  受信のみ
                                  ▼
                              scans.npz
                                  │  run_slam.py（sim と共通）
                                  ▼
                            map.ply（3D地図）
```

| ファイル | 役割 |
|---|---|
| `discover_topics.py` | **[エントリポイント]** G1 が流している DDS トピックを列挙する（受信のみ） |
| `record_scans_real.py` | **[エントリポイント]** LiDAR 点群を収録して `scans.npz` にする（受信のみ） |
| `pointcloud2.py` | `sensor_msgs/PointCloud2` を numpy の点群に変換する |

## 手順

### 1. ケーブルを繋いで疎通を確認する

```bash
bash Common/network/setup_ethernet_for_g1.sh
python Common/network/check_g1_connectivity.py
```

`READY` が出ればよい。詳細は [Common/network/README.md](../../Common/network/README.md)。

**`--check-bridge-ports` は不要。** あれは lerobot 方式（G1 本体側で
`run_g1_server.py` を動かす方式）専用で、今回は操作 PC から直接 DDS を
読むだけなので ping / SSH の確認だけで足りる。

### 2. LiDAR のトピック名を調べる

トピック名は `unitree_sdk2py` のどこにも書かれていない（同梱の LiDAR 関連定義は
Go2 用の `rt/utlidar/switch` だけ）。名前を推測して総当たりするより、DDS の
組み込みトピック（DCPSPublication）から**実際に publish されているものを
一覧する**ほうが確実なので、そうしている。

```bash
./G1_HuggingFace/venv/bin/python Mapping/real/discover_topics.py --network-interface enp3s0
```

`PointCloud2` 型のトピックがあれば候補として表示される。

### 3. 手動で歩かせながら収録する

```bash
./G1_HuggingFace/venv/bin/python Mapping/real/record_scans_real.py \
    --topic <手順2で見つけたトピック> --duration 90
```

収録中に G1 を手動で歩かせ、計測したい範囲をひと回りさせる。
シミュレーションでの実測では、水平 360 度の LiDAR なら 10m × 8m の部屋を
16.7m 歩くだけで全体が撮れた。壁際を舐めるより、**部屋の中を回って
同じ面を別角度から複数回見る**ほうが位置合わせが安定する。

### 4. 地図にする

```bash
./G1_HuggingFace/venv/bin/python Mapping/run_slam.py --scans Mapping/data/scans_real.npz
```

`gt_poses`（姿勢の真値）は実機には存在しないので `scans.npz` に入れていない。
`run_slam.py` はその場合、誤差評価を飛ばして地図だけ出す。

## 実機で確かめる必要があること

以下は**憶測で書かずに実機で確認する**。

1. **LiDAR のトピック名**（手順 2 で判明する）。
2. **LiDAR が最初から ON かどうか**。Go2 には `rt/utlidar/switch` で
   ON/OFF する仕組みがある。`discover_topics.py` でトピックが見つからない場合、
   OFF になっている可能性がある（ON にするのは書き込みなので、
   このフォルダのスクリプトには入れていない）。
3. **点群の座標系**。`record_scans_real.py` は「点群は LiDAR センサー座標系」
   という前提で `scans.npz` に入れる。もし既にロボット基準やオドメトリ基準に
   変換済みの点群が流れてくる場合、SLAM 側の前提と食い違う。
   受信時に表示される `frame_id` で判断できる。
4. **フレームレートと 1 フレームの点数**。`run_slam.py` は 10Hz 前後・
   1 フレーム 1 万点前後で調整してある（`--max-hz` で間引ける）。
   Livox 系は非反復スキャンで「1 フレーム」の切り方が機種依存になる。
5. **LiDAR の機種**。`Mapping/common/lidar_spec.py` の既定は仮置きの汎用
   回転式 LiDAR。実機の点群を直接使う実機側では spec は参照しないが、
   シミュレーションを実機に近づけたい場合はここを合わせる。
6. **LiDAR の取り付け位置・姿勢**（ロボット基準の外部パラメータ）。
   地図を作るだけなら不要だが、「地図とロボットの足元の位置関係」を
   出すには要る。

## 関連

- [Mapping/README.md](../README.md) — 方式とシミュレーションでの検証結果
- [Common/network/README.md](../../Common/network/README.md) — 通信・疎通確認
- [SimpleWalk/real/walk_forward_real_sdk.py](../../SimpleWalk/real/walk_forward_real_sdk.py) —
  操作 PC から直接 DDS に繋ぐ方式の実例（あちらは送信あり）
